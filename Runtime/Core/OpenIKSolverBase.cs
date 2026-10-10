using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;

namespace OpenIK
{
    /// <summary>Selects whether a solver writes each solution to the chain's transforms.</summary>
    public enum ApplyMode
    {
        /// <summary>Apply every solution to the transforms, moving speed-limited joints at their capped speed.</summary>
        Automatic,

        /// <summary>
        /// Only solve. Read <see cref="OpenIKSolverBase.LastOutput"/>, or call
        /// <see cref="OpenIKSolverBase.Apply"/> to write it to the transforms.
        /// </summary>
        Manual
    }

    /// <summary>Selects when a solver runs.</summary>
    public enum UpdateMode
    {
        /// <summary>Step the solver every frame in LateUpdate with scaled delta time.</summary>
        LateUpdate,

        /// <summary>
        /// Never step automatically. Call <see cref="OpenIKSolverBase.Step"/>, or
        /// <see cref="OpenIKSolverBase.Solve"/> and <see cref="OpenIKSolverBase.Apply"/>, from your own code.
        /// </summary>
        Manual
    }

    /// <summary>The target's world-space pose, captured once at the start of a solve.</summary>
    public readonly struct IKGoal
    {
        /// <summary>World-space position the end effector moves toward.</summary>
        public readonly Vector3 Position;

        /// <summary>World-space rotation, used by solvers with an orientation objective.</summary>
        public readonly Quaternion Rotation;

        public IKGoal(Vector3 position, Quaternion rotation)
        {
            Position = position;
            Rotation = rotation;
        }

        /// <summary>The goal's forward axis (+Z) in world space.</summary>
        public Vector3 Forward => Rotation * Vector3.forward;
    }

    /// <summary>What a solver's SolveChain returns.</summary>
    public readonly struct SolveResult
    {
        /// <summary>Number of iterations the solver ran.</summary>
        public readonly int Iterations;

        /// <summary>
        /// Remaining error against the goal, in the same units as the solver's tolerance.
        /// The solve counts as converged when this is at most the solver's Tolerance.
        /// </summary>
        public readonly float Error;

        public SolveResult(int iterations, float error)
        {
            Iterations = iterations;
            Error = error;
        }
    }

    /// <summary>
    /// Base class for OpenIK solver components. It owns the chain setup, the per-frame update, and
    /// the application of solutions to the transforms, so a solver only implements SolveChain.
    /// </summary>
    /// <remarks>
    /// Each <see cref="Step"/> first calls <see cref="Solve"/>, which computes <see cref="LastOutput"/>
    /// without touching the transforms. When ApplyMode is Automatic, it then calls
    /// <see cref="Apply"/>, which moves the transforms toward that solution within the joints'
    /// speed limits.
    /// </remarks>
    public abstract class OpenIKSolverBase : MonoBehaviour
    {
        [Header("Execution")]
        [Tooltip("When the solver runs. LateUpdate steps it every frame. Manual leaves it to scripts, which should call Step, or Solve and Apply.")]
        [SerializeField] private UpdateMode updateMode = UpdateMode.LateUpdate;

        [Tooltip("Automatic writes each solution to the chain's transforms, moving joints with Limit Speed enabled at their capped speed. " +
                 "Manual only solves; other scripts read LastOutput or call Apply.")]
        [FormerlySerializedAs("mode")]
        [SerializeField] private ApplyMode applyMode = ApplyMode.Automatic;

        [Header("Chain")]
        [Tooltip("Transform the end effector moves toward.")]
        [SerializeField] private Transform target;

        [Tooltip("Joint transforms ordered from the root to the end effector.")]
        [SerializeField] private List<Transform> chainJoints = new();

        [Header("Solving")]
        [Tooltip("Stop iterating when the end effector is this close to the target, in world units.")]
        [Min(0f)]
        [SerializeField] private float tolerance = 0.001f;

        [Tooltip("Maximum solver iterations per frame. Higher values allow more refinement but cost more processing time. Stops early when Tolerance is reached.")]
        [Min(0)]
        [SerializeField] private int maxIterations = 10;

        [Tooltip("Start each solve from the joint rotations and segment offsets captured at initialization, instead of from the current pose.")]
        [SerializeField] private bool solveFromRestPose;

        [Tooltip("Coordinate speed-limited joints so they reach the solved pose together. Faster joints slow down to match " +
                 "the slowest; each joint stays within its own speed limit.")]
        [SerializeField] private bool synchronizeLimitedJoints;

        [Tooltip("Skip per-frame joint configuration updates when constraints and speed limits stay unchanged.")]
        [SerializeField] private bool staticSolverConfiguration;

        [Tooltip("Choose when to draw the chain's joints and bone connections.")]
        [SerializeField] private GizmoDrawMode boneGizmoMode = GizmoDrawMode.SelectedOnly;

        private readonly SolverChain _chain = new();
        private readonly IKSolverOutput _output = new();
        private readonly IKPoseApplier _applier = new();
        private IKApplicationStatus _applicationStatus = IKApplicationStatus.NotApplied;
        private bool _isInitialized;

        // True while speed-limited joints lag behind LastOutput, so the next solve starts from it.
        private bool _hasLaggingSolution;

        // Root-parent rotation that LastOutput was solved with.
        private Quaternion _solvedRootParentRotation = Quaternion.identity;

        /// <summary>
        /// Fires after every solve, before the solution is applied. Subscribers receive the solver's
        /// reused IKSolverOutput; copy any data you need to keep.
        /// </summary>
        public event Action<IKSolverOutput> Solved;

        /// <summary>
        /// Fires after a solution is written to the transforms, by <see cref="Apply"/> or
        /// <see cref="SnapToSolution"/>, with the resulting <see cref="ApplicationStatus"/>.
        /// </summary>
        public event Action<IKApplicationStatus> Applied;

        /// <summary>When the solver runs.</summary>
        public UpdateMode UpdateMode
        {
            get => updateMode;
            set => updateMode = value;
        }

        /// <summary>Whether <see cref="Step"/> writes each solution to the transforms.</summary>
        public ApplyMode ApplyMode
        {
            get => applyMode;
            set => applyMode = value;
        }

        /// <summary>Transform the end effector moves toward. The solver skips solving while it is null.</summary>
        public Transform Target
        {
            get => target;
            set => target = value;
        }

        /// <summary>The authored joint transforms, in root-to-end order.</summary>
        public IReadOnlyList<Transform> ChainJoints => chainJoints;

        /// <summary>Error at which the solver stops iterating and reports convergence.</summary>
        public float Tolerance
        {
            get => tolerance;
            set => tolerance = Mathf.Max(0f, value);
        }

        /// <summary>Maximum number of iterations per solve.</summary>
        public int MaxIterations
        {
            get => maxIterations;
            set => maxIterations = Mathf.Max(0, value);
        }

        /// <summary>When true, each solve starts from the rest pose instead of the current pose.</summary>
        public bool SolveFromRestPose
        {
            get => solveFromRestPose;
            set => solveFromRestPose = value;
        }

        /// <summary>
        /// When true, <see cref="Apply"/> slows speed-limited joints that cannot reach the solved pose
        /// in one step by a common factor, so they arrive together.
        /// </summary>
        public bool SynchronizeLimitedJoints
        {
            get => synchronizeLimitedJoints;
            set => synchronizeLimitedJoints = value;
        }

        /// <summary>
        /// When true, joint constraints and speed limits are read once at initialization and later
        /// changes to them are ignored.
        /// </summary>
        public bool StaticSolverConfiguration
        {
            get => staticSolverConfiguration;
            set => staticSolverConfiguration = value;
        }

        /// <summary>When the Scene view draws the chain's joints and bone connections.</summary>
        public GizmoDrawMode BoneGizmoMode => boneGizmoMode;

        /// <summary>
        /// The runtime chain: per-joint solver state and the constraints bound to each joint.
        /// Empty until the solver initializes.
        /// </summary>
        public SolverChain Chain => _chain;

        /// <summary>
        /// Output of the most recent solve: the solved pose and its convergence statistics. While
        /// joint speed limits are active, the transforms can still be moving toward this pose; see
        /// <see cref="ApplicationStatus"/>.
        /// </summary>
        public IKSolverOutput LastOutput => _output;

        /// <summary>
        /// What happened when <see cref="LastOutput"/> was applied to the transforms. Its Applied
        /// flag is false until the latest solution is applied.
        /// </summary>
        public IKApplicationStatus ApplicationStatus => _applicationStatus;

        /// <summary>Name used in log messages, for example "FABRIK Solver".</summary>
        protected virtual string SolverName => GetType().Name;

        /// <summary>True when the solver controls the end joint's rotation, so applying writes it.</summary>
        protected virtual bool WritesEndRotation => false;

        /// <summary>False when the solver keeps slider segments rigid. The solver then warns about enabled sliders.</summary>
        protected virtual bool SupportsSliderTranslation => true;

        /// <summary>
        /// Moves the chain's solver state toward the IKGoal
        /// </summary>
        /// <remarks>
        /// When this is called, Chain holds the pose to start from and up-to-date
        /// constraints. Read and write only the <see cref="SolverJoint"/> solver state; the base class
        /// copies the result into <see cref="LastOutput"/> and applies it.
        /// </remarks>
        /// <param name="goal">The target's pose at the start of this solve.</param>
        /// <returns>The iteration count and the remaining error against <paramref name="goal"/>.</returns>
        protected abstract SolveResult SolveChain(in IKGoal goal);

        /// <summary>Called once after the chain is built, before the first solve.</summary>
        protected virtual void OnInitialized() { }

        /// <summary>Called after one or more joints were bound to different constraint types.</summary>
        protected virtual void OnConstraintsRebound() { }

        /// <summary>
        /// Angle in degrees between the actual end orientation and the target's orientation objective,
        /// read from the transforms after application. NaN for solvers without an orientation objective.
        /// </summary>
        protected virtual float ComputeActualOrientationError() => float.NaN;

        private void Awake()
        {
            Initialize();
        }

        private void LateUpdate()
        {
            if (updateMode == UpdateMode.LateUpdate)
                Step(Time.deltaTime);
        }

        /// <summary>
        /// Runs one solver update: solves, then applies the solution when ApplyMode is Automatic.
        /// With UpdateMode set to LateUpdate, the solver calls this every frame with scaled delta time.
        /// </summary>
        /// <param name="deltaTime">Elapsed time for the joint speed limits.</param>
        public void Step(float deltaTime)
        {
            if (Solve() && applyMode == ApplyMode.Automatic)
                Apply(deltaTime);
        }

        /// <summary>
        /// Solves the chain toward Target and stores the result in
        /// <see cref="LastOutput"/>. Never writes transforms.
        /// </summary>
        /// <returns>False when the solver is not set up or has no target, so nothing was solved.</returns>
        public bool Solve()
        {
            if ((!_isInitialized && !Initialize()) || target == null)
                return false;

            if (!staticSolverConfiguration)
                RefreshConfiguration();

            SetStartingPose();
            _solvedRootParentRotation = _chain.RootParentRotation;

            SolveResult result = SolveChain(new IKGoal(target.position, target.rotation));
            _output.Populate(_chain, result.Iterations, result.Error, result.Error <= tolerance, WritesEndRotation);

            // The new solution has not been applied yet.
            _applicationStatus = IKApplicationStatus.NotApplied;
            Solved?.Invoke(_output);
            return true;
        }

        /// <summary>
        /// Moves the transforms toward the <see cref="LastOutput"/>, moving/rotating speed-limited joints
        /// by at most their speed multiplied by <paramref name="deltaTime"/>.
        /// </summary>
        /// <remarks>
        /// When ApplyMode is Automatic, <see cref="Step"/> already applies each
        /// solution; calling this method as well applies it twice.
        /// </remarks>
        /// <param name="deltaTime">Elapsed time for the speed limits. Zero or less keeps limited joints still.</param>
        /// <returns>
        /// The resulting status, which also becomes <see cref="ApplicationStatus"/>. Not applied when
        /// there is no output yet for the current chain, for example before the first solve.
        /// </returns>
        public IKApplicationStatus Apply(float deltaTime)
        {
            if (!_output.MatchesChain(_chain))
                return IKApplicationStatus.NotApplied;

            OnPoseApplied(_applier.Apply(_output, _chain, deltaTime, synchronizeLimitedJoints));
            return _applicationStatus;
        }

        /// <summary>
        /// Immediately writes <see cref="LastOutput"/> to the transforms, ignoring speed limits. Use it
        /// to initialize a chain or recover after a teleport. It does not solve again.
        /// </summary>
        /// <returns>False when there is no output for the current chain, for example before the first solve.</returns>
        public bool SnapToSolution()
        {
            if (!_output.MatchesChain(_chain))
                return false;

            _output.ApplyToSourceTransforms();
            OnPoseApplied(new IKApplicationStatus(
                applied: true,
                usedSpeedLimits: false,
                anyJointLimited: false,
                anyJointBlocked: false,
                anyJointStartedOutsideLimits: false,
                positionError: float.NaN,
                orientationError: float.NaN));
            return true;
        }

        // Validates the setup and builds the chain. Disables the component when the setup is invalid.
        private bool Initialize()
        {
            _isInitialized = false;
            if (!SolverSetupValidation.Validate(this, SolverName, target, chainJoints))
                return false;

            _chain.Initialize(chainJoints);
            WarnIfSlidersUnsupported();
            OnInitialized();
            _isInitialized = true;
            return true;
        }

        // Pulls the latest joint component settings into the runtime constraints.
        private void RefreshConfiguration()
        {
            _chain.UpdateConstraints();
            bool rebound = _chain.RefreshConstraintBindings();
            _chain.ApplyRuntimeConfigs();
            _chain.ComputeChainLength();

            if (rebound)
            {
                WarnIfSlidersUnsupported();
                OnConstraintsRebound();
            }
        }

        // Sets the solver state to the pose this solve starts from:
        // - the rest pose, when Solve From Rest Pose is on;
        // - LastOutput, while speed-limited joints are still moving toward it, because solving
        //   from the lagging transforms can flip between two IK solutions every frame;
        // - otherwise, the current pose of the actual transforms.
        private void SetStartingPose()
        {
            if (solveFromRestPose)
                _chain.SyncSolverStateFromRestPose();
            else if (_hasLaggingSolution && _output.MatchesChain(_chain))
                _chain.SyncSolverStateFromOutput(_output, _solvedRootParentRotation);
            else
                _chain.SyncSolverStateFromTransforms();

            // Only the next apply decides whether the joints still lag.
            _hasLaggingSolution = false;
        }

        // Runs after every write to the transforms: completes the status with the actual target
        // errors, decides where the next solve starts, and fires the Applied event.
        private void OnPoseApplied(IKApplicationStatus status)
        {
            _applicationStatus = WithActualTargetErrors(status);
            KeepSolutionForNextSolve();
            Applied?.Invoke(_applicationStatus);
        }

        // Keeps LastOutput as the next solve's starting pose while speed-limited joints lag behind it.
        private void KeepSolutionForNextSolve()
        {
            _hasLaggingSolution = _applicationStatus.Applied
                && (_applicationStatus.AnyJointLimited || _applicationStatus.AnyJointBlocked);
        }

        private IKApplicationStatus WithActualTargetErrors(IKApplicationStatus status)
        {
            if (!status.Applied || target == null || _chain.Count == 0)
                return status;

            Transform end = _chain.Joints[_chain.Count - 1].Transform;
            float positionError = Vector3.Distance(end.position, target.position);
            return status.WithTargetErrors(positionError, ComputeActualOrientationError());
        }

        private void WarnIfSlidersUnsupported()
        {
            if (SupportsSliderTranslation)
                return;

            foreach (SolverJoint joint in _chain.Joints)
            {
                if (joint.Segment != null && joint.Segment.RequiresExplicitTransformPosition)
                {
                    Debug.LogWarning($"[OpenIK] The {SolverName} does not solve slider translation. Slider segments stay at their current length.", this);
                    return;
                }
            }
        }
    }
}
