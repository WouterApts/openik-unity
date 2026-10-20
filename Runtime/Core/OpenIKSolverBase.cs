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
        /// Only solve. Read <see cref="OpenIKSolverBase.LastSolution"/>, or call
        /// <see cref="OpenIKSolverBase.Apply"/> to write it to the transforms.
        /// </summary>
        Manual
    }

    /// <summary>Selects when a solver runs.</summary>
    public enum UpdateMode
    {
        // Explicit values keep saved scenes stable; declaration order sets the Inspector order.

        /// <summary>Step the solver every frame in LateUpdate with scaled delta time.</summary>
        LateUpdate = 0,

        /// <summary>
        /// Step the solver in FixedUpdate with the fixed time step, in sync with physics. With
        /// <see cref="OpenIKSolverBase.Interpolate"/> on, the chain moves smoothly between fixed steps.
        /// </summary>
        FixedUpdate = 2,

        /// <summary>
        /// Never step automatically. Call <see cref="OpenIKSolverBase.Step"/>, or
        /// <see cref="OpenIKSolverBase.Solve"/> and <see cref="OpenIKSolverBase.Apply"/>, from your own code.
        /// </summary>
        Manual = 1
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
        /// By default, the solve counts as converged when this is at most the solver's Tolerance.
        /// </summary>
        public readonly float Error;

        public SolveResult(int iterations, float error)
        {
            Iterations = iterations;
            Error = error;
        }
    }

    /// <summary>
    /// Base class for every OpenIK solver component. It owns the per-frame update, the solve and apply
    /// steps, <see cref="LastSolution"/>, and the application of solutions with joint speed limits.
    /// </summary>
    /// <remarks>
    /// Each <see cref="Step"/> first calls <see cref="Solve"/>, which computes <see cref="LastSolution"/>
    /// without touching the transforms. When ApplyMode is Automatic, it then calls
    /// <see cref="Apply"/>, which moves the transforms toward that solution within the joints'
    /// speed limits.
    ///
    /// Solvers for one chain toward one target derive from <see cref="SingleChainSolverBase"/>. A
    /// solver that builds its own joints derives from this class directly and implements its abstract
    /// members.
    /// </remarks>
    public abstract class OpenIKSolverBase : MonoBehaviour
    {
        [Header("Execution")]
        [Tooltip("When the solver runs.\n" +
                 "• LateUpdate: steps it every frame.\n" +
                 "• FixedUpdate: steps it with the fixed time step, in sync with physics.\n" +
                 "• Manual: leaves it to scripts, which should call Step, or Solve and Apply.")]
        [SerializeField] private UpdateMode updateMode = UpdateMode.LateUpdate;

        [Tooltip("Smooths the chain between fixed steps, like Rigidbody interpolation. The visible pose trails the solved pose by up to " +
                 "one fixed step; physics and the next solve use the solved pose. Used with Update Mode FixedUpdate and Apply Mode Automatic.")]
        [SerializeField] private bool interpolate = true;

        [Tooltip("How each solution reaches the chain's transforms.\n" +
                 "• Automatic: writes it to the transforms, moving joints with Limit Speed enabled at their capped speed.\n" +
                 "• Manual: only solves; other scripts read LastSolution or call Apply.")]
        [FormerlySerializedAs("mode")]
        [SerializeField] private ApplyMode applyMode = ApplyMode.Automatic;

        [Tooltip("Maximum solver iterations per frame. Higher values allow more refinement but cost more processing time. Stops early once the solve converges.")]
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

        // True while speed-limited joints lag behind LastSolution, so the next solve starts from it.
        private bool _hasLaggingSolution;

        // Root-parent rotation that LastSolution was solved with.
        private Quaternion _solvedRootParentRotation = Quaternion.identity;

        // Fixed-step interpolation: the chain's local pose before and after the latest fixed step.
        private Vector3[] _previousLocalPositions = Array.Empty<Vector3>();
        private Quaternion[] _previousLocalRotations = Array.Empty<Quaternion>();
        private Vector3[] _solvedLocalPositions = Array.Empty<Vector3>();
        private Quaternion[] _solvedLocalRotations = Array.Empty<Quaternion>();
        private bool _hasSolvedLocalPose;
        private bool _showsInterpolatedPose;
        private bool _isFixedStepping;

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

        /// <summary>
        /// When true, a solver in FixedUpdate mode with Automatic apply smooths the chain between fixed
        /// steps, like Rigidbody interpolation.
        /// </summary>
        /// <remarks>
        /// The transforms show a pose blended between the last two fixed steps, so they trail the
        /// solved pose by up to one fixed step. Before each fixed step the solver restores the solved
        /// pose, so physics and the next solve never see the blended one.
        /// </remarks>
        public bool Interpolate
        {
            get => interpolate;
            set => interpolate = value;
        }

        /// <summary>Whether <see cref="Step"/> writes each solution to the transforms.</summary>
        public ApplyMode ApplyMode
        {
            get => applyMode;
            set => applyMode = value;
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
        /// Result of the most recent solve: the solved pose and its convergence statistics. While
        /// joint speed limits are active, the transforms can still be moving toward this pose; see
        /// <see cref="ApplicationStatus"/>.
        /// </summary>
        /// <remarks>The result buffers are reused on the next solve. Copy pose data you need to keep.</remarks>
        public IKSolverOutput LastSolution => _output;

        /// <summary>Compatibility alias for <see cref="LastSolution"/>. Use LastSolution in new code.</summary>
        public IKSolverOutput LastOutput => LastSolution;

        /// <summary>
        /// What happened when <see cref="LastSolution"/> was applied to the transforms. Its Applied
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
        /// Checks the setup before the chain is built, when the solver initializes.
        /// </summary>
        /// <returns>
        /// False when the setup is invalid. Log the reason and disable the component before returning,
        /// as the built-in solvers do.
        /// </returns>
        protected abstract bool ValidateSetup();

        /// <summary>
        /// Builds the runtime chain after <see cref="ValidateSetup"/> passed. To build a tree of joints,
        /// use <see cref="SolverChain.Initialize(IReadOnlyList{Transform}, IReadOnlyList{int})"/>.
        /// </summary>
        /// <param name="chain">The solver's chain, to initialize.</param>
        protected abstract void BuildChain(SolverChain chain);

        /// <summary>True when the solver has a goal to solve toward. Solve does nothing while it is false.</summary>
        protected abstract bool HasGoal { get; }

        /// <summary>
        /// Moves the chain's solver state toward the solver's goals.
        /// </summary>
        /// <remarks>
        /// When this is called, Chain holds the pose to start from and up-to-date
        /// constraints. Read and write only the <see cref="SolverJoint"/> solver state; the base class
        /// copies the result into <see cref="LastSolution"/> and applies it.
        /// </remarks>
        /// <returns>The iteration count and the remaining error.</returns>
        protected abstract SolveResult SolveChain();

        /// <summary>Whether a solve result counts as converged.</summary>
        /// <param name="result">The result SolveChain returned.</param>
        protected abstract bool IsConverged(in SolveResult result);

        /// <summary>
        /// Distance in metres from the applied end effector to its target, read from the transforms after
        /// application, for <see cref="ApplicationStatus"/>. With several targets, return the largest.
        /// Only called while HasGoal is true.
        /// </summary>
        protected abstract float ComputeActualPositionError();

        /// <summary>Called once after the chain is built, before the first solve.</summary>
        protected virtual void OnInitialized() { }

        /// <summary>Called after one or more joints were bound to different constraint types.</summary>
        protected virtual void OnConstraintsRebound() { }

        // Called after each configuration refresh, before the starting pose is set. Rebound is true when
        // a joint was bound to a different constraint type. SingleChainSolverBase uses it for its poles,
        // so it works even when a solver overrides OnConstraintsRebound without calling the base method.
        private protected virtual void OnConfigurationRefreshed(bool rebound) { }

        /// <summary>
        /// Angle in degrees between the actual end orientation and the target's orientation objective,
        /// read from the transforms after application. NaN for solvers without an orientation objective.
        /// </summary>
        protected virtual float ComputeActualOrientationError() => float.NaN;

        /// <summary>
        /// Validates the setup and builds the chain again, for solvers whose joints change at runtime.
        /// The rest pose is captured from the transforms as they are, so put the joints that were
        /// already in the chain back in their rest pose first.
        /// </summary>
        /// <returns>False when the new setup is invalid. The solver then stays uninitialized.</returns>
        protected bool Rebuild()
        {
            return Initialize();
        }

        // True after the chain was built from a valid setup.
        private protected bool IsInitialized => _isInitialized;

        // Makes the next solve validate and build the chain again.
        private protected void ResetInitialization() => _isInitialized = false;

        private void Awake()
        {
            Initialize();
        }

        private void FixedUpdate()
        {
            if (updateMode != UpdateMode.FixedUpdate)
                return;

            if (InterpolatesFixedSteps)
            {
                // Start the step from the solved pose, which also becomes the start of the next blend.
                RestoreSolvedLocalPose();
                if (_hasSolvedLocalPose)
                    CopyLocalPose(_solvedLocalPositions, _solvedLocalRotations, _previousLocalPositions, _previousLocalRotations);
            }
            else
            {
                StopInterpolation();
            }

            _isFixedStepping = true;
            try
            {
                Step(Time.fixedDeltaTime);
            }
            finally
            {
                _isFixedStepping = false;
            }
        }

        private void LateUpdate()
        {
            if (InterpolatesFixedSteps)
            {
                // Rigidbody interpolation uses the same blend factor.
                ShowInterpolatedPose((float)((Time.timeAsDouble - Time.fixedTimeAsDouble) / Time.fixedDeltaTime));
                return;
            }

            StopInterpolation();
            if (updateMode == UpdateMode.LateUpdate)
                Step(Time.deltaTime);
        }

        /// <summary>
        /// Runs one solver update: solves, then applies the solution when ApplyMode is Automatic.
        /// With UpdateMode set to LateUpdate, the solver calls this every frame with scaled delta time;
        /// with FixedUpdate, every fixed step with the fixed time step.
        /// </summary>
        /// <param name="deltaTime">Elapsed time for the joint speed limits.</param>
        public void Step(float deltaTime)
        {
            if (Solve() && applyMode == ApplyMode.Automatic)
                Apply(deltaTime);
        }

        /// <summary>
        /// Solves the chain toward its target, or targets, and stores the result in
        /// <see cref="LastSolution"/>. Never writes transforms.
        /// </summary>
        /// <returns>False when the solver is not set up or has no target, so nothing was solved.</returns>
        public bool Solve()
        {
            if ((!_isInitialized && !Initialize()) || !HasGoal)
                return false;

            if (!staticSolverConfiguration)
                OnConfigurationRefreshed(RefreshConfiguration());

            SetStartingPose();
            _solvedRootParentRotation = _chain.RootParentRotation;

            SolveResult result = SolveChain();
            _output.Populate(_chain, result.Iterations, result.Error, IsConverged(result), WritesEndRotation);

            // The new solution has not been applied yet.
            _applicationStatus = IKApplicationStatus.NotApplied;
            Solved?.Invoke(_output);
            return true;
        }

        /// <summary>
        /// Moves the transforms toward the <see cref="LastSolution"/>, moving/rotating speed-limited joints
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
        /// Immediately writes <see cref="LastSolution"/> to the transforms, ignoring speed limits. Use it
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
            if (!ValidateSetup())
                return false;

            BuildChain(_chain);
            _hasLaggingSolution = false;
            _hasSolvedLocalPose = false;
            _showsInterpolatedPose = false;
            WarnIfSlidersUnsupported();
            OnInitialized();
            _isInitialized = true;
            return true;
        }

        // Pulls the latest joint component settings into the runtime constraints. Returns true when
        // any joint was bound to a different constraint type.
        private bool RefreshConfiguration()
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

            return rebound;
        }

        // Sets the solver state to the pose this solve starts from:
        // - the rest pose, when Solve From Rest Pose is on;
        // - LastSolution, while speed-limited joints are still moving toward it, because solving
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
            RecordSolvedLocalPose();
            Applied?.Invoke(_applicationStatus);
        }

        private bool InterpolatesFixedSteps =>
            interpolate && updateMode == UpdateMode.FixedUpdate && applyMode == ApplyMode.Automatic;

        // Records the pose just written to the transforms as the end point of the interpolation.
        // A pose written outside a fixed step, for example by SnapToSolution, shows without blending.
        private void RecordSolvedLocalPose()
        {
            _showsInterpolatedPose = false;
            if (!InterpolatesFixedSteps)
            {
                _hasSolvedLocalPose = false;
                return;
            }

            int count = _chain.Count;
            if (_solvedLocalPositions.Length != count)
            {
                _previousLocalPositions = new Vector3[count];
                _previousLocalRotations = new Quaternion[count];
                _solvedLocalPositions = new Vector3[count];
                _solvedLocalRotations = new Quaternion[count];
                _hasSolvedLocalPose = false;
            }

            SolverJoint[] joints = _chain.Joints;
            for (int i = 0; i < count; i++)
                joints[i].Transform.GetLocalPositionAndRotation(out _solvedLocalPositions[i], out _solvedLocalRotations[i]);

            if (!_hasSolvedLocalPose || !_isFixedStepping)
                CopyLocalPose(_solvedLocalPositions, _solvedLocalRotations, _previousLocalPositions, _previousLocalRotations);
            _hasSolvedLocalPose = true;
        }

        // Writes a blend between the previous and the latest fixed-step pose. Zero shows the previous pose.
        private void ShowInterpolatedPose(float blend)
        {
            if (!_hasSolvedLocalPose || !ChainTransformsExist())
                return;

            blend = Mathf.Clamp01(blend);
            SolverJoint[] joints = _chain.Joints;
            for (int i = 0; i < joints.Length; i++)
            {
                joints[i].Transform.SetLocalPositionAndRotation(
                    Vector3.Lerp(_previousLocalPositions[i], _solvedLocalPositions[i], blend),
                    Quaternion.Slerp(_previousLocalRotations[i], _solvedLocalRotations[i], blend));
            }

            _showsInterpolatedPose = true;
        }

        // Puts the transforms back in the latest solved pose after ShowInterpolatedPose.
        private void RestoreSolvedLocalPose()
        {
            if (!_showsInterpolatedPose)
                return;

            _showsInterpolatedPose = false;
            if (!_hasSolvedLocalPose || !ChainTransformsExist())
                return;

            SolverJoint[] joints = _chain.Joints;
            for (int i = 0; i < joints.Length; i++)
                joints[i].Transform.SetLocalPositionAndRotation(_solvedLocalPositions[i], _solvedLocalRotations[i]);
        }

        // Leaves the solved pose on the transforms and forgets it, for example after a mode change.
        private void StopInterpolation()
        {
            RestoreSolvedLocalPose();
            _hasSolvedLocalPose = false;
        }

        private bool ChainTransformsExist()
        {
            if (_solvedLocalPositions.Length != _chain.Count)
                return false;

            foreach (SolverJoint joint in _chain.Joints)
            {
                if (joint.Transform == null)
                    return false;
            }

            return true;
        }

        private static void CopyLocalPose(Vector3[] fromPositions, Quaternion[] fromRotations, Vector3[] toPositions, Quaternion[] toRotations)
        {
            Array.Copy(fromPositions, toPositions, fromPositions.Length);
            Array.Copy(fromRotations, toRotations, fromRotations.Length);
        }

        // Keeps LastSolution as the next solve's starting pose while speed-limited joints lag behind it.
        private void KeepSolutionForNextSolve()
        {
            _hasLaggingSolution = _applicationStatus.Applied
                && (_applicationStatus.AnyJointLimited || _applicationStatus.AnyJointBlocked);
        }

        private IKApplicationStatus WithActualTargetErrors(IKApplicationStatus status)
        {
            if (!status.Applied || !HasGoal || _chain.Count == 0)
                return status;

            float positionError = ComputeActualPositionError();
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
