using System;
using UnityEngine;

namespace OpenIK
{
    /// <summary>
    /// MonoBehaviour base for all OpenIK solvers (<see cref="FABRIKSolver"/>, <see cref="CCDIKSolver"/>, <see cref="JacobianIKSolver"/>).
    /// Owns the <see cref="SolveMode"/> toggle, the reusable <see cref="IKSolverOutput"/> buffer,
    /// the <see cref="Solved"/> event, and the shared <see cref="IKPoseApplier"/> that writes the solved
    /// pose to the scene, so consumers can use any solver as a chain pose generator.
    /// </summary>
    public abstract class OpenIKSolverBase : MonoBehaviour
    {
        [Header("Mode")]
        [Tooltip("SolveAndApply (default) writes the solved pose to the chain's transforms each frame, moving joints with Limit Speed enabled at their capped speed. " +
                 "SolveOnly leaves the transforms untouched and exposes the result via LastOutput / the Solved event.")]
        [SerializeField] protected SolveMode mode = SolveMode.SolveAndApply;

        [Tooltip("Coordinate speed-limited joints so they reach the solved pose together. Faster joints slow down to match " +
                 "the slowest; each joint stays within its own speed limit.")]
        [SerializeField] protected bool synchronizeLimitedJoints;

        protected readonly IKSolverOutput _output = new IKSolverOutput();

        private readonly IKPoseApplier _applier = new IKPoseApplier();
        private IKApplicationStatus _applicationStatus = IKApplicationStatus.NotApplied;
        private bool _warnedAboutApplyInSolveAndApply;

        // The previous solution in joint space, kept while speed-limited joints lag behind it.
        private Quaternion[] _seedLocalRotations = Array.Empty<Quaternion>();
        private Vector3[] _seedLocalOffsets = Array.Empty<Vector3>();
        private bool _hasSolutionSeed;

        /// <summary>
        /// Output of the most recent solve: the desired pose and its convergence statistics. While
        /// joint speed limits are active, the transforms can still be moving toward this pose; see
        /// <see cref="ApplicationStatus"/>.
        /// </summary>
        public IKSolverOutput LastOutput => _output;

        /// <summary>
        /// What happened when the most recent solve was written to the transforms. Updated before
        /// <see cref="Solved"/> fires. <see cref="IKApplicationStatus.Applied"/> is false in
        /// <see cref="SolveMode.SolveOnly"/>.
        /// </summary>
        public IKApplicationStatus ApplicationStatus => _applicationStatus;

        /// <summary>Fires after every solve. Subscribers receive the solver's reused
        /// <see cref="IKSolverOutput"/> instance; copy any fields you need to retain.</summary>
        public event Action<IKSolverOutput> Solved;

        /// <summary>
        /// When true, speed-limited joints that cannot all reach the solved pose in one step are slowed
        /// by a common factor so they arrive together (read/write). Used in
        /// <see cref="SolveMode.SolveAndApply"/>; in <see cref="SolveMode.SolveOnly"/>, the caller of
        /// <see cref="ApplyLastOutput"/> chooses instead.
        /// </summary>
        public bool SynchronizeLimitedJoints
        {
            get => synchronizeLimitedJoints;
            set => synchronizeLimitedJoints = value;
        }

        /// <summary>Current solve mode (read/write).</summary>
        public SolveMode Mode
        {
            get => mode;
            set => mode = value;
        }

        /// <summary>The chain this solver solves and applies.</summary>
        protected abstract SolverChain Chain { get; }

        /// <summary>The solve target, used to report the actual end-effector error after application.</summary>
        protected abstract Transform SolveTarget { get; }

        /// <summary>
        /// Angle in degrees between the actual end orientation and the target's orientation objective,
        /// read from the transforms after application. NaN for solvers without an orientation objective.
        /// </summary>
        protected virtual float ComputeActualOrientationError() => float.NaN;

        /// <summary>
        /// Immediately writes <see cref="LastOutput"/> to the transforms, ignoring speed limits. Use it
        /// to initialize a chain or recover after a teleport. It applies the stored solution and does
        /// not solve again, and it writes even in <see cref="SolveMode.SolveOnly"/>.
        /// </summary>
        /// <returns>False when there is no output for the current chain, for example before the first solve.</returns>
        public bool SnapToSolution()
        {
            SolverChain chain = Chain;
            if (!_output.MatchesChain(chain))
                return false;

            _output.ApplyToSourceTransforms();
            SetApplicationStatus(new IKApplicationStatus(true, false, false, false, false, float.NaN, float.NaN));
            return true;
        }

        /// <summary>
        /// Applies <see cref="LastOutput"/> to the transforms, moving speed-limited joints by at most
        /// their speed multiplied by <paramref name="deltaTime"/>. For custom application in
        /// <see cref="SolveMode.SolveOnly"/>; in <see cref="SolveMode.SolveAndApply"/> the solver already
        /// applies every solve, so this call does nothing.
        /// </summary>
        /// <param name="deltaTime">Elapsed time for the speed budget, chosen by the caller.</param>
        /// <param name="synchronizeLimitedJoints">
        /// When true, slow the limited joints by a common factor so they arrive together. The solver's
        /// own <see cref="SynchronizeLimitedJoints"/> setting is not used here; the caller of this method
        /// decides.
        /// </param>
        /// <returns>The resulting status, which also becomes <see cref="ApplicationStatus"/>.</returns>
        public IKApplicationStatus ApplyLastOutput(float deltaTime, bool synchronizeLimitedJoints = false)
        {
            if (mode == SolveMode.SolveAndApply)
            {
                if (!_warnedAboutApplyInSolveAndApply)
                {
                    _warnedAboutApplyInSolveAndApply = true;
                    Debug.LogWarning(
                        "[OpenIK] ApplyLastOutput is for SolveOnly mode; this solver already applies every solve in SolveAndApply mode. The call was ignored.",
                        this);
                }

                return IKApplicationStatus.NotApplied;
            }

            SolverChain chain = Chain;
            if (!_output.MatchesChain(chain))
                return IKApplicationStatus.NotApplied;

            // Use the argument, not the solver's Synchronize setting. The caller of this method decides.
            SetApplicationStatus(_applier.Apply(_output, chain, deltaTime, synchronizeLimitedJoints));
            return _applicationStatus;
        }

        /// <summary>
        /// Writes the populated output in <see cref="SolveMode.SolveAndApply"/> using this frame's
        /// scaled delta time, records <see cref="ApplicationStatus"/>, and raises <see cref="Solved"/>.
        /// Solvers call this once at the end of every solve.
        /// </summary>
        protected void ApplyAndRaiseSolved()
        {
            SetApplicationStatus(mode == SolveMode.SolveAndApply
                ? _applier.Apply(_output, Chain, Time.deltaTime, synchronizeLimitedJoints)
                : IKApplicationStatus.NotApplied);

            RaiseSolved();
        }

        /// <summary>
        /// Seeds the chain's solver state for the next solve: from the rest pose when
        /// <paramref name="solveFromRestPose"/> is set, otherwise from the current transforms.
        /// </summary>
        /// <remarks>
        /// While speed-limited joints are still moving toward the previous solution, the solve
        /// continues from that solution (in joint space, anchored at the actual root) instead of from
        /// the lagging transforms. Re-solving from a pose between two IK solutions can otherwise flip
        /// the solution between branches every frame, leaving the arm oscillating. Once the transforms
        /// reach the solution, solving from the transforms resumes, so external edits are picked up.
        /// </remarks>
        protected void SyncSolverStateForSolve(bool solveFromRestPose)
        {
            SolverChain chain = Chain;
            if (solveFromRestPose)
                chain.SyncSolverStateFromRestPose();
            else if (_hasSolutionSeed && _seedLocalRotations.Length == chain.Count)
                chain.SyncSolverStateFromLocalPose(_seedLocalRotations, _seedLocalOffsets);
            else
                chain.SyncSolverStateFromTransforms();
        }

        protected void RaiseSolved() => Solved?.Invoke(_output);

        private void SetApplicationStatus(IKApplicationStatus status)
        {
            _applicationStatus = WithActualTargetErrors(status);
            UpdateSolutionSeed();
        }

        /// Keeps the latest solution as the next solve's seed while limited joints lag behind it.
        private void UpdateSolutionSeed()
        {
            SolverChain chain = Chain;
            _hasSolutionSeed = _applicationStatus.Applied
                && (_applicationStatus.AnyJointLimited || _applicationStatus.AnyJointBlocked)
                && _output.MatchesChain(chain);
            if (!_hasSolutionSeed)
                return;

            int count = chain.Count;
            if (_seedLocalRotations.Length != count)
            {
                _seedLocalRotations = new Quaternion[count];
                _seedLocalOffsets = new Vector3[count];
            }

            Quaternion rootParentRotation = chain.RootParentTransform != null
                ? chain.RootParentTransform.rotation
                : Quaternion.identity;
            for (int i = 0; i < count; i++)
            {
                int parentIdx = chain.Joints[i].ParentIndex;
                Quaternion parentRotation = parentIdx >= 0 ? _output.WorldRotations[parentIdx] : rootParentRotation;
                _seedLocalRotations[i] = Quaternion.Normalize(Quaternion.Inverse(parentRotation) * _output.WorldRotations[i]);
                _seedLocalOffsets[i] = parentIdx >= 0
                    ? Quaternion.Inverse(parentRotation) * (_output.WorldPositions[i] - _output.WorldPositions[parentIdx])
                    : Vector3.zero;
            }
        }

        private IKApplicationStatus WithActualTargetErrors(IKApplicationStatus status)
        {
            Transform target = SolveTarget;
            SolverChain chain = Chain;
            if (!status.Applied || target == null || chain == null || chain.Count == 0)
                return status;

            Transform end = chain.Joints[chain.Count - 1].Transform;
            float positionError = Vector3.Distance(end.position, target.position);
            return status.WithTargetErrors(positionError, ComputeActualOrientationError());
        }
    }
}
