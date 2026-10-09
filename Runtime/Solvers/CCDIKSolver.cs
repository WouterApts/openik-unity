using System.Collections.Generic;
using UnityEngine;

namespace OpenIK
{
    /// <summary>
    /// Cyclic Coordinate Descent (CCD) Solver ― Each iteration walks the chain from end to root and, at
    /// every joint, rotates the vector pointing at the end-effector to align with the vector
    /// pointing at the target, clamping against angular constraints along the way.
    /// </summary>
    /// <remarks>
    /// Solves for end-effector position only; Does not support prismatic (slider) translation,
    /// slider segments are held rigid during the solve.
    /// </remarks>
    public class CCDIKSolver : OpenIKSolverBase
    {
        [SerializeField] private Transform target;
        [SerializeField] private float tolerance = 0.001f;
        [SerializeField] private int maxIterations = 10;
        [SerializeField] private List<Transform> chainJoints = new();

        [Header("Stability & Performance")]
        [Tooltip("Enable for Optimal Performance when the solver chain, constraints, and runtime configs are static. Skips per-frame constraint updates, constraint binding refreshes, runtime config application (including joint speed limits), and chain length recomputation.")]
        [SerializeField] private bool staticSolverConfiguration = false;
        [SerializeField] private bool solveFromRestPose = false;

        [Header("CCD")]
        [Tooltip("How much of each CCD rotation step is applied. Lower values can improve stability for tight constraints.")]
        [Range(0.01f, 1f)]
        [SerializeField] private float rotationStep = 1f;

        private readonly SolverChain _chain = new();
        private bool _warnedAboutSlider;

        protected override SolverChain Chain => _chain;
        protected override Transform SolveTarget => target;

        private SolverJoint[] Joints => _chain.Joints;
        private int EndJointIdx => _chain.Count - 1;
        private Transform RootParentTransform => _chain.RootParentTransform;

        private void Awake()
        {
            if (!SolverSetupValidation.Validate(this, "CCD Solver", target, chainJoints))
                return;

            _chain.Initialize(chainJoints);
            WarnIfActiveSliderPresent();
        }

        /// <summary>
        /// Refreshes runtime constraint bindings for the current chain.
        /// </summary>
        /// <returns>
        /// True when any joint's runtime segment or angular constraint was rebound; otherwise false.
        /// </returns>
        private bool RefreshConstraintBindings()
        {
            return _chain.RefreshConstraintBindings();
        }

        /// <summary>
        /// Copies the latest serialized joint settings into the already-bound runtime constraints.
        /// </summary>
        private void ApplyRuntimeConfigs()
        {
            _chain.ApplyRuntimeConfigs();
        }

        private void LateUpdate()
        {
            if (_chain.Count < 2 || target == null) return;

            if (!staticSolverConfiguration)
            {
                _chain.UpdateConstraints();
                if (RefreshConstraintBindings())
                    _warnedAboutSlider = false;
                ApplyRuntimeConfigs();
                _chain.ComputeChainLength();
                WarnIfActiveSliderPresent();
            }

            SyncSolverStateForSolve(solveFromRestPose);

            Solve();
        }

        /// <summary>
        /// Runs the iterative CCD pass and applies the final solver pose to the transform chain.
        /// <para>
        /// Each joint rotation step is optionally scaled by <see cref="rotationStep"/>, clamped by
        /// the joint's runtime angular constraint, and propagated through downstream solver joints.
        /// The solve exits early when the end effector reaches <see cref="tolerance"/> or when
        /// <see cref="maxIterations"/> is reached.
        /// </para>
        /// </summary>
        private void Solve()
        {
            float step = Mathf.Clamp01(rotationStep);
            if (step <= 0f)
                return;

            float toleranceSqr = tolerance * tolerance;
            Vector3 targetPosition = target.position;
            int iteration = 0;
            float endToTargetSqr = (Joints[EndJointIdx].SolverPosition - targetPosition).sqrMagnitude;

            while (iteration < maxIterations)
            {
                if (endToTargetSqr <= toleranceSqr)
                    break;

                for (int jointIdx = EndJointIdx - 1; jointIdx >= 0; jointIdx--)
                {
                    Vector3 jointToEnd = Joints[EndJointIdx].SolverPosition - Joints[jointIdx].SolverPosition;
                    Vector3 jointToTarget = targetPosition - Joints[jointIdx].SolverPosition;
                    if (jointToEnd.sqrMagnitude <= 1e-8f || jointToTarget.sqrMagnitude <= 1e-8f)
                        continue;

                    Quaternion rotationDelta = Quaternion.FromToRotation(jointToEnd, jointToTarget);
                    Quaternion stepDelta = step >= 1f
                        ? rotationDelta
                        : Quaternion.Slerp(Quaternion.identity, rotationDelta, step);
                    Quaternion desiredRotation = stepDelta * Joints[jointIdx].SolverRotation;

                    Quaternion parentRotation = GetParentRotation(jointIdx);
                    Quaternion clampedRotation = Joints[jointIdx].ClampRotation(
                        parentRotation,
                        Joints[jointIdx].SolverRotation,
                        desiredRotation);

                    Quaternion appliedDelta = clampedRotation * Quaternion.Inverse(Joints[jointIdx].SolverRotation);
                    Joints[jointIdx].SolverRotation = clampedRotation;
                    _chain.RotateAndRepositionDownstream(jointIdx, appliedDelta);
                }

                ReconstructQuaternions();

                iteration++;
                endToTargetSqr = (Joints[EndJointIdx].SolverPosition - targetPosition).sqrMagnitude;
                if (endToTargetSqr <= toleranceSqr)
                    break;
            }

            float finalError = Mathf.Sqrt(endToTargetSqr);
            _output.Populate(_chain, iteration, finalError, finalError <= tolerance);

            ApplyAndRaiseSolved();
        }

        /// <summary>
        /// Gets the world rotation used as the parent frame for a joint's local deviation.
        /// </summary>
        /// <param name="jointIdx">Index of the joint whose parent frame should be resolved.</param>
        /// <returns>
        /// The solver rotation of the previous joint, the root parent transform rotation, or identity
        /// when no parent exists.
        /// </returns>
        private Quaternion GetParentRotation(int jointIdx)
        {
            int parentIdx = Joints[jointIdx].ParentIndex;
            if (parentIdx >= 0)
                return Joints[parentIdx].SolverRotation;

            return RootParentTransform != null ? RootParentTransform.rotation : Quaternion.identity;
        }

        /// <summary>
        /// Rebuilds solver quaternions from their forward and up basis vectors to limit accumulated
        /// floating-point drift from repeated incremental rotations.
        /// </summary>
        private void ReconstructQuaternions()
        {
            for (int i = 0; i < Joints.Length; i++)
            {
                Quaternion q = Joints[i].SolverRotation;
                Vector3 forward = q * Vector3.forward;
                Vector3 up = q * Vector3.up;

                if (forward.sqrMagnitude > 0.001f && up.sqrMagnitude > 0.001f)
                    Joints[i].SolverRotation = Quaternion.LookRotation(forward, up);
            }
        }

        /// <summary>
        /// Logs a one-time warning when an enabled slider segment is present in the CCD chain.
        /// </summary>
        private void WarnIfActiveSliderPresent()
        {
            if (_warnedAboutSlider)
                return;

            for (int i = 0; i < Joints.Length; i++)
            {
                if (Joints[i].Segment != null && Joints[i].Segment.RequiresExplicitTransformPosition)
                {
                    Debug.LogWarning("[OpenIK] The CCD Solver does not actively solve prismatic (slider) translation.", this);
                    _warnedAboutSlider = true;
                    return;
                }
            }
        }

        [SerializeField] private GizmoDrawMode boneGizmoMode = GizmoDrawMode.SelectedOnly;
        /// <summary>
        /// Gets when the CCD bone connection gizmos should be drawn in the Scene view.
        /// </summary>
        /// <value>The selected gizmo draw mode for this solver.</value>
        public GizmoDrawMode BoneGizmoMode => boneGizmoMode;

        /// <summary>
        /// Gets the authored root-to-end joint list used by gizmo drawing and solver setup.
        /// </summary>
        /// <value>The serialized CCD chain transforms in root-to-end order.</value>
        public IReadOnlyList<Transform> ChainJoints => chainJoints;
    }
}
