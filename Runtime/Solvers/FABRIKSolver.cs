using System.Collections.Generic;
using UnityEngine;

namespace OpenIK
{
    /// <summary>
    /// Forward And Backward Reaching Inverse Kinematics (FABRIK) Solver ― Each iteration runs two
    /// position-based passes: a backward pass that pulls each joint toward the target while
    /// preserving segment lengths, then a forward pass that re-anchors the chain at the root.
    /// Angular and segment constraints are applied per joint between passes.
    /// </summary>
    /// <remarks>
    /// Solves for end-effector position only; supports prismatic (slider) segments by allowing
    /// segment length to change within configured limits during the position passes.
    /// </remarks>
    public class FABRIKSolver : OpenIKSolverBase
    {
        [Tooltip("Transform the end effector moves toward.")]
        [SerializeField] private Transform target;
        [Tooltip("Position error threshold in world units used to stop iterating.")]
        [SerializeField] private float tolerance = 0.001f;
        [Tooltip("Maximum solver iterations per frame. Higher values allow more refinement but cost more processing time. Stops early when Tolerance is reached.")]
        [SerializeField] private int maxIterations = 8;
        [Tooltip("Joint transforms ordered from the root to the end effector.")]
        [SerializeField] private List<Transform> chainJoints = new();

        [Header("Stability & Performance")]
        [Tooltip("Skip per-frame joint configuration updates when constraints and speed limits stay unchanged.")]
        [SerializeField] private bool staticSolverConfiguration = false;
        [Tooltip("Help a nearly straight chain bend toward a target aligned with it.")]
        [SerializeField] private bool useSingularityHandling = true;
        [Tooltip("Start each solve from the joint rotations and segment offsets captured at initialization.")]
        [SerializeField] private bool solveFromRestPose = false;

        private readonly SolverChain _chain = new();

        protected override SolverChain Chain => _chain;
        protected override Transform SolveTarget => target;

        // convenience properties
        private SolverJoint[] Joints => _chain.Joints;
        private int EndJointIdx => _chain.Count - 1;
        private Transform RootParentTransform => _chain.RootParentTransform;
        private float ChainLength => _chain.ChainLength;

        private void Awake()
        {
            if (!SolverSetupValidation.Validate(this, "FABRIK Solver", target, chainJoints))
                return;

            ValidateHierarchyDepths();
            _chain.Initialize(chainJoints);
        }

        private bool RefreshConstraintBindings()
        {
            return _chain.RefreshConstraintBindings();
        }

        private void ApplyRuntimeConfigs()
        {
            _chain.ApplyRuntimeConfigs();
        }

        private void ValidateHierarchyDepths()
        {
            for (int i = 1; i < chainJoints.Count; i++)
            {
                int parentDepth = GetTransformDepth(chainJoints[i - 1]);
                int childDepth = GetTransformDepth(chainJoints[i]);
                if (parentDepth > childDepth)
                    Debug.LogError($"[OpenFABRIK] Invalid joint order: \"{chainJoints[i - 1].name}\" (depth {parentDepth}) is deeper than \"{chainJoints[i].name}\" (depth {childDepth}). Parent joints must NOT be deeper than their children.");
            }
        }

        private static int GetTransformDepth(Transform t)
        {
            int depth = 0;
            while (t.parent != null)
            {
                depth++;
                t = t.parent;
            }
            return depth;
        }

        private void LateUpdate()
        {
            if (_chain.Count < 2 || target == null) return;

            if (!staticSolverConfiguration)
            {
                _chain.UpdateConstraints();
                RefreshConstraintBindings();
                ApplyRuntimeConfigs();
                _chain.ComputeChainLength();
            }

            SyncSolverStateForSolve(solveFromRestPose);

            Solve();
        }

        /// <summary>
        /// Runs the iterative FABRIK solve in solver-state space and populates <see cref="LastOutput"/>
        /// with the final pose plus convergence stats. When <see cref="Mode"/> is
        /// <see cref="SolveMode.SolveAndApply"/>, the result is then applied to the chain's
        /// transforms, honoring joint speed limits.
        /// </summary>
        /// <remark>
        /// The root position, joint rotations, and any segment positions that
        /// require explicit writes are updated from the final solver state. The end effector may still
        /// be outside <see cref="tolerance"/> if <see cref="maxIterations"/> is reached first.
        /// </remark>
        private void Solve()
        {
            int iteration = 0;

            // Store initial root joint position.
            Vector3 initialRootPos = Joints[0].SolverPosition;

            // The user-supplied target — used for honest end-of-solve error reporting even when the
            // target is unreachable.
            Vector3 unclampedTargetPosition = target.position;

            // The target the solver actually drives toward — clamped to chain length so an unreachable
            // target doesn't keep FABRIK chasing forever.
            Vector3 targetPosition = unclampedTargetPosition;
            if (ChainLength > 0f)
            {
                Vector3 rootToTarget = targetPosition - initialRootPos;
                float dist = rootToTarget.magnitude;
                if (dist > ChainLength)
                    targetPosition = initialRootPos + (rootToTarget / dist) * ChainLength;
            }

            // A fully-extended chain can get stuck collinear with the target; this one-frame offset
            // gives FABRIK a perpendicular direction to fold toward without changing the final target.
            Vector3 singularityOffset = Vector3.zero;
            if (useSingularityHandling)
                singularityOffset = GetSingularityOffset(targetPosition);

            var distEndTarget = Vector3.Distance(Joints[EndJointIdx].SolverPosition, targetPosition);
            while (distEndTarget > tolerance && iteration < maxIterations)
            {
                // On first iteration, add singularity offset to break out of collinear lock
                Vector3 iterationTarget = (iteration == 0 && singularityOffset != Vector3.zero)
                    ? targetPosition + singularityOffset
                    : targetPosition;

                ForwardReach(iterationTarget);
                BackwardReach(initialRootPos);

                // Quaternion reconstruction to prevent drift
                ReconstructQuaternions();

                iteration++;
                distEndTarget = Vector3.Distance(Joints[EndJointIdx].SolverPosition, targetPosition);
            }

            // Report error against the user's actual target — not the chain-length-clamped one — so
            // an unreachable target produces a non-zero FinalError and Converged == false. This
            // matches CCD/Jacobian's reporting semantics.
            float finalError = Vector3.Distance(Joints[EndJointIdx].SolverPosition, unclampedTargetPosition);
            _output.Populate(_chain, iteration, finalError, finalError <= tolerance);

            ApplyAndRaiseSolved();
        }

        /// <summary>
        /// Detects if the chain is near singularity (fully extended and collinear with target)
        /// and returns a perpendicular offset to break out of it.
        /// </summary>
        private Vector3 GetSingularityOffset(Vector3 effectiveTarget)
        {
            int endIdx = EndJointIdx;
            Vector3 rootPos = Joints[0].SolverPosition;
            Vector3 endPos = Joints[endIdx].SolverPosition;

            Vector3 rootToEnd = endPos - rootPos;
            Vector3 rootToTarget = effectiveTarget - rootPos;

            float rootToEndDist = rootToEnd.magnitude;
            float rootToTargetDist = rootToTarget.magnitude;

            // Only trigger when chain is nearly fully extended
            float secondToLastLength = Joints[endIdx].Segment.MaxReach;
            if (rootToEndDist < ChainLength - secondToLastLength * 0.1f)
                return Vector3.zero;

            // Various degenerate checks
            if (rootToEndDist == 0f || rootToTargetDist == 0f)
                return Vector3.zero;

            // Only trigger when target is closer than or at the end effector (need to fold inward)
            if (rootToTargetDist > rootToEndDist)
                return Vector3.zero;

            // Check if directions are nearly collinear
            float dot = Vector3.Dot(rootToEnd / rootToEndDist, rootToTarget / rootToTargetDist);
            if (dot < 0.999f)
                return Vector3.zero;

            // Singularity detected: compute perpendicular offset.
            Vector3 ikDirection = rootToTarget / rootToTargetDist;
            Vector3 secondaryDirection = new Vector3(ikDirection.y, ikDirection.z, ikDirection.x);

            // If second-to-last joint is a hinge, use its axis to avoid fighting the constraint plane
            int secondToLastIdx = Joints[endIdx].ParentIndex;
            if (secondToLastIdx >= 0)
            {
                if (Joints[secondToLastIdx].Angular is HingeAngularConstraint hinge)
                    secondaryDirection = Joints[secondToLastIdx].SolverRotation * hinge.HingeAxis;
            }

            Vector3 offset = Vector3.Cross(ikDirection, secondaryDirection) * secondToLastLength * 0.5f;
            return offset;
        }

        /// Reconstructs solver quaternions to prevent NaN accumulation from repeated FromToRotation operations.
        private void ReconstructQuaternions()
        {
            for (int i = 0; i < Joints.Length; i++)
            {
                Quaternion q = Joints[i].SolverRotation;
                Vector3 forward = q * Vector3.forward;
                Vector3 up = q * Vector3.up;

                // LookRotation needs meaningful basis vectors; skip degenerate rotations instead of normalizing noise.
                if (forward.sqrMagnitude > 0.001f && up.sqrMagnitude > 0.001f)
                    Joints[i].SolverRotation = Quaternion.LookRotation(forward, up);
            }
        }

        /// <summary>
        /// Forward reaching (end-to-root) pass. Anchors the end effector at the target position, then works towards the root.
        /// Each parent is repositioned at bone length from its child and rotated to face it.
        /// If necessary the child joint's rotation is then constrained. Any rotation correction is propagated to keep the chain pose consistent.
        /// When a constraint correction displaces the end-effector, the chain is counter-rotated around the parent's position and translated to restore it to the target.
        /// </summary>
        /// <param name="targetPosition">The target position the end effector should reach toward.</param>
        private void ForwardReach(Vector3 targetPosition)
        {
            Joints[EndJointIdx].SolverPosition = targetPosition;

            var childIdx = EndJointIdx;
            var parentIdx = Joints[EndJointIdx].ParentIndex;
            while (parentIdx != -1)
            {
                // Standard FABRIK: reposition parent at correct bone length from child
                Joints[parentIdx].SolverPosition = ComputeRepositionedParent(childIdx, parentIdx);

                // Sync parent rotation to point at child
                Vector3 currentDir = Joints[parentIdx].SolverRotation * GetCurrentSegmentLocalOffset(childIdx);
                Vector3 desiredDir = Joints[childIdx].SolverPosition - Joints[parentIdx].SolverPosition;
                if (currentDir.sqrMagnitude > 0f && desiredDir.sqrMagnitude > 0f)
                {
                    Joints[parentIdx].SolverRotation = Quaternion.FromToRotation(currentDir, desiredDir)
                                                        * Joints[parentIdx].SolverRotation;
                }

                // Apply angular constraint on the child joint.
                Quaternion parentRotation = Joints[parentIdx].SolverRotation;
                Quaternion childRotation = Joints[childIdx].SolverRotation;
                Quaternion clampedChildRotation = Joints[childIdx].ClampRotation(
                    parentRotation,
                    childRotation,
                    childRotation);

                if (clampedChildRotation != childRotation)
                {
                    // Remember end effector position before constraining
                    Vector3 endPosBefore = Joints[EndJointIdx].SolverPosition;

                    // Step 1: Apply delta rotation to rotate child + downstream joints.
                    Quaternion delta = clampedChildRotation * Quaternion.Inverse(Joints[childIdx].SolverRotation);
                    Joints[childIdx].SolverRotation = clampedChildRotation;
                    _chain.RotateAndRepositionDownstream(childIdx, delta);

                    // Step 2: Compensate by rotating from parent joint down so end effector swings back to direction of the target
                    Vector3 endPosAfter = Joints[EndJointIdx].SolverPosition;
                    Vector3 fromParent = endPosAfter - Joints[parentIdx].SolverPosition;
                    Vector3 toParent = endPosBefore - Joints[parentIdx].SolverPosition;
                    if (fromParent.sqrMagnitude > 0f && toParent.sqrMagnitude > 0f)
                    {
                        Quaternion compensation = Quaternion.FromToRotation(fromParent, toParent);

                        // Rotate parent and all downstream joints (rotation and position)
                        Joints[parentIdx].SolverRotation = compensation * Joints[parentIdx].SolverRotation;
                        _chain.RotateAndRepositionDownstream(parentIdx, compensation);
                    }

                    // Step 3: Translate to reposition end effector exactly at target position
                    Vector3 remainingOffset = endPosBefore - Joints[EndJointIdx].SolverPosition;
                    for (int i = parentIdx; i <= EndJointIdx; i++)
                        Joints[i].SolverPosition += remainingOffset;
                }

                childIdx = parentIdx;
                parentIdx = Joints[parentIdx].ParentIndex;
            }
        }

        /// <summary>
        /// Backwards reaching (root-to-end) pass. Re-anchors the root, then works toward the end effector.
        /// Each joint is repositioned using its parent's rotation and its own local offset, then rotated towards its child, clamping joint constraints along the way.
        /// Any rotation correction is propagated to the downstream joints to keep the chain pose consistent.
        /// </summary>
        /// <param name="InitialRootPosition">Root position captured before the forward pass.</param>
        private void BackwardReach(Vector3 InitialRootPosition)
        {
            // root -> end:
            Joints[0].SolverPosition = InitialRootPosition;

            // Constrain root joint relative to its non-joint parent transform
            if (RootParentTransform != null)
            {
                Quaternion parentRotation = RootParentTransform.rotation;
                Quaternion rootRotation = Joints[0].SolverRotation;
                Quaternion newRootRotation = Joints[0].ClampRotation(
                    parentRotation,
                    rootRotation,
                    rootRotation);

                if (newRootRotation != rootRotation)
                {
                    // Propagate root rotation change to all downstream joints
                    Quaternion delta = newRootRotation * Quaternion.Inverse(Joints[0].SolverRotation);
                    Joints[0].SolverRotation = newRootRotation;
                    _chain.RotateAndRepositionDownstream(0, delta);
                }
            }

            var parentIdx = 0;
            var childIdx = 1;
            var endJointReached = false;
            while (!endJointReached)
            {
                Quaternion parentRotation = Joints[parentIdx].SolverRotation;

                // Derive child position from parent, respecting any prismatic travel.
                Joints[childIdx].SolverPosition = ComputeRepositionedChild(childIdx, parentIdx);

                // Compute desired rotation: swing child to point at its next child (or keep current if end effector)
                Quaternion desiredChildRotation = Joints[childIdx].SolverRotation;
                if (childIdx != EndJointIdx)
                {
                    int nextChildIdx = childIdx + 1;
                    Vector3 nextChildPos = Joints[nextChildIdx].SolverPosition;
                    Vector3 currentDir = Joints[childIdx].SolverRotation * GetCurrentSegmentLocalOffset(nextChildIdx);
                    Vector3 desiredDir = nextChildPos - Joints[childIdx].SolverPosition;
                    if (currentDir.sqrMagnitude > 0f && desiredDir.sqrMagnitude > 0f)
                    {
                        Quaternion swing = Quaternion.FromToRotation(currentDir, desiredDir);
                        desiredChildRotation = swing * Joints[childIdx].SolverRotation;
                    }
                }

                // Clamp the desired rotation against the child's angular constraint.
                Quaternion clampedChildRotation = Joints[childIdx].ClampRotation(
                    parentRotation, Joints[childIdx].SolverRotation, desiredChildRotation);

                // Propagate rotation delta to all downstream joints.
                Quaternion childDelta = clampedChildRotation * Quaternion.Inverse(Joints[childIdx].SolverRotation);
                Joints[childIdx].SolverRotation = clampedChildRotation;
                _chain.RotateAndRepositionDownstream(childIdx, childDelta);

                if (childIdx != EndJointIdx)
                {
                    parentIdx = childIdx;
                    childIdx += 1;
                }
                else endJointReached = true;
            }

        }

        // Computes where parentIdx would be repositioned, without modifying the solver state.
        private Vector3 ComputeRepositionedParent(int childIdx, int parentIdx)
        {
            return Joints[childIdx].Segment.ConstrainParentPosition(
                new ISegmentConstraint.SolveContext(
                    Joints[parentIdx].SolverPosition,
                    Joints[parentIdx].SolverRotation,
                    Joints[childIdx].SolverPosition));
        }

        private Vector3 ComputeRepositionedChild(int childIdx, int parentIdx)
        {
            return Joints[childIdx].Segment.ConstrainChildPosition(
                new ISegmentConstraint.SolveContext(
                    Joints[parentIdx].SolverPosition,
                    Joints[parentIdx].SolverRotation,
                    Joints[childIdx].SolverPosition));
        }

        private Vector3 GetCurrentSegmentLocalOffset(int childIdx)
        {
            return Joints[childIdx].Segment.GetCurrentLocalOffset();
        }

        [Tooltip("Choose when to draw the chain's joints and bone connections.")]
        [SerializeField] private GizmoDrawMode boneGizmoMode = GizmoDrawMode.SelectedOnly;
        public GizmoDrawMode BoneGizmoMode => boneGizmoMode;
        public IReadOnlyList<Transform> ChainJoints => chainJoints;
    }
}
