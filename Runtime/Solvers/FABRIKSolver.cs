using UnityEngine;

namespace OpenIK
{
    /// <summary>
    /// Forward And Backward Reaching Inverse Kinematics (FABRIK) solver. Moves the end effector to
    /// the target position.
    /// </summary>
    /// <remarks>
    /// Each iteration runs two passes. The forward pass places the end effector on the target and
    /// works toward the root, moving each joint to its segment length from the next one. The backward
    /// pass puts the root back in place and rebuilds the chain toward the end effector. Joint
    /// constraints are applied in both passes, and slider segments change length within their limits.
    /// FABRIK solves for position only; use <see cref="JacobianIKSolver"/> to match orientation too.
    /// </remarks>
    public class FABRIKSolver : OpenIKSolverBase
    {
        [Header("FABRIK")]
        [Tooltip("Help a nearly straight chain bend toward a target aligned with it.")]
        [SerializeField] private bool useSingularityHandling = true;

        /// <summary>When true, a nearly straight chain gets help bending toward a target aligned with it.</summary>
        public bool UseSingularityHandling
        {
            get => useSingularityHandling;
            set => useSingularityHandling = value;
        }

        protected override string SolverName => "FABRIK Solver";

        private SolverJoint[] Joints => Chain.Joints;
        private int EndJointIdx => Chain.Count - 1;

        protected override SolveResult SolveChain(in IKGoal goal)
        {
            Vector3 rootPosition = Joints[0].SolverPosition;

            // Iterate toward a target clamped to the chain's reach, so an unreachable target does not
            // keep FABRIK chasing it. The reported error still uses the real target.
            Vector3 reachableTarget = ClampToChainReach(goal.Position, rootPosition);

            // A fully extended chain can lock up collinear with the target. Offsetting the first
            // iteration's target gives the chain a direction to fold toward.
            Vector3 singularityOffset = useSingularityHandling ? GetSingularityOffset(reachableTarget) : Vector3.zero;

            int iteration = 0;
            float distanceToTarget = Vector3.Distance(Joints[EndJointIdx].SolverPosition, reachableTarget);
            while (distanceToTarget > Tolerance && iteration < MaxIterations)
            {
                Vector3 iterationTarget = iteration == 0 && singularityOffset != Vector3.zero
                    ? reachableTarget + singularityOffset
                    : reachableTarget;

                ForwardReach(iterationTarget);
                BackwardReach(rootPosition);
                Chain.ReconstructRotations();

                iteration++;
                distanceToTarget = Vector3.Distance(Joints[EndJointIdx].SolverPosition, reachableTarget);
            }

            return new SolveResult(iteration, Vector3.Distance(Joints[EndJointIdx].SolverPosition, goal.Position));
        }

        private Vector3 ClampToChainReach(Vector3 targetPosition, Vector3 rootPosition)
        {
            float chainLength = Chain.ChainLength;
            if (chainLength <= 0f)
                return targetPosition;

            Vector3 rootToTarget = targetPosition - rootPosition;
            float distance = rootToTarget.magnitude;
            return distance > chainLength
                ? rootPosition + (rootToTarget / distance) * chainLength
                : targetPosition;
        }

        // Returns a sideways offset for the first iteration's target when the chain is nearly fully
        // extended and the target lies on the chain's line, closer than the end effector. Zero otherwise.
        private Vector3 GetSingularityOffset(Vector3 targetPosition)
        {
            int endIdx = EndJointIdx;
            Vector3 rootPos = Joints[0].SolverPosition;
            Vector3 endPos = Joints[endIdx].SolverPosition;

            Vector3 rootToEnd = endPos - rootPos;
            Vector3 rootToTarget = targetPosition - rootPos;

            float rootToEndDist = rootToEnd.magnitude;
            float rootToTargetDist = rootToTarget.magnitude;

            // Only when the chain is nearly fully extended.
            float lastSegmentLength = Joints[endIdx].Segment.MaxReach;
            if (rootToEndDist < Chain.ChainLength - lastSegmentLength * 0.1f)
                return Vector3.zero;

            if (rootToEndDist == 0f || rootToTargetDist == 0f)
                return Vector3.zero;

            // Only when the chain has to fold inward to reach the target.
            if (rootToTargetDist > rootToEndDist)
                return Vector3.zero;

            // Only when the target lies on the chain's line.
            float dot = Vector3.Dot(rootToEnd / rootToEndDist, rootToTarget / rootToTargetDist);
            if (dot < 0.999f)
                return Vector3.zero;

            Vector3 ikDirection = rootToTarget / rootToTargetDist;
            Vector3 secondaryDirection = new Vector3(ikDirection.y, ikDirection.z, ikDirection.x);

            // Fold along the last bone's hinge axis, if it has one, so the offset does not fight the hinge plane.
            int lastBoneIdx = Joints[endIdx].ParentIndex;
            if (lastBoneIdx >= 0 && Joints[lastBoneIdx].Angular is HingeAngularConstraint hinge)
                secondaryDirection = Joints[lastBoneIdx].SolverRotation * hinge.HingeAxis;

            return Vector3.Cross(ikDirection, secondaryDirection) * lastSegmentLength * 0.5f;
        }

        // Forward pass, from the end effector to the root. Places the end effector on the target, then
        // moves each parent to its segment length from its child and turns it to face the child. When
        // the child's constraint corrects its rotation, the chain below the parent is swung and shifted
        // so the end effector stays on the target.
        private void ForwardReach(Vector3 targetPosition)
        {
            Joints[EndJointIdx].SolverPosition = targetPosition;

            int childIdx = EndJointIdx;
            int parentIdx = Joints[EndJointIdx].ParentIndex;
            while (parentIdx != -1)
            {
                Joints[parentIdx].SolverPosition = ComputeRepositionedParent(childIdx, parentIdx);

                // Turn the parent to point at the child.
                Vector3 currentDir = Joints[parentIdx].SolverRotation * GetCurrentSegmentLocalOffset(childIdx);
                Vector3 desiredDir = Joints[childIdx].SolverPosition - Joints[parentIdx].SolverPosition;
                if (currentDir.sqrMagnitude > 0f && desiredDir.sqrMagnitude > 0f)
                {
                    Joints[parentIdx].SolverRotation = Quaternion.FromToRotation(currentDir, desiredDir)
                                                       * Joints[parentIdx].SolverRotation;
                }

                Quaternion parentRotation = Joints[parentIdx].SolverRotation;
                Quaternion childRotation = Joints[childIdx].SolverRotation;
                Quaternion clampedChildRotation = Joints[childIdx].ClampRotation(parentRotation, childRotation, childRotation);

                if (clampedChildRotation != childRotation)
                {
                    Vector3 endPosBefore = Joints[EndJointIdx].SolverPosition;

                    // 1. Rotate the child, and everything below it, into its constraint.
                    Quaternion delta = clampedChildRotation * Quaternion.Inverse(Joints[childIdx].SolverRotation);
                    Joints[childIdx].SolverRotation = clampedChildRotation;
                    Chain.RotateAndRepositionDownstream(childIdx, delta);

                    // 2. Swing the chain around the parent so the end effector points back at the target.
                    Vector3 fromParent = Joints[EndJointIdx].SolverPosition - Joints[parentIdx].SolverPosition;
                    Vector3 toParent = endPosBefore - Joints[parentIdx].SolverPosition;
                    if (fromParent.sqrMagnitude > 0f && toParent.sqrMagnitude > 0f)
                    {
                        Quaternion compensation = Quaternion.FromToRotation(fromParent, toParent);
                        Joints[parentIdx].SolverRotation = compensation * Joints[parentIdx].SolverRotation;
                        Chain.RotateAndRepositionDownstream(parentIdx, compensation);
                    }

                    // 3. Shift the chain so the end effector sits exactly on the target again.
                    Vector3 remainingOffset = endPosBefore - Joints[EndJointIdx].SolverPosition;
                    for (int i = parentIdx; i <= EndJointIdx; i++)
                        Joints[i].SolverPosition += remainingOffset;
                }

                childIdx = parentIdx;
                parentIdx = Joints[parentIdx].ParentIndex;
            }
        }

        // Backward pass, from the root to the end effector. Puts the root back at its starting
        // position, then places each joint from its parent's pose and turns it toward its child within
        // its constraint. Every rotation is carried down to the joints below.
        private void BackwardReach(Vector3 rootPosition)
        {
            Joints[0].SolverPosition = rootPosition;

            // Constrain the root relative to the transform it is parented to, if any.
            if (Chain.RootParentTransform != null)
            {
                Quaternion rootRotation = Joints[0].SolverRotation;
                Quaternion clampedRootRotation = Joints[0].ClampRotation(Chain.RootParentRotation, rootRotation, rootRotation);

                if (clampedRootRotation != rootRotation)
                {
                    Quaternion delta = clampedRootRotation * Quaternion.Inverse(Joints[0].SolverRotation);
                    Joints[0].SolverRotation = clampedRootRotation;
                    Chain.RotateAndRepositionDownstream(0, delta);
                }
            }

            for (int childIdx = 1; childIdx <= EndJointIdx; childIdx++)
            {
                int parentIdx = childIdx - 1;
                Quaternion parentRotation = Joints[parentIdx].SolverRotation;

                Joints[childIdx].SolverPosition = ComputeRepositionedChild(childIdx, parentIdx);

                // Turn the child to point at its own child. The end effector keeps its rotation.
                Quaternion desiredChildRotation = Joints[childIdx].SolverRotation;
                if (childIdx != EndJointIdx)
                {
                    int nextChildIdx = childIdx + 1;
                    Vector3 currentDir = Joints[childIdx].SolverRotation * GetCurrentSegmentLocalOffset(nextChildIdx);
                    Vector3 desiredDir = Joints[nextChildIdx].SolverPosition - Joints[childIdx].SolverPosition;
                    if (currentDir.sqrMagnitude > 0f && desiredDir.sqrMagnitude > 0f)
                        desiredChildRotation = Quaternion.FromToRotation(currentDir, desiredDir) * Joints[childIdx].SolverRotation;
                }

                Quaternion clampedChildRotation = Joints[childIdx].ClampRotation(
                    parentRotation, Joints[childIdx].SolverRotation, desiredChildRotation);

                Quaternion childDelta = clampedChildRotation * Quaternion.Inverse(Joints[childIdx].SolverRotation);
                Joints[childIdx].SolverRotation = clampedChildRotation;
                Chain.RotateAndRepositionDownstream(childIdx, childDelta);
            }
        }

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
    }
}
