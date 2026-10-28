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
    /// pass puts the root back in place and works toward the end effector, turning each joint toward
    /// the forward position of its child joint. Joint constraints are applied in both passes.
    /// FABRIK solves for position only. use <see cref="JacobianIKSolver"/> to match orientation too.
    /// </remarks>
    [AddComponentMenu("OpenIK/FABRIK Solver", 0)]
    public class FABRIKSolver : SingleChainSolverBase
    {
        [Space(6)]
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

                // After the reach passes, so they cannot undo the final pole direction. The base
                // class already turned the starting pose before the first iteration.
                ApplyPoles();
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
        // so the end effector stays on the target. The root is limited last, against its parent
        // transform.
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

                ClampKeepingEndInPlace(childIdx, Joints[parentIdx].SolverRotation, parentIdx);

                childIdx = parentIdx;
                parentIdx = Joints[parentIdx].ParentIndex;
            }
            
            ClampKeepingEndInPlace(0, Chain.RootParentRotation, 0);
        }

        // Clamps a joint against its parent's rotation. When the clamp changes the joint, this swings the
        // chain around the pivot joint and shifts it, so the end effector stays where it was.
        private void ClampKeepingEndInPlace(int jointIdx, Quaternion parentRotation, int pivotIdx)
        {
            Quaternion rotation = Joints[jointIdx].SolverRotation;
            Quaternion clampedRotation = Joints[jointIdx].ClampRotation(parentRotation, rotation, rotation);
            if (clampedRotation == rotation)
                return;

            Vector3 endPosBefore = Joints[EndJointIdx].SolverPosition;

            // 1. Rotate the joint, and everything below it, into its constraint.
            Quaternion delta = clampedRotation * Quaternion.Inverse(rotation);
            Joints[jointIdx].SolverRotation = clampedRotation;
            Chain.RotateAndRepositionDownstream(jointIdx, delta);

            // 2. Swing the chain around the pivot so the end effector points back where it was.
            Vector3 fromPivot = Joints[EndJointIdx].SolverPosition - Joints[pivotIdx].SolverPosition;
            Vector3 toPivot = endPosBefore - Joints[pivotIdx].SolverPosition;
            if (fromPivot.sqrMagnitude > 0f && toPivot.sqrMagnitude > 0f)
            {
                Quaternion compensation = Quaternion.FromToRotation(fromPivot, toPivot);
                Joints[pivotIdx].SolverRotation = compensation * Joints[pivotIdx].SolverRotation;
                Chain.RotateAndRepositionDownstream(pivotIdx, compensation);
            }

            // 3. Shift the chain so the end effector sits exactly where it was again.
            Vector3 remainingOffset = endPosBefore - Joints[EndJointIdx].SolverPosition;
            for (int i = pivotIdx; i <= EndJointIdx; i++)
                Joints[i].SolverPosition += remainingOffset;
        }

        // Backward pass, from the root to the end effector. Puts the root back at its starting
        // position, then turns each joint toward the position the forward pass gave its child, within
        // its constraint, and places the child from the joint's new pose.
        private void BackwardReach(Vector3 rootPosition)
        {
            Joints[0].SolverPosition = rootPosition;

            // Without a parent transform, the root is constrained relative to world space.
            TurnTowardChild(0, Chain.RootParentRotation);

            for (int childIdx = 1; childIdx <= EndJointIdx; childIdx++)
            {
                int parentIdx = childIdx - 1;
                Joints[childIdx].SolverPosition = ComputeRepositionedChild(childIdx, parentIdx);
                TurnTowardChild(childIdx, Joints[parentIdx].SolverRotation);
            }
        }

        // Turns a joint toward its child's current position, within its constraint. The end effector
        // has no child and is only clamped. The turn carries down to the rotations below but not to
        // the positions.
        private void TurnTowardChild(int jointIdx, Quaternion parentRotation)
        {
            Quaternion rotation = Joints[jointIdx].SolverRotation;
            Quaternion desiredRotation = rotation;
            if (jointIdx != EndJointIdx)
            {
                int childIdx = jointIdx + 1;
                Vector3 currentDir = rotation * GetCurrentSegmentLocalOffset(childIdx);
                Vector3 desiredDir = Joints[childIdx].SolverPosition - Joints[jointIdx].SolverPosition;
                if (currentDir.sqrMagnitude > 0f && desiredDir.sqrMagnitude > 0f)
                    desiredRotation = Quaternion.FromToRotation(currentDir, desiredDir) * rotation;
            }

            Quaternion clampedRotation = Joints[jointIdx].ClampRotation(parentRotation, rotation, desiredRotation);
            Quaternion delta = clampedRotation * Quaternion.Inverse(rotation);
            Joints[jointIdx].SolverRotation = clampedRotation;
            for (int i = jointIdx + 1; i <= EndJointIdx; i++)
                Joints[i].SolverRotation = delta * Joints[i].SolverRotation;
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
