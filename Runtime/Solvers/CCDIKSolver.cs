using UnityEngine;

namespace OpenIK
{
    /// <summary>
    /// Cyclic Coordinate Descent (CCD) solver. Moves the end effector to the target position by
    /// rotating one joint at a time.
    /// </summary>
    /// <remarks>
    /// Each iteration walks the chain from the end effector to the root. At every joint it rotates the
    /// direction toward the end effector onto the direction toward the target, within the joint's
    /// constraint. CCD solves for position only and keeps slider segments at their current length.
    /// </remarks>
    public class CCDIKSolver : OpenIKSolverBase
    {
        private const float MinRotationStep = 0.01f;

        [Header("CCD")]
        [Tooltip("How much of each CCD rotation step is applied. Lower values can improve stability for tight constraints.")]
        [Range(MinRotationStep, 1f)]
        [SerializeField] private float rotationStep = 1f;

        /// <summary>Fraction of each joint's rotation toward the target applied per step, from 0.01 to 1.</summary>
        public float RotationStep
        {
            get => rotationStep;
            set => rotationStep = Mathf.Clamp(value, MinRotationStep, 1f);
        }

        protected override string SolverName => "CCD Solver";

        protected override bool SupportsSliderTranslation => false;

        protected override SolveResult SolveChain(in IKGoal goal)
        {
            SolverJoint[] joints = Chain.Joints;
            int endIdx = joints.Length - 1;
            float step = Mathf.Clamp(rotationStep, MinRotationStep, 1f);
            float toleranceSqr = Tolerance * Tolerance;

            int iteration = 0;
            float endToTargetSqr = (joints[endIdx].SolverPosition - goal.Position).sqrMagnitude;
            while (iteration < MaxIterations && endToTargetSqr > toleranceSqr)
            {
                for (int jointIdx = endIdx - 1; jointIdx >= 0; jointIdx--)
                    RotateJointTowardTarget(jointIdx, goal.Position, step);

                Chain.ReconstructRotations();

                iteration++;
                endToTargetSqr = (joints[endIdx].SolverPosition - goal.Position).sqrMagnitude;
            }

            return new SolveResult(iteration, Mathf.Sqrt(endToTargetSqr));
        }

        // Rotates one joint so its direction toward the end effector turns onto its direction toward
        // the target, scaled by the rotation step and clamped by the joint's constraint. The joints
        // below it follow.
        private void RotateJointTowardTarget(int jointIdx, Vector3 targetPosition, float step)
        {
            SolverJoint[] joints = Chain.Joints;
            SolverJoint joint = joints[jointIdx];

            Vector3 jointToEnd = joints[joints.Length - 1].SolverPosition - joint.SolverPosition;
            Vector3 jointToTarget = targetPosition - joint.SolverPosition;
            if (jointToEnd.sqrMagnitude <= 1e-8f || jointToTarget.sqrMagnitude <= 1e-8f)
                return;

            Quaternion rotationDelta = Quaternion.FromToRotation(jointToEnd, jointToTarget);
            Quaternion stepDelta = step >= 1f
                ? rotationDelta
                : Quaternion.Slerp(Quaternion.identity, rotationDelta, step);
            Quaternion desiredRotation = stepDelta * joint.SolverRotation;

            Quaternion clampedRotation = joint.ClampRotation(
                Chain.GetParentSolverRotation(jointIdx),
                joint.SolverRotation,
                desiredRotation);

            Quaternion appliedDelta = clampedRotation * Quaternion.Inverse(joint.SolverRotation);
            joint.SolverRotation = clampedRotation;
            Chain.RotateAndRepositionDownstream(jointIdx, appliedDelta);
        }
    }
}
