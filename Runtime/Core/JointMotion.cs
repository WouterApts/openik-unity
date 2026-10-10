using System;
using UnityEngine;

namespace OpenIK
{
    /// <summary>
    /// Degrees of freedom whose speed a <see cref="ConstrainedJoint"/> type can limit.
    /// </summary>
    [Flags]
    public enum JointMotionSupport
    {
        /// <summary>The joint type has no speed-limit support; Limit Speed has no effect.</summary>
        None = 0,
        /// <summary>The joint's rotation relative to its IK parent can be speed limited (degrees per second).</summary>
        Angular = 1,
        /// <summary>The joint's translation relative to its IK parent can be speed limited (metres per second).</summary>
        Linear = 2
    }

    /// <summary>
    /// Runtime copy of a joint's speed-limit settings, pushed from its <see cref="ConstrainedJoint"/>
    /// together with the other runtime constraint settings.
    /// </summary>
    public readonly struct JointMotionLimit
    {
        /// <summary>True when the joint moves toward the solved pose at a capped speed.</summary>
        public readonly bool LimitSpeed;

        /// <summary>Maximum angular speed in degrees per second. Never negative or NaN.</summary>
        public readonly float MaxAngularSpeed;

        /// <summary>Maximum linear speed in metres per second. Never negative or NaN.</summary>
        public readonly float MaxLinearSpeed;

        public JointMotionLimit(bool limitSpeed, float maxAngularSpeed, float maxLinearSpeed)
        {
            LimitSpeed = limitSpeed;
            MaxAngularSpeed = SanitizeSpeed(maxAngularSpeed);
            MaxLinearSpeed = SanitizeSpeed(maxLinearSpeed);
        }

        /// <summary>No speed limit; the joint takes its solved pose immediately.</summary>
        public static JointMotionLimit Unlimited => default;

        /// Rejects non-finite values: NaN becomes zero, infinity becomes the largest finite speed,
        /// and negative speeds clamp to zero.
        private static float SanitizeSpeed(float speed)
        {
            if (float.IsNaN(speed) || speed <= 0f)
                return 0f;
            return float.IsPositiveInfinity(speed) ? float.MaxValue : speed;
        }
    }

    /// <summary>Outcome of one speed-limited joint step.</summary>
    public enum JointMotionStatus
    {
        /// <summary>The joint reached its desired pose during this step.</summary>
        Reached,
        /// <summary>The joint's speed budget ran out before it reached its desired pose.</summary>
        Limited,
        /// <summary>The budget allowed movement, but no valid progress toward the desired pose was found.</summary>
        Blocked
    }

    /// <summary>Result of one speed-limited joint step.</summary>
    public readonly struct JointMotionStep
    {
        public readonly JointMotionStatus Status;

        /// <summary>
        /// True when the joint's actual starting pose violated its constraint.
        /// </summary>
        public readonly bool StartedOutsideLimits;

        public JointMotionStep(JointMotionStatus status, bool startedOutsideLimits)
        {
            Status = status;
            StartedOutsideLimits = startedOutsideLimits;
        }
    }

    /// <summary>
    /// Optional capability of an <see cref="IAngularConstraint"/>: moves a joint's rotation toward a
    /// desired rotation by a bounded angle without leaving the constraint's allowed range.
    /// </summary>
    /// <remarks>
    /// Deviations are expressed in the constraint-axis frame, as produced by
    /// <see cref="SolverJoint.ToConstraintDeviation"/>. Implementations must not mutate solver state.
    /// </remarks>
    public interface IAngularMotionProvider
    {
        /// <summary>
        /// Angle in degrees the joint still has to travel from <paramref name="currentDeviation"/> to
        /// <paramref name="desiredDeviation"/> along the path that <see cref="StepDeviation"/> follows.
        /// </summary>
        float GetMotionDistance(Quaternion currentDeviation, Quaternion desiredDeviation);

        /// <param name="currentDeviation">The joint's actual deviation from rest pose at the start of the step.</param>
        /// <param name="desiredDeviation">The solved deviation the joint is moving toward.</param>
        /// <param name="maxDegrees">Nonnegative angular budget for this step.</param>
        /// <param name="appliedDeviation">The deviation to apply this step.</param>
        JointMotionStep StepDeviation(
            Quaternion currentDeviation,
            Quaternion desiredDeviation,
            float maxDegrees,
            out Quaternion appliedDeviation);
    }

    /// <summary>
    /// Optional capability of an <see cref="ISegmentConstraint"/>: moves a joint's offset from its IK
    /// parent toward a desired offset by a bounded distance without leaving the segment's allowed range.
    /// </summary>
    /// <remarks>
    /// Offsets are expressed in the IK parent's rotation frame in world units, matching
    /// <see cref="ISegmentConstraint.GetCurrentLocalOffset"/>. Implementations must not mutate solver state.
    /// </remarks>
    public interface ISegmentMotionProvider
    {
        /// <summary>
        /// Distance in metres the joint still has to travel from <paramref name="currentLocalOffset"/>
        /// to <paramref name="desiredLocalOffset"/> along the path that <see cref="StepLocalOffset"/> follows.
        /// </summary>
        float GetMotionDistance(Vector3 currentLocalOffset, Vector3 desiredLocalOffset);

        /// <param name="currentLocalOffset">The joint's actual offset from rest pose at the start of the step.</param>
        /// <param name="desiredLocalOffset">The solved offset the joint is moving toward.</param>
        /// <param name="maxDistance">Nonnegative distance budget for this step, in metres.</param>
        /// <param name="appliedLocalOffset">The offset to apply this step.</param>
        JointMotionStep StepLocalOffset(
            Vector3 currentLocalOffset,
            Vector3 desiredLocalOffset,
            float maxDistance,
            out Vector3 appliedLocalOffset);
    }
}
