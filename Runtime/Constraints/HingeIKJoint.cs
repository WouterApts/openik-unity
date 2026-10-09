using UnityEngine;

namespace OpenIK
{
    public class HingeIKJoint : ConstrainedJoint
    {
        [Header("Hinge Axis")]
        [Tooltip("The local-space axis the hinge rotates around.")]
        public Vector3 hingeAxis = Vector3.forward;

        [Header("Hinge Constraints")]
        [Tooltip("Rotation in degrees around the hinge axis that defines where 0 degrees is.")]
        [Range(0f, 360f)] public float zeroAngleOffset = 180f;
        [Space]
        [Tooltip("Minimum hinge angle in degrees (relative to zero angle).")]
        [Range(-180f, 0f)] public float hingeMinAngle = -90;
        [Tooltip("Maximum hinge angle in degrees (relative to zero angle).")]
        [Range(0f, 180f)] public float hingeMaxAngle = 90;

        private Quaternion _localConstraintAxisRotation = Quaternion.identity;

        private Vector3 _cachedHingeAxis;
        private float _cachedZeroAngleOffset;

        public override Quaternion LocalConstraintAxisRotation => _localConstraintAxisRotation;

        public override JointMotionSupport MotionSupport => JointMotionSupport.Angular;

        /// <summary>
        /// Refreshes cached hinge data when the hinge axis or zero-angle offset changes.
        /// </summary>
        /// <remarks>
        /// Recomputes the local constraint frame when <see cref="hingeAxis"/> or
        /// <see cref="zeroAngleOffset"/> changes. Min and max angles are not cached here; they are
        /// copied into the runtime config each frame by <see cref="ApplyAngularConfig"/>.
        /// </remarks>
        public override void UpdateConstraints()
        {
            if (_cachedHingeAxis != hingeAxis || !Mathf.Approximately(_cachedZeroAngleOffset, zeroAngleOffset))
            {
                _cachedHingeAxis = hingeAxis;
                _cachedZeroAngleOffset = zeroAngleOffset;
                _localConstraintAxisRotation = Quaternion.FromToRotation(Vector3.forward, ComputeZeroReferenceDirection().normalized);
            }
        }

        // The zero-angle reference vector (perpendicular to the hinge axis, rotated by zeroAngleOffset).
        // Defines the constraint frame origin; hingeMinAngle / hingeMaxAngle are measured from this.
        private Vector3 ComputeZeroReferenceDirection()
        {
            Vector3 hinge = hingeAxis.normalized;
            Vector3 perp = Mathf.Abs(Vector3.Dot(hinge, Vector3.forward)) < 0.99f
                ? Vector3.forward
                : Vector3.up;
            Vector3 baseDir = Vector3.Cross(hinge, perp).normalized;
            return Quaternion.AngleAxis(zeroAngleOffset, hinge) * baseDir;
        }

        public override IAngularConstraint CreateAngularConstraint(in IAngularConstraint.SetupData setupData)
        {
            return new HingeAngularConstraint(BuildAngularConfig());
        }

        public override void ApplyAngularConfig(IAngularConstraint angular)
        {
            if (angular is HingeAngularConstraint hinge)
                hinge.ApplyConfig(BuildAngularConfig());
        }

        /// Packages the current hinge settings for the runtime angular constraint.
        private HingeAngularConstraint.Config BuildAngularConfig()
        {
            Vector3 hinge = hingeAxis.sqrMagnitude > 1e-8f
                ? hingeAxis.normalized
                : Vector3.forward;
            return new HingeAngularConstraint.Config(
                hinge,
                _localConstraintAxisRotation,
                hingeMinAngle,
                hingeMaxAngle);
        }

        /// <summary>
        /// Projects a deviation quaternion onto the hinge axis plane,
        /// without clamping the angle. Used in the FABRIK forward pass for hinge enforcement.
        /// </summary>
        /// <param name="deviation">The deviation quaternion to project.</param>
        /// <param name="localHingeAxis">The hinge axis in constraint-frame-local space.</param>
        /// <returns>A normalized quaternion containing only the hinge-axis rotation component.</returns>
        public static Quaternion ProjectOntoHingePlane(Quaternion deviation, Vector3 localHingeAxis)
        {
            float dot = deviation.x * localHingeAxis.x + deviation.y * localHingeAxis.y + deviation.z * localHingeAxis.z;
            float projX = localHingeAxis.x * dot;
            float projY = localHingeAxis.y * dot;
            float projZ = localHingeAxis.z * dot;

            Quaternion hingeOnly = new Quaternion(projX, projY, projZ, deviation.w);
            hingeOnly.Normalize();

            return hingeOnly;
        }

        /// <summary>
        /// Clamps a deviation quaternion to a hinge constraint.
        /// Projects the deviation onto rotation around the hinge axis, then clamps the angle.
        /// </summary>
        /// <param name="deviation">The deviation quaternion to clamp.</param>
        /// <param name="localHingeAxis">The hinge axis in constraint-frame-local space.</param>
        /// <param name="minAngle">Minimum allowed angle in degrees.</param>
        /// <param name="maxAngle">Maximum allowed angle in degrees.</param>
        /// <returns>A quaternion representing the clamped rotation around the hinge axis.</returns>
        public static Quaternion ClampHinge(Quaternion deviation, Vector3 localHingeAxis, float minAngle, float maxAngle)
        {
            float angleDeg = ExtractHingeAngle(deviation, localHingeAxis);
            float clampedDeg = Mathf.Clamp(angleDeg, minAngle, maxAngle);

            return Quaternion.AngleAxis(clampedDeg, localHingeAxis);
        }

        /// <summary>
        /// Extracts the signed rotation angle of a deviation around the hinge axis, ignoring any
        /// off-axis rotation.
        /// </summary>
        /// <param name="deviation">The deviation quaternion to measure.</param>
        /// <param name="localHingeAxis">The hinge axis in constraint-frame-local space.</param>
        /// <returns>The signed hinge angle in degrees, in the range [-180, 180].</returns>
        public static float ExtractHingeAngle(Quaternion deviation, Vector3 localHingeAxis)
        {
            // Project the deviation onto the hinge axis:
            // Extract only the component of rotation around localHingeAxis
            float dot = deviation.x * localHingeAxis.x + deviation.y * localHingeAxis.y + deviation.z * localHingeAxis.z;
            float projX = localHingeAxis.x * dot;
            float projY = localHingeAxis.y * dot;
            float projZ = localHingeAxis.z * dot;

            // Construct normalized quaternion with only the hinge-axis component
            Quaternion hingeOnly = new Quaternion(projX, projY, projZ, deviation.w);
            hingeOnly.Normalize();

            // Ensure w > 0 for consistent angle extraction
            if (hingeOnly.w < 0f)
            {
                hingeOnly.x = -hingeOnly.x;
                hingeOnly.y = -hingeOnly.y;
                hingeOnly.z = -hingeOnly.z;
                hingeOnly.w = -hingeOnly.w;
            }

            // Extract signed angle around the hinge axis
            float sinHalf = Mathf.Sqrt(hingeOnly.x * hingeOnly.x + hingeOnly.y * hingeOnly.y + hingeOnly.z * hingeOnly.z);
            float sign = (hingeOnly.x * localHingeAxis.x + hingeOnly.y * localHingeAxis.y + hingeOnly.z * localHingeAxis.z) >= 0f ? 1f : -1f;
            float angleRad = 2f * Mathf.Atan2(sinHalf * sign, hingeOnly.w);
            return angleRad * Mathf.Rad2Deg;
        }
    }
}
