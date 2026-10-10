using UnityEngine;

namespace OpenIK
{
    /// <summary>Limits a joint to rotation around one axis, between a minimum and a maximum angle.</summary>
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

        // Rebuilds the constraint frame when the hinge axis or zero angle changes. The min and max
        // angles are not cached; ApplyAngularConfig copies them before each solve.
        public override void UpdateConstraints()
        {
            if (_cachedHingeAxis != hingeAxis || !Mathf.Approximately(_cachedZeroAngleOffset, zeroAngleOffset))
            {
                _cachedHingeAxis = hingeAxis;
                _cachedZeroAngleOffset = zeroAngleOffset;
                _localConstraintAxisRotation = Quaternion.FromToRotation(Vector3.forward, ComputeZeroReferenceDirection().normalized);
            }
        }

        // The direction of the zero angle: perpendicular to the hinge axis and turned by zeroAngleOffset.
        // hingeMinAngle and hingeMaxAngle are measured from it.
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

        /// <summary>Keeps only the rotation around the hinge axis, without clamping the angle.</summary>
        /// <param name="deviation">The joint's rotation away from its rest pose, in the constraint frame.</param>
        /// <param name="localHingeAxis">The hinge axis in the constraint frame.</param>
        /// <returns>A normalized rotation around the hinge axis only.</returns>
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
        /// Keeps only the rotation around the hinge axis and clamps its angle to the allowed range.
        /// </summary>
        /// <param name="deviation">The joint's rotation away from its rest pose, in the constraint frame.</param>
        /// <param name="localHingeAxis">The hinge axis in the constraint frame.</param>
        /// <param name="minAngle">Minimum allowed angle in degrees.</param>
        /// <param name="maxAngle">Maximum allowed angle in degrees.</param>
        /// <returns>The clamped rotation around the hinge axis.</returns>
        public static Quaternion ClampHinge(Quaternion deviation, Vector3 localHingeAxis, float minAngle, float maxAngle)
        {
            float angleDeg = ExtractHingeAngle(deviation, localHingeAxis);
            float clampedDeg = Mathf.Clamp(angleDeg, minAngle, maxAngle);

            return Quaternion.AngleAxis(clampedDeg, localHingeAxis);
        }

        /// <summary>Measures the signed angle around the hinge axis, ignoring rotation around other axes.</summary>
        /// <param name="deviation">The joint's rotation away from its rest pose, in the constraint frame.</param>
        /// <param name="localHingeAxis">The hinge axis in the constraint frame.</param>
        /// <returns>The signed hinge angle in degrees, from -180 to 180.</returns>
        public static float ExtractHingeAngle(Quaternion deviation, Vector3 localHingeAxis)
        {
            // Keep only the rotation around localHingeAxis.
            float dot = deviation.x * localHingeAxis.x + deviation.y * localHingeAxis.y + deviation.z * localHingeAxis.z;
            float projX = localHingeAxis.x * dot;
            float projY = localHingeAxis.y * dot;
            float projZ = localHingeAxis.z * dot;

            Quaternion hingeOnly = new Quaternion(projX, projY, projZ, deviation.w);
            hingeOnly.Normalize();

            // A positive w gives the same angle for both forms of the same rotation.
            if (hingeOnly.w < 0f)
            {
                hingeOnly.x = -hingeOnly.x;
                hingeOnly.y = -hingeOnly.y;
                hingeOnly.z = -hingeOnly.z;
                hingeOnly.w = -hingeOnly.w;
            }

            float sinHalf = Mathf.Sqrt(hingeOnly.x * hingeOnly.x + hingeOnly.y * hingeOnly.y + hingeOnly.z * hingeOnly.z);
            float sign = (hingeOnly.x * localHingeAxis.x + hingeOnly.y * localHingeAxis.y + hingeOnly.z * localHingeAxis.z) >= 0f ? 1f : -1f;
            float angleRad = 2f * Mathf.Atan2(sinHalf * sign, hingeOnly.w);
            return angleRad * Mathf.Rad2Deg;
        }
    }
}
