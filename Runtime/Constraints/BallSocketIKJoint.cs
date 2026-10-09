using UnityEngine;

namespace OpenIK
{
    public class BallSocketIKJoint : ConstrainedJoint
    {
        [Header("Constraint Axis")]
        [Tooltip("The local-space direction of the constraint axis (center of the swing cone). Z-forward is default.")]
        public Vector3 constraintAxis = Vector3.forward;

        [Header("Angular Constraints (degrees, 0-180)")]
        [Tooltip("Maximum swing angle from the constraint axis in the pitch direction, in degrees. A value of 45 allows 45 degrees to either side.")]
        [Range(0f, 180f)] public float swingPitchHalfAngle = 90f;
        [Tooltip("Maximum swing angle from the constraint axis in the yaw direction, in degrees. A value of 45 allows 45 degrees to either side.")]
        [Range(0f, 180f)] public float swingYawHalfAngle = 90f;
        [Space]
        [Tooltip("Maximum twist in either direction around the constraint axis, in degrees. Zero locks twist; 180 allows unrestricted twist.")]
        [Range(0f, 180f)] public float twistHalfAngle = 180f;

        public float SwingPitchHalfSin { get; private set; }
        public float SwingYawHalfSin { get; private set; }

        private Quaternion _localConstraintAxisRotation = Quaternion.identity;

        private Vector3 _cachedConstraintAxis;
        private float _cachedPitch;
        private float _cachedYaw;

        public override Quaternion LocalConstraintAxisRotation => _localConstraintAxisRotation;

        /// The speed cap applies to the joint's total relative rotation (swing and twist combined).
        public override JointMotionSupport MotionSupport => JointMotionSupport.Angular;

        /// <summary>
        /// Refreshes cached data when the constraint axis or swing limits change.
        /// </summary>
        /// <remarks>
        /// Recomputes the local constraint frame when <see cref="constraintAxis"/> changes, the pitch
        /// sine when <see cref="swingPitchHalfAngle"/> changes, and the yaw sine when
        /// <see cref="swingYawHalfAngle"/> changes. Twist is not cached here; it is copied into the
        /// runtime config each frame by <see cref="ApplyAngularConfig"/>.
        /// </remarks>
        public override void UpdateConstraints()
        {
            if (_cachedConstraintAxis != constraintAxis)
            {
                _cachedConstraintAxis = constraintAxis;
                _localConstraintAxisRotation = Quaternion.FromToRotation(Vector3.forward, constraintAxis.normalized);
            }
            if (!Mathf.Approximately(_cachedPitch, swingPitchHalfAngle))
            {
                _cachedPitch = swingPitchHalfAngle;
                SwingPitchHalfSin = Mathf.Sin(Mathf.Deg2Rad * Mathf.Clamp(_cachedPitch, 0f, 180f) * 0.5f);
            }
            if (!Mathf.Approximately(_cachedYaw, swingYawHalfAngle))
            {
                _cachedYaw = swingYawHalfAngle;
                SwingYawHalfSin = Mathf.Sin(Mathf.Deg2Rad * Mathf.Clamp(_cachedYaw, 0f, 180f) * 0.5f);
            }
        }

        public override IAngularConstraint CreateAngularConstraint(in IAngularConstraint.SetupData setupData)
        {
            return new BallSocketAngularConstraint(BuildAngularConfig());
        }

        public override void ApplyAngularConfig(IAngularConstraint angular)
        {
            if (angular is BallSocketAngularConstraint ballSocket)
                ballSocket.ApplyConfig(BuildAngularConfig());
        }

        /// Packages the current ball-socket settings for the runtime angular constraint.
        private BallSocketAngularConstraint.Config BuildAngularConfig()
        {
            return new BallSocketAngularConstraint.Config(
                _localConstraintAxisRotation,
                SwingPitchHalfSin,
                SwingYawHalfSin,
                twistHalfAngle);
        }

        /// <summary>Converts a swing quaternion (x, y, 0, w) back to a direction on the sphere.</summary>
        /// <param name="swing">The swing quaternion to convert.</param>
        /// <param name="constraintRot">The constraint frame rotation to apply.</param>
        /// <returns>A world-space direction vector on the constraint sphere.</returns>
        public static Vector3 SwingToDirection(Quaternion swing, Quaternion constraintRot)
        {
            swing.Normalize();
            return constraintRot * (swing * Vector3.forward);
        }

        public Quaternion GetConstraintFrameRotation()
        {
            // Recompute from the serialized field so gizmos work in edit mode before Awake.
            return GetConstraintBaseRotation() * Quaternion.FromToRotation(Vector3.forward, constraintAxis.normalized);
        }

        /// <summary>Clamps the twist component around the forward axis to the allowed range.</summary>
        /// <param name="twist">The twist-only quaternion to clamp.</param>
        /// <param name="twistHalfAngle">Maximum allowed twist in degrees (symmetric around zero).</param>
        /// <returns>The clamped twist quaternion.</returns>
        public static Quaternion ClampTwist(Quaternion twist, float twistHalfAngle)
        {
            float angleRad = 2f * Mathf.Atan2(twist.z, twist.w);
            float signedDeg = Mathf.DeltaAngle(0f, angleRad * Mathf.Rad2Deg);
            float clamped = Mathf.Clamp(signedDeg, -twistHalfAngle, twistHalfAngle);
            if (Mathf.Approximately(clamped, signedDeg))
                return twist;
            return Quaternion.AngleAxis(clamped, Vector3.forward);
        }

        /// <summary>Clamps a swing quaternion to an elliptical cone defined by pitch and yaw half-angle sines.</summary>
        /// <param name="swing">The swing-only quaternion (z component is zero).</param>
        /// <param name="pitchHalfSin">Sine of half the pitch limit angle.</param>
        /// <param name="yawHalfSin">Sine of half the yaw limit angle.</param>
        /// <returns>The clamped swing quaternion.</returns>
        public static Quaternion ClampSwing(Quaternion swing, float pitchHalfSin, float yawHalfSin)
        {
            const float SWING_EPSILON = 1e-4f;
            float a = yawHalfSin;
            float b = pitchHalfSin;

            // Swing quaternions store the cone direction in x/y. The ellipse equation clamps
            // asymmetric yaw/pitch limits while preserving the closest reachable direction.
            if (a < SWING_EPSILON && b < SWING_EPSILON)
                return Quaternion.identity;

            if (a < SWING_EPSILON)
            {
                float cy = Mathf.Clamp(swing.y, -b, b);
                float s2 = 1f - cy * cy;
                return new Quaternion(0f, cy, 0f, s2 > 0f ? Mathf.Sqrt(s2) : 0f);
            }

            if (b < SWING_EPSILON)
            {
                float cx = Mathf.Clamp(swing.x, -a, a);
                float s2 = 1f - cx * cx;
                return new Quaternion(cx, 0f, 0f, s2 > 0f ? Mathf.Sqrt(s2) : 0f);
            }

            float ex = swing.x / a;
            float ey = swing.y / b;
            if (ex * ex + ey * ey <= 1f)
                return swing;

            Vector2 closest = ClosestPointOnEllipse(swing.x, swing.y, a, b);
            float sNew2 = 1f - closest.x * closest.x - closest.y * closest.y;
            return new Quaternion(closest.x, closest.y, 0f, sNew2 > 0f ? Mathf.Sqrt(sNew2) : 0f);
        }

        /// <summary>Extracts the swing component of a quaternion around the forward (Z) axis.</summary>
        /// <param name="q">The quaternion to decompose.</param>
        /// <returns>A swing quaternion with z=0, representing rotation perpendicular to forward.</returns>
        public static Quaternion ExtractSwingAroundForwardVector(Quaternion q)
        {
            float w = q.w, z = q.z;
            float s = Mathf.Sqrt(w * w + z * z);
            if (s < 1e-6f)
                return q;

            // Remove the Z-axis twist component, leaving only the rotation that moves forward.
            float invS = 1f / s;
            return new Quaternion(
                (w * q.x - q.y * z) * invS,
                (w * q.y + q.x * z) * invS,
                0f,
                s);
        }

        /// <summary>Extracts the twist component of a quaternion around the forward (Z) axis.</summary>
        /// <param name="q">The quaternion to decompose.</param>
        /// <returns>A twist quaternion with x=0 and y=0, representing rotation around forward.</returns>
        public static Quaternion ExtractTwistAroundForwardVector(Quaternion q)
        {
            float w = q.w, z = q.z;
            float s = Mathf.Sqrt(w * w + z * z);
            if (s < 1e-6f)
                return Quaternion.identity;

            // Keep only the normalized quaternion projection around the local forward axis.
            float invS = 1f / s;
            return new Quaternion(0f, 0f, z * invS, w * invS);
        }

        /// <summary>Finds the closest point on an axis-aligned ellipse to a given point using Newton's method.</summary>
        /// <param name="px">X coordinate of the query point.</param>
        /// <param name="py">Y coordinate of the query point.</param>
        /// <param name="a">Ellipse semi-axis length along X.</param>
        /// <param name="b">Ellipse semi-axis length along Y.</param>
        /// <returns>The closest point on the ellipse.</returns>
        private static Vector2 ClosestPointOnEllipse(float px, float py, float a, float b)
        {
            float t = Mathf.Atan2(a * py, b * px);
            float a2 = a * a;
            float b2 = b * b;
            float a2MinusB2 = a2 - b2;

            // Newton's method solves for the ellipse parameter whose normal passes through the query point.
            for (int i = 0; i < 8; i++)
            {
                float sinT = Mathf.Sin(t);
                float cosT = Mathf.Cos(t);
                float f = a2MinusB2 * sinT * cosT - px * a * sinT + py * b * cosT;
                float fPrime = a2MinusB2 * (cosT * cosT - sinT * sinT) - px * a * cosT - py * b * sinT;
                if (Mathf.Abs(fPrime) < 1e-10f)
                    break;
                float dt = f / fPrime;
                t -= dt;
                if (Mathf.Abs(dt) < 1e-6f)
                    break;
            }

            return new Vector2(a * Mathf.Cos(t), b * Mathf.Sin(t));
        }
    }
}
