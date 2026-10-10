using UnityEngine;

namespace OpenIK
{
    /// <summary>Limits a joint's swing to a cone around an axis, and its twist around that axis.</summary>
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

        // The speed limit applies to swing and twist together.
        public override JointMotionSupport MotionSupport => JointMotionSupport.Angular;

        // Rebuilds the constraint frame when the axis changes, and the swing sines when their angles
        // change. Twist is not cached; ApplyAngularConfig copies it before each solve.
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

        private BallSocketAngularConstraint.Config BuildAngularConfig()
        {
            return new BallSocketAngularConstraint.Config(
                _localConstraintAxisRotation,
                SwingPitchHalfSin,
                SwingYawHalfSin,
                twistHalfAngle);
        }

        /// <summary>Converts a swing rotation (x, y, 0, w) to the direction it turns the forward axis to.</summary>
        /// <param name="swing">The swing rotation, in the constraint frame.</param>
        /// <param name="constraintRot">The world rotation of the constraint frame.</param>
        /// <returns>The world-space direction.</returns>
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

        /// <summary>Clamps a twist around the forward axis to the allowed range.</summary>
        /// <param name="twist">A rotation around the forward axis only.</param>
        /// <param name="twistHalfAngle">Maximum twist in either direction, in degrees.</param>
        /// <returns>The clamped twist.</returns>
        public static Quaternion ClampTwist(Quaternion twist, float twistHalfAngle)
        {
            float angleRad = 2f * Mathf.Atan2(twist.z, twist.w);
            float signedDeg = Mathf.DeltaAngle(0f, angleRad * Mathf.Rad2Deg);
            float clamped = Mathf.Clamp(signedDeg, -twistHalfAngle, twistHalfAngle);
            if (Mathf.Approximately(clamped, signedDeg))
                return twist;
            return Quaternion.AngleAxis(clamped, Vector3.forward);
        }

        /// <summary>Clamps a swing to an elliptical cone set by the pitch and yaw limits.</summary>
        /// <param name="swing">A swing rotation, with a z component of zero.</param>
        /// <param name="pitchHalfSin">Sine of half the pitch limit angle.</param>
        /// <param name="yawHalfSin">Sine of half the yaw limit angle.</param>
        /// <returns>The clamped swing.</returns>
        public static Quaternion ClampSwing(Quaternion swing, float pitchHalfSin, float yawHalfSin)
        {
            const float SWING_EPSILON = 1e-4f;
            float a = yawHalfSin;
            float b = pitchHalfSin;

            // A swing stores its direction in x and y. Clamping to an ellipse handles different yaw
            // and pitch limits and keeps the closest direction inside them.
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

        /// <summary>Returns the swing part of a rotation: the part that moves the forward (Z) axis.</summary>
        /// <param name="q">The rotation to split.</param>
        /// <returns>The swing, with a z component of zero.</returns>
        public static Quaternion ExtractSwingAroundForwardVector(Quaternion q)
        {
            float w = q.w, z = q.z;
            float s = Mathf.Sqrt(w * w + z * z);
            if (s < 1e-6f)
                return q;

            // Remove the twist around Z, leaving only the rotation that moves the forward axis.
            float invS = 1f / s;
            return new Quaternion(
                (w * q.x - q.y * z) * invS,
                (w * q.y + q.x * z) * invS,
                0f,
                s);
        }

        /// <summary>Returns the twist part of a rotation: the rotation around the forward (Z) axis.</summary>
        /// <param name="q">The rotation to split.</param>
        /// <returns>The twist, with x and y components of zero.</returns>
        public static Quaternion ExtractTwistAroundForwardVector(Quaternion q)
        {
            float w = q.w, z = q.z;
            float s = Mathf.Sqrt(w * w + z * z);
            if (s < 1e-6f)
                return Quaternion.identity;

            // Keep only the rotation around the forward axis, normalized.
            float invS = 1f / s;
            return new Quaternion(0f, 0f, z * invS, w * invS);
        }

        // Closest point to (px, py) on an axis-aligned ellipse with half-axes a (X) and b (Y).
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
