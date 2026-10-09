using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

namespace OpenIK.Editor
{
    public static class BallSocketIKJointGizmos
    {
        [DrawGizmo(GizmoType.Active | GizmoType.NonSelected | GizmoType.Selected)]
        private static void DrawGizmo(BallSocketIKJoint joint, GizmoType gizmoType)
        {
            if (!ConstrainedJointGizmos.ShouldDraw(joint)) return;
            if (!joint.jointIsEnabled) return;

            ConstrainedJointGizmos.DrawConstraintAxis(joint);

            if (joint.swingPitchHalfAngle >= 180f && joint.swingYawHalfAngle >= 180f) return;

            Vector3 pos = joint.transform.position;
            Quaternion constraintRot = joint.GetConstraintFrameRotation();

            float a = Mathf.Sin(Mathf.Deg2Rad * Mathf.Clamp(joint.swingYawHalfAngle, 0f, 180f) * 0.5f);
            float b = Mathf.Sin(Mathf.Deg2Rad * Mathf.Clamp(joint.swingPitchHalfAngle, 0f, 180f) * 0.5f);
            float radius = 0.8f;
            int segments = 48;

            var prevZTest = Handles.zTest;
            Handles.zTest = CompareFunction.Always;

            // Draw the boundary ellipse — matches ClampSwing's (x/a)^2 + (y/b)^2 = 1
            Handles.color = Color.yellow;
            Vector3 prevPoint = Vector3.zero;
            for (int i = 0; i <= segments; i++)
            {
                float t = (float)i / segments * Mathf.PI * 2f;
                float sx = a * Mathf.Cos(t);
                float sy = b * Mathf.Sin(t);
                float sw = Mathf.Sqrt(Mathf.Max(0f, 1f - sx * sx - sy * sy));
                Quaternion swing = new Quaternion(sx, sy, 0f, sw);

                Vector3 dir = BallSocketIKJoint.SwingToDirection(swing, constraintRot);
                Vector3 point = pos + dir * radius;

                if (i > 0)
                    Handles.DrawLine(prevPoint, point);
                prevPoint = point;
            }

            // Draw inner rings for visual fill
            for (int ring = 1; ring <= 5; ring++)
            {
                float scale = ring / 6f;
                Handles.color = new Color(1f, 1f, 0f, scale * 0.4f);
                Vector3 prevInner = Vector3.zero;
                for (int i = 0; i <= segments; i++)
                {
                    float t = (float)i / segments * Mathf.PI * 2f;
                    float sx = a * scale * Mathf.Cos(t);
                    float sy = b * scale * Mathf.Sin(t);
                    float sw = Mathf.Sqrt(Mathf.Max(0f, 1f - sx * sx - sy * sy));
                    Quaternion swing = new Quaternion(sx, sy, 0f, sw);

                    Vector3 dir = BallSocketIKJoint.SwingToDirection(swing, constraintRot);
                    Vector3 point = pos + dir * radius;

                    if (i > 0)
                        Handles.DrawLine(prevInner, point);
                    prevInner = point;
                }
            }

            Handles.zTest = prevZTest;
        }
    }
}
