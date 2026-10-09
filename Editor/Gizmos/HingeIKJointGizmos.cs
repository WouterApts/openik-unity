using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

namespace OpenIK.Editor
{
    public static class HingeIKJointGizmos
    {
        [DrawGizmo(GizmoType.Active | GizmoType.NonSelected | GizmoType.Selected)]
        private static void DrawGizmo(HingeIKJoint joint, GizmoType gizmoType)
        {
            if (!ConstrainedJointGizmos.ShouldDraw(joint)) return;
            if (!joint.jointIsEnabled) return;

            Vector3 pos = joint.transform.position;
            Quaternion rot = joint.GetConstraintBaseRotation();
            Vector3 hingeAxisWorld = rot * joint.hingeAxis.normalized;

            // Compute zero reference from hingeAxis + zeroAngleOffset
            Vector3 hinge = joint.hingeAxis.normalized;
            Vector3 perp = Mathf.Abs(Vector3.Dot(hinge, Vector3.forward)) < 0.99f
                ? Vector3.forward
                : Vector3.up;
            Vector3 baseDir = Vector3.Cross(hinge, perp).normalized;
            Vector3 zeroRefLocal = Quaternion.AngleAxis(joint.zeroAngleOffset, hinge) * baseDir;
            Vector3 zeroRefWorld = rot * zeroRefLocal;

            float arcRadius = 0.8f;

            var prevZTest = Handles.zTest;
            Handles.zTest = CompareFunction.Always;

            // Draw joint dot
            Handles.color = Color.yellow;
            Handles.SphereHandleCap(0, pos, Quaternion.identity, 0.08f, EventType.Repaint);

            // Draw hinge axis
            Handles.DrawAAPolyLine(4f, pos - hingeAxisWorld * arcRadius * 0.4f, pos + hingeAxisWorld * arcRadius * 0.4f);

            // Draw arc showing hinge range
            int segments = 32;
            Handles.color = Color.yellow;
            Vector3 prevPoint = Vector3.zero;
            for (int i = 0; i <= segments; i++)
            {
                float t = (float)i / segments;
                float angle = Mathf.Lerp(joint.hingeMinAngle, joint.hingeMaxAngle, t);
                Quaternion arcRot = Quaternion.AngleAxis(angle, hingeAxisWorld);
                Vector3 point = pos + arcRot * zeroRefWorld * arcRadius;
                if (i > 0)
                    Handles.DrawLine(prevPoint, point);
                prevPoint = point;
            }

            // Draw zero-angle reference line
            Handles.color = new Color(1f, 1f, 0f, 0.2f);
            Handles.DrawLine(pos, pos + zeroRefWorld * arcRadius);

            // Draw lines from joint to arc extents
            Handles.color = new Color(1f, 1f, 0f, 0.5f);
            Vector3 minDir = Quaternion.AngleAxis(joint.hingeMinAngle, hingeAxisWorld) * zeroRefWorld;
            Vector3 maxDir = Quaternion.AngleAxis(joint.hingeMaxAngle, hingeAxisWorld) * zeroRefWorld;
            Handles.DrawLine(pos, pos + minDir * arcRadius);
            Handles.DrawLine(pos, pos + maxDir * arcRadius);

            Handles.zTest = prevZTest;
        }
    }
}
