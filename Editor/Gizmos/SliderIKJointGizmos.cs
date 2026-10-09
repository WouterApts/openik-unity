using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

namespace OpenIK.Editor
{
    public static class SliderIKJointGizmos
    {
        [DrawGizmo(GizmoType.Active | GizmoType.NonSelected | GizmoType.Selected)]
        private static void DrawGizmo(SliderIKJoint joint, GizmoType gizmoType)
        {
            if (!ConstrainedJointGizmos.ShouldDraw(joint)) return;
            if (!joint.jointIsEnabled) return;

            Vector3 pos = joint.transform.position;
            Vector3 axisLocal = joint.slideAxis.sqrMagnitude > 1e-8f
                ? joint.slideAxis.normalized
                : Vector3.forward;
            Vector3 axisWorld = joint.transform.rotation * axisLocal;
            float minLength = Mathf.Min(joint.minLength, joint.maxLength);
            float maxLength = Mathf.Max(joint.minLength, joint.maxLength);
            Vector3 start = pos + axisWorld * minLength;
            Vector3 end = pos + axisWorld * maxLength;

            var prevZTest = Handles.zTest;
            Handles.zTest = CompareFunction.Always;

            Handles.color = Color.yellow;
            Handles.SphereHandleCap(0, pos, Quaternion.identity, 0.08f, EventType.Repaint);
            Handles.DrawAAPolyLine(4f, start, end);
            Handles.SphereHandleCap(0, start, Quaternion.identity, 0.06f, EventType.Repaint);
            Handles.SphereHandleCap(0, end, Quaternion.identity, 0.06f, EventType.Repaint);

            Handles.color = new Color(1f, 1f, 0f, 0.3f);
            Handles.DrawDottedLine(pos, start, 4f);
            Handles.DrawDottedLine(pos, end, 4f);

            Handles.zTest = prevZTest;
        }
    }
}
