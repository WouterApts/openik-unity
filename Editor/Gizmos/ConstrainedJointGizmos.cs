using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

namespace OpenIK.Editor
{
    public static class ConstrainedJointGizmos
    {
        /// <summary>
        /// Draws the swing-cone center axis line and joint dot for a ball-socket joint.
        /// </summary>
        public static void DrawConstraintAxis(BallSocketIKJoint joint)
        {
            if (!joint.jointIsEnabled) return;

            Vector3 pos = joint.transform.position;
            Quaternion rot = joint.GetConstraintBaseRotation();
            Vector3 constraintAxisWorld = rot * joint.constraintAxis.normalized;
            float axisLength = 0.8f;
            Vector3 tip = pos + constraintAxisWorld * axisLength;

            var prevZTest = Handles.zTest;
            Handles.zTest = CompareFunction.Always;
            Handles.color = Color.yellow;
            Handles.DrawAAPolyLine(4f, pos, tip);
            Handles.SphereHandleCap(0, pos, Quaternion.identity, 0.08f, EventType.Repaint);
            Handles.zTest = prevZTest;
        }

        /// <summary>
        /// Returns true if this joint's gizmos should draw right now.
        /// </summary>
        public static bool ShouldDraw(ConstrainedJoint joint)
        {
            if (joint.GizmoMode == GizmoDrawMode.Always)
                return true;

            // SelectedOnly: only when this exact transform is selected
            return Selection.activeTransform == joint.transform;
        }
    }
}
