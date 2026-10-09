using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

namespace OpenIK.Editor
{
    public static class SolverGizmos
    {
        [DrawGizmo(GizmoType.Active | GizmoType.NonSelected | GizmoType.Selected)]
        private static void DrawFABRIKGizmo(FABRIKSolver solver, GizmoType gizmoType)
        {
            DrawBoneConnections(solver.transform, solver.BoneGizmoMode, solver.ChainJoints);
        }

        [DrawGizmo(GizmoType.Active | GizmoType.NonSelected | GizmoType.Selected)]
        private static void DrawJacobianGizmo(JacobianIKSolver solver, GizmoType gizmoType)
        {
            DrawBoneConnections(solver.transform, solver.BoneGizmoMode, solver.ChainJoints);
        }

        [DrawGizmo(GizmoType.Active | GizmoType.NonSelected | GizmoType.Selected)]
        private static void DrawCCDGizmo(CCDIKSolver solver, GizmoType gizmoType)
        {
            DrawBoneConnections(solver.transform, solver.BoneGizmoMode, solver.ChainJoints);
        }

        private static void DrawBoneConnections(Transform solverTransform, GizmoDrawMode mode, System.Collections.Generic.IReadOnlyList<Transform> joints)
        {
            if (joints == null || joints.Count < 2) return;

            if (mode == GizmoDrawMode.SelectedOnly && Selection.activeTransform != solverTransform)
                return;

            var prevZTest = Handles.zTest;
            Handles.zTest = CompareFunction.Always;
            Handles.color = Color.red;
            for (int i = 0; i < joints.Count; i++)
            {
                if (joints[i] == null) continue;
                Handles.SphereHandleCap(0, joints[i].position, Quaternion.identity, 0.08f, EventType.Repaint);
                if (i > 0 && joints[i - 1] != null)
                    Handles.DrawLine(joints[i - 1].position, joints[i].position);
            }
            Handles.zTest = prevZTest;
        }
    }
}
