using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

namespace OpenIK.Editor
{
    public static class SolverGizmos
    {
        // Registered for the single-chain base type, so built-in and custom single-chain solvers all draw their chain.
        private static readonly Color PoleColor = new(1f, 0.48f, 0.06f);
        private static readonly Color HighlightColor = new(1f, 0.55f, 0.5f);
        private static readonly List<PoleSection> Sections = new();

        // A fixed world size, like the joint spheres, so the cube does not grow when zooming out.
        private const float PoleGizmoSize = 0.2f;

        [DrawGizmo(GizmoType.Active | GizmoType.NonSelected | GizmoType.Selected)]
        private static void DrawSolverGizmo(SingleChainSolverBase solver, GizmoType gizmoType)
        {
            if (solver.BoneGizmoMode == GizmoDrawMode.Always || Selection.activeTransform == solver.transform)
                DrawBoneConnections(solver.ChainJoints);
            DrawPoles(solver);
        }

        // An orange cube at each pole target and a dotted line from its bend joint, fading with Weight.
        // With the solver selected, a faint triangle shows the section each pole turns.
        private static void DrawPoles(SingleChainSolverBase solver)
        {
            bool selected = Selection.activeTransform == solver.transform;
            if (solver.BoneGizmoMode == GizmoDrawMode.Always || selected)
                DrawPoles(solver.ChainJoints, solver.Poles, selected);
        }

        // With showSections, a faint triangle shows the section each pole turns.
        internal static void DrawPoles(IReadOnlyList<Transform> joints, IReadOnlyList<IKPole> poles, bool showSections, bool[] blocked = null)
        {
            if (poles.Count == 0)
                return;

            var prevZTest = Handles.zTest;
            Handles.zTest = CompareFunction.Always;

            foreach (IKPole pole in poles)
            {
                if (pole?.BendJoint == null || pole.Target == null || pole.Weight <= 0f)
                    continue;

                Vector3 polePosition = pole.Target.position;
                Handles.color = new Color(PoleColor.r, PoleColor.g, PoleColor.b, Mathf.Lerp(0.25f, 1f, pole.Weight));
                Handles.DrawDottedLine(pole.BendJoint.position, polePosition, 4f);
                Handles.CubeHandleCap(0, polePosition, Quaternion.identity, PoleGizmoSize, EventType.Repaint);
            }

            if (showSections)
            {
                PoleSectionResolver.Resolve(joints, poles, Sections, null);
                Handles.color = new Color(PoleColor.r, PoleColor.g, PoleColor.b, 0.08f);
                foreach (PoleSection section in Sections)
                {
                    if (poles[section.EntryIndex].Target == null || joints[section.Start] == null || joints[section.End] == null)
                        continue;
                    if (PoleSectionResolver.GetBlockedProblem(joints, section, blocked) != null)
                        continue;

                    Handles.DrawAAConvexPolygon(
                        joints[section.Start].position,
                        joints[section.Bend].position,
                        joints[section.End].position);
                }
            }

            Handles.zTest = prevZTest;
        }

        // A dotted first bone marks a fixed offset from a rigid junction, which cannot turn on its own.
        // Highlighted bones are drawn lighter and thicker.
        internal static void DrawBoneConnections(IReadOnlyList<Transform> joints, bool dottedFirstBone = false, bool highlighted = false)
        {
            if (joints == null || joints.Count < 2) return;

            var prevZTest = Handles.zTest;
            Handles.zTest = CompareFunction.Always;
            Handles.color = highlighted ? HighlightColor : Color.red;
            float thickness = highlighted ? 3f : 0f;
            for (int i = 0; i < joints.Count; i++)
            {
                if (joints[i] == null) continue;
                Handles.SphereHandleCap(0, joints[i].position, Quaternion.identity, 0.08f, EventType.Repaint);
                if (i == 1 && dottedFirstBone && joints[0] != null)
                    Handles.DrawDottedLine(joints[0].position, joints[1].position, 4f);
                else if (i > 0 && joints[i - 1] != null)
                    Handles.DrawLine(joints[i - 1].position, joints[i].position, thickness);
            }
            Handles.zTest = prevZTest;
        }
    }
}
