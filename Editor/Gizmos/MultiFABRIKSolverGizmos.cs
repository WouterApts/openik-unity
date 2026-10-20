using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace OpenIK.Editor
{
    internal static class MultiFABRIKSolverGizmos
    {
        private static readonly List<string> Problems = new();
        private static readonly List<MultiFABRIKChain> Chains = new();
        private static readonly Color JunctionColor = new(0.247f, 0.663f, 1f);

        // With Selected Only, the tree draws while the solver or a joint holding one of its chains is
        // selected. The chains on a selected joint draw highlighted, with their pole sections.
        [DrawGizmo(GizmoType.Active | GizmoType.NonSelected | GizmoType.Selected)]
        private static void DrawSolverGizmo(MultiFABRIKSolver solver, GizmoType gizmoType)
        {
            Transform selected = Selection.activeTransform;
            bool solverSelected = selected == solver.transform;
            bool jointSelected = false;
            if (selected != null)
                foreach (MultiFABRIKChain onJoint in selected.GetComponents<MultiFABRIKChain>())
                    jointSelected |= solver.Lists(onJoint);
            if (solver.BoneGizmoMode == GizmoDrawMode.SelectedOnly && !solverSelected && !jointSelected) return;

            var tree = new MultiFABRIKTree();
            Chains.Clear();
            Chains.AddRange(solver.Chains);
            bool valid = tree.Build(Chains, Problems);
            if (Chains.Count == 0) return;
            Color previous = Handles.color;
            int rootChains = 0;
            foreach (MultiFABRIKChain chain in Chains)
                if (chain.transform == Chains[0].transform) rootChains++;
            for (int c = 0; c < Chains.Count; c++)
            {
                MultiFABRIKChain chain = Chains[c];
                Transform branch = chain.transform;
                bool highlighted = jointSelected && branch == selected;
                bool point = chain.Junction == MultiFABRIKJunctionMode.Point;
                bool root = c == 0 || branch == Chains[0].transform;
                // A single root chain turns its root like plain FABRIK; other rigid first bones are fixed offsets.
                SolverGizmos.DrawBoneConnections(chain.Joints, !point && (!root || rootChains > 1), highlighted);
                SolverGizmos.DrawPoles(chain.Joints, chain.Poles, solverSelected || highlighted, valid ? tree.Chains[c].Blocked : null);
                Handles.color = chain.Enabled ? JunctionColor : Color.gray;
                if (point && chain.Joints.Count > 1)
                {
                    // One sphere marks the shared attachment of every chain on this joint.
                    bool drawn = false;
                    for (int earlier = 0; earlier < c; earlier++)
                        drawn |= Chains[earlier].transform == branch;
                    if (!drawn && chain.Joints[1] != null)
                        Handles.SphereHandleCap(0, chain.Joints[1].position, Quaternion.identity, 0.25f, EventType.Repaint);
                }
                else if (!root)
                    Handles.SphereHandleCap(0, branch.position, Quaternion.identity, 0.2f, EventType.Repaint);
                Transform tip = chain.Joints[chain.Joints.Count - 1];
                if (tip != null && chain.Target != null)
                    Handles.DrawDottedLine(tip.position, chain.Target.position, 4f);
            }
            Handles.color = previous;
        }
    }
}
