using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEditorInternal;
using UnityEngine;

namespace OpenIK.Editor
{
    // Editing helpers shared by the Multi FABRIK solver and chain inspectors.
    internal static class MultiFABRIKEditorUtility
    {
        internal const string JunctionJointTooltip = "Joint 0 is the junction where this branch begins. It is this component's transform "
            + "and is included in the chain in both modes. Place the FABRIK Chain component on this joint.";

        internal const string RigidModeDescription = "Joint 0 turns as one rigid piece with every chain on it. "
            + "Each chain's joint 1 keeps a fixed offset from joint 0.";

        internal const string PointModeDescription = "Each chain turns on its own pivot, its joint 1. "
            + "Make the pivots direct children of joint 0 at the same local position, and put joint limits on the pivots.";

        internal static string BranchModeTooltip(Transform joint)
        {
            string name = joint != null ? joint.name : "joint 0";
            return $"How the chains on {name} connect to it. The mode applies to every chain on this joint.\n\n"
                + "• Rigid: " + RigidModeDescription + "\n"
                + "• Point: " + PointModeDescription + "\n\n"
                + "At a Point junction, a chain that continues through the joint also needs its own pivot. "
                + "Changing the mode keeps the joints and hierarchy.";
        }

        internal const string PivotTooltip = "Joint 1 is this branch's independent pivot at a Point junction. "
            + "It must be a direct child of joint 0 and share the same local position as the other pivots. "
            + "The common offset can be zero or nonzero. Joint constraints should go on the pivots.";

        // The chain's pivot at a Point junction: its first joint after the junction. Null for Rigid
        // junctions and chains without one.
        internal static Transform Pivot(MultiFABRIKChain chain) =>
            chain.Junction == MultiFABRIKJunctionMode.Point && chain.Joints.Count > 1 ? chain.Joints[1] : null;

        // Sets the junction of every chain on the joint as one Undo step.
        internal static void SetJunction(Transform joint, MultiFABRIKJunctionMode mode)
        {
            MultiFABRIKChain[] chains = joint.GetComponents<MultiFABRIKChain>();
            if (chains.Length == 0) return;
            using (var serialized = new SerializedObject(chains))
            {
                serialized.FindProperty("junction").enumValueIndex = (int)mode;
                serialized.ApplyModifiedProperties();
            }
            InternalEditorUtility.RepaintAllViews();
        }

        // The solver whose Chains list holds this chain: one on a parent first, then any in the scene.
        internal static MultiFABRIKSolver FindSolver(MultiFABRIKChain chain)
        {
            foreach (MultiFABRIKSolver solver in chain.GetComponentsInParent<MultiFABRIKSolver>(true))
                if (solver.Lists(chain)) return solver;
            if (!chain.gameObject.scene.IsValid()) return null;
            foreach (GameObject root in chain.gameObject.scene.GetRootGameObjects())
                foreach (MultiFABRIKSolver solver in root.GetComponentsInChildren<MultiFABRIKSolver>(true))
                    if (solver.Lists(chain)) return solver;
            return null;
        }

        // The chains in dragged objects: chain components, and the chains on dragged GameObjects and
        // Transforms and on everything below them.
        internal static List<MultiFABRIKChain> ChainsIn(IEnumerable<UnityEngine.Object> objects)
        {
            var result = new List<MultiFABRIKChain>();
            foreach (UnityEngine.Object dropped in objects)
            {
                if (dropped is MultiFABRIKChain chain) { if (!result.Contains(chain)) result.Add(chain); continue; }
                GameObject gameObject = dropped as GameObject ?? (dropped as Component)?.gameObject;
                if (gameObject == null || EditorUtility.IsPersistent(gameObject)) continue;
                foreach (MultiFABRIKChain found in gameObject.GetComponentsInChildren<MultiFABRIKChain>(true))
                    if (!result.Contains(found)) result.Add(found);
            }
            return result;
        }

        // Adds the chains that are not listed yet to the solver's Chains list, with Pull 1 and Pin 0,
        // as one Undo step. Returns how many were added.
        internal static int AddToSolver(MultiFABRIKSolver solver, IEnumerable<MultiFABRIKChain> chains)
        {
            int added = 0;
            using (var serialized = new SerializedObject(solver))
            {
                SerializedProperty list = serialized.FindProperty("chains");
                foreach (MultiFABRIKChain chain in chains)
                {
                    if (chain == null || solver.Lists(chain) || Listed(list, chain)) continue;
                    int index = list.arraySize++;
                    SerializedProperty item = list.GetArrayElementAtIndex(index);
                    item.FindPropertyRelative("chain").objectReferenceValue = chain;
                    item.FindPropertyRelative("pull").floatValue = 1f;
                    item.FindPropertyRelative("pin").floatValue = 0f;
                    added++;
                }
                serialized.ApplyModifiedProperties();
            }
            if (added > 0) InternalEditorUtility.RepaintAllViews();
            return added;
        }

        private static bool Listed(SerializedProperty list, MultiFABRIKChain chain)
        {
            for (int i = 0; i < list.arraySize; i++)
                if (list.GetArrayElementAtIndex(i).FindPropertyRelative("chain").objectReferenceValue == chain) return true;
            return false;
        }

        // The problems whose message is about this chain or about a point junction on its joint.
        internal static void ProblemsFor(MultiFABRIKChain chain, List<string> problems, List<string> result)
        {
            result.Clear();
            string label = MultiFABRIKTree.Label(chain) + ":";
            string junction = $"Point junction {chain.name}:";
            foreach (string problem in problems)
                if (problem.StartsWith(label, StringComparison.Ordinal) || problem.StartsWith(junction, StringComparison.Ordinal))
                    result.Add(problem);
        }

        // Messages for other OpenIK solvers in the scene that also move joints of this solver.
        internal static void FindOverlaps(MultiFABRIKSolver solver, IReadOnlyList<MultiFABRIKChain> chains, List<string> result)
        {
            result.Clear();
            var joints = new HashSet<Transform>();
            foreach (MultiFABRIKChain chain in chains)
                foreach (Transform joint in chain.Joints)
                    if (joint != null) joints.Add(joint);
            if (joints.Count == 0 || !solver.gameObject.scene.IsValid()) return;

            foreach (GameObject root in solver.gameObject.scene.GetRootGameObjects())
            {
                foreach (OpenIKSolverBase other in root.GetComponentsInChildren<OpenIKSolverBase>(true))
                {
                    if (other == solver) continue;
                    Transform shared = null;
                    if (other is SingleChainSolverBase single)
                    {
                        foreach (Transform joint in single.ChainJoints)
                            if (joint != null && joints.Contains(joint)) { shared = joint; break; }
                    }
                    else if (other is MultiFABRIKSolver multi)
                    {
                        foreach (MultiFABRIKChain chain in multi.Chains)
                        {
                            foreach (Transform joint in chain.Joints)
                                if (joint != null && joints.Contains(joint)) { shared = joint; break; }
                            if (shared != null) break;
                        }
                    }
                    if (shared != null)
                        result.Add($"{shared.name} is also a joint of the {ObjectNames.NicifyVariableName(other.GetType().Name)} on {other.name}. "
                            + "Both solvers write the same transforms; remove the joint from one of them.");
                }
            }
        }
    }
}
