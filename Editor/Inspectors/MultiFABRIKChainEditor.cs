using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace OpenIK.Editor
{
    /// <summary>Draws one FABRIK Chain of a Multi FABRIK Solver: its solver, its junction, its target, joints, and poles, and its solving settings.</summary>
    [CustomEditor(typeof(MultiFABRIKChain))]
    public sealed class MultiFABRIKChainEditor : UnityEditor.Editor
    {
        private readonly List<MultiFABRIKChain> _chains = new();
        private readonly List<string> _problems = new();
        private readonly List<string> _own = new();
        private MultiFABRIKTree _tree;
        private MultiFABRIKSolver _solver;
        private bool _valid;
        private int _index;
        private PoleListDrawer _poles;
        private MultiFABRIKJointList _joints;

        private MultiFABRIKChain Chain => (MultiFABRIKChain)target;

        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            Refresh();

            DrawStatus();
            EditorGUILayout.PropertyField(serializedObject.FindProperty("chainName"), new GUIContent("Name", "The chain's name in the inspectors and setup messages."));

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Chain", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("target"));
            (_joints ??= new MultiFABRIKJointList(serializedObject)).Draw();
            if (Application.isPlaying)
                EditorGUILayout.HelpBox("Change joints outside Play mode. Scripts can use AddChain and RemoveChain to rebuild at runtime.", MessageType.Info);
            Poles.Draw();

            EditorGUILayout.Space();
            DrawBranchMode();
            EditorGUILayout.HelpBox(Chain.Junction == MultiFABRIKJunctionMode.Point
                ? MultiFABRIKEditorUtility.PointModeDescription
                : MultiFABRIKEditorUtility.RigidModeDescription, MessageType.Info);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Solving", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("tolerance"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("useSingularityHandling"), new GUIContent("Singularity Handling",
                "Help a nearly straight chain bend toward a target aligned with it."));

            MultiFABRIKEditorUtility.ProblemsFor(Chain, _problems, _own);
            foreach (string problem in _own)
                EditorGUILayout.HelpBox(problem, MessageType.Error);
            if (_own.Count == 0 && !_valid && _solver != null)
                EditorGUILayout.HelpBox("The solver has setup problems in other chains. Select it to see them.", MessageType.Info);

            if (Application.isPlaying)
            {
                MultiFABRIKChainResult result = Chain.LastResult;
                EditorGUILayout.LabelField(result.HasTarget ? $"Error: {result.Error:G4}  {(result.Converged ? "Converged" : "Unreached")}" : "No active target",
                    EditorStyles.miniLabel);
            }

            serializedObject.ApplyModifiedProperties();
        }

        private PoleListDrawer Poles => _poles ??= new PoleListDrawer(serializedObject, serializedObject.FindProperty("poles"),
            () => Chain.Joints, () => Chain.Poles, () => Chain.Target,
            () => _valid && _index >= 0 ? _tree.Chains[_index].Blocked : null);

        private void Refresh()
        {
            _solver = MultiFABRIKEditorUtility.FindSolver(Chain);
            _chains.Clear();
            _problems.Clear();
            _valid = false;
            _index = -1;
            if (_solver == null) return;
            _tree = new MultiFABRIKTree();
            _valid = _solver.BuildTree(_chains, _tree, _problems);
            _index = _chains.IndexOf(Chain);
        }

        // The solver this chain belongs to, and the chain's place in the tree.
        private void DrawStatus()
        {
            if (_solver == null)
            {
                EditorGUILayout.HelpBox("Not in a Multi FABRIK Solver's Chains list. Drag this component into the list on the solver.", MessageType.Warning);
                using (new EditorGUI.DisabledScope(Application.isPlaying))
                {
                    var parentSolver = Chain.GetComponentInParent<MultiFABRIKSolver>(true);
                    if (parentSolver != null && GUILayout.Button($"Add to the Multi FABRIK Solver on {parentSolver.name}"))
                    {
                        MultiFABRIKEditorUtility.AddToSolver(parentSolver, new[] { Chain });
                        GUIUtility.ExitGUI();
                    }
                }
                return;
            }

            using (new EditorGUI.DisabledScope(true))
                EditorGUILayout.ObjectField(new GUIContent("Solver", "The Multi FABRIK Solver whose Chains list holds this chain. It holds this chain's Pull and Pin, Root Pin, and the solving settings."),
                    _solver, typeof(MultiFABRIKSolver), true);
            EditorGUILayout.LabelField(" ", Place(), EditorStyles.miniLabel);
        }

        private string Place()
        {
            if (!_valid || _index < 0) return "Place in the tree unknown until the setup problems are fixed.";
            int branch = _tree.Chains[_index].BranchIndex;
            if (branch == 0) return "Root chain";
            MultiFABRIKChain owner = _chains[_tree.Owners[branch]];
            return $"Branches from {(string.IsNullOrEmpty(owner.ChainName) ? owner.name : owner.ChainName)}";
        }

        // The mode belongs to this joint, so changing it sets every chain on it.
        private void DrawBranchMode()
        {
            SerializedProperty junction = serializedObject.FindProperty("junction");
            // The tree reads the mode when it is built, so it cannot change on a running solver.
            EditorGUI.BeginChangeCheck();
            MultiFABRIKJunctionMode mode;
            using (new EditorGUI.DisabledScope(Application.isPlaying))
                mode = (MultiFABRIKJunctionMode)EditorGUILayout.EnumPopup(new GUIContent("Junction Mode", MultiFABRIKEditorUtility.BranchModeTooltip(Chain.transform)),
                    (MultiFABRIKJunctionMode)junction.enumValueIndex);
            if (!EditorGUI.EndChangeCheck()) return;
            serializedObject.ApplyModifiedProperties();
            MultiFABRIKEditorUtility.SetJunction(Chain.transform, mode);
            serializedObject.Update();
        }
    }
}
