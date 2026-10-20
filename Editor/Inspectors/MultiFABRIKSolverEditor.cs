using System.Collections.Generic;
using UnityEditor;
using UnityEditorInternal;
using UnityEngine;

namespace OpenIK.Editor
{
    /// <summary>
    /// Draws the Multi FABRIK solver's Chains list, where chain components are dragged in and each chain's target,
    /// junction, Pull, and Pin are set, with the tree's setup problems below it.
    /// </summary>
    [CustomEditor(typeof(MultiFABRIKSolver))]
    public sealed class MultiFABRIKSolverEditor : OpenIKSolverEditor
    {
        private const string DropHint = "Drag FABRIK Chain components here, or the joints that hold them. "
            + "Dragging a joint adds the chains on it and below it.";

        private readonly List<MultiFABRIKChain> _chains = new();
        private readonly List<string> _problems = new();
        private readonly List<string> _own = new();
        private readonly List<string> _overlaps = new();
        private MultiFABRIKTree _tree;
        private bool _valid;
        private readonly Dictionary<MultiFABRIKChain, SerializedObject> _chainObjects = new();

        // The list shows its items in tree order, indented by depth, without changing the stored order.
        // Each row's item index in the stored list, and its depth in the tree.
        private readonly List<int> _order = new();
        private readonly List<int> _depths = new();
        private const float IndentWidth = 15f;
        private SerializedProperty _items;
        private ReorderableList _list;
        private GUIStyle _chainReferenceStyle;
        private MultiFABRIKSolver Solver => (MultiFABRIKSolver)target;
        private static float Line => EditorGUIUtility.singleLineHeight;
        private static float Gap => EditorGUIUtility.standardVerticalSpacing;

        private void OnDisable()
        {
            foreach (SerializedObject chainObject in _chainObjects.Values) chainObject.Dispose();
            _chainObjects.Clear();
        }

        protected override void DrawChainSection()
        {
            _tree = new MultiFABRIKTree();
            _valid = Solver.BuildTree(_chains, _tree, _problems);
            _items = serializedObject.FindProperty("chains");
            _list ??= CreateList();

            EditorGUILayout.Space();
            Rect header = EditorGUILayout.GetControlRect();
            _items.isExpanded = EditorGUI.Foldout(header, _items.isExpanded,
                new GUIContent(_items.arraySize > 0 ? $"Chains ({_items.arraySize})" : "Chains", DropHint), true, EditorStyles.foldoutHeader);
            HandleDrop(header);
            if (_items.isExpanded)
            {
                bool playing = Application.isPlaying;
                _list.displayAdd = _list.displayRemove = !playing;
                UpdateOrder();
                _list.DoLayoutList();
                HandleDrop(GUILayoutUtility.GetLastRect());
                if (playing)
                    EditorGUILayout.HelpBox("Change the Chains list outside Play mode. Scripts can use AddChain and RemoveChain to rebuild at runtime.", MessageType.Info);
            }

            foreach (string problem in _problems)
                EditorGUILayout.HelpBox(problem, MessageType.Error);
            for (int i = 0; i < _chains.Count; i++)
                if (HasUnpinnedTarget(_chains, i, Solver.GetPin(_chains[i])))
                    EditorGUILayout.HelpBox($"{_chains[i].ChainName}: Pin is 0 and other chains branch from this target's tip. "
                        + "Their pull can keep this tip away from its own target.", MessageType.Warning);
            MultiFABRIKEditorUtility.FindOverlaps(Solver, _chains, _overlaps);
            foreach (string overlap in _overlaps)
                EditorGUILayout.HelpBox(overlap, MessageType.Warning);
        }

        // The chain component's serialized fields, which this list and the chain Inspector both edit.
        internal SerializedObject ChainObject(MultiFABRIKChain chain)
        {
            if (!_chainObjects.TryGetValue(chain, out SerializedObject chainObject) || chainObject.targetObject == null)
            {
                chainObject = new SerializedObject(chain);
                _chainObjects[chain] = chainObject;
            }
            return chainObject;
        }

        protected override void DrawSolverSettings()
        {
            EditorGUILayout.PropertyField(serializedObject.FindProperty("rootPin"));
        }

        private ReorderableList CreateList()
        {
            // The rows follow the tree, so they cannot be reordered by dragging.
            return new ReorderableList(serializedObject, _items, false, false, true, true)
            {
                elementHeightCallback = row => (5f + (HasStatus(row) ? 1f : 0f)) * (Line + Gap) + Gap,
                drawElementCallback = (rect, row, active, focused) => DrawRow(rect, row),
                drawNoneElementCallback = rect => EditorGUI.LabelField(rect, "Drag chains here.", EditorStyles.miniLabel),
                onAddCallback = list =>
                {
                    // Unity copies the preceding item when it extends a list.
                    int index = _items.arraySize++;
                    SerializedProperty item = _items.GetArrayElementAtIndex(index);
                    item.FindPropertyRelative("chain").objectReferenceValue = null;
                    item.FindPropertyRelative("pull").floatValue = 1f;
                    item.FindPropertyRelative("pin").floatValue = 0f;
                    list.index = index;
                },
                onRemoveCallback = list =>
                {
                    if (list.index < 0 || list.index >= _order.Count) return;
                    _items.DeleteArrayElementAtIndex(_order[list.index]);
                    list.index = -1;
                }
            };
        }

        // Puts the listed chains in tree order, then the items that are not in the tree, such as empty
        // items and repeats, in stored order.
        private void UpdateOrder()
        {
            _order.Clear();
            _depths.Clear();
            var placed = new bool[_items.arraySize];
            for (int c = 0; c < _chains.Count; c++)
            {
                for (int i = 0; i < _items.arraySize; i++)
                {
                    if (placed[i] || _items.GetArrayElementAtIndex(i).FindPropertyRelative("chain").objectReferenceValue != _chains[c]) continue;
                    placed[i] = true;
                    _order.Add(i);
                    _depths.Add(_valid ? Depth(c) : 0);
                    break;
                }
            }
            for (int i = 0; i < placed.Length; i++)
            {
                if (placed[i]) continue;
                _order.Add(i);
                _depths.Add(0);
            }
        }

        // The number of chains between the root and this chain.
        private int Depth(int treeIndex)
        {
            int depth = 0;
            for (int branch = _tree.Chains[treeIndex].BranchIndex; branch > 0; branch = _tree.Chains[_tree.Owners[branch]].BranchIndex)
                depth++;
            return depth;
        }

        // Indents a row by its depth and keeps the value columns of all rows aligned.
        private void DrawRow(Rect rect, int row)
        {
            if (row >= _order.Count) return;
            float indent = _depths[row] * IndentWidth;
            if (indent > 0f)
            {
                Color guide = EditorGUIUtility.isProSkin ? new Color(1f, 1f, 1f, 0.15f) : new Color(0f, 0f, 0f, 0.2f);
                for (int level = 1; level <= _depths[row]; level++)
                    EditorGUI.DrawRect(new Rect(rect.x + level * IndentWidth - IndentWidth * 0.5f, rect.y + 2f, 1f, rect.height - 4f), guide);
            }
            float labelWidth = EditorGUIUtility.labelWidth;
            EditorGUIUtility.labelWidth = Mathf.Max(60f, labelWidth - indent);
            try { DrawItem(new Rect(rect.x + indent, rect.y, rect.width - indent, rect.height), _order[row]); }
            finally { EditorGUIUtility.labelWidth = labelWidth; }
        }

        private void DrawItem(Rect rect, int index)
        {
            if (index >= _items.arraySize) return;
            SerializedProperty item = _items.GetArrayElementAtIndex(index);
            var chain = item.FindPropertyRelative("chain").objectReferenceValue as MultiFABRIKChain;
            int treeIndex = chain != null ? _chains.IndexOf(chain) : -1;
            var row = new Rect(rect.x, rect.y + Gap, rect.width, Line);

            string name = chain == null ? "Empty" : string.IsNullOrEmpty(chain.ChainName) ? chain.name : chain.ChainName;
            using (new EditorGUI.DisabledScope(Application.isPlaying))
            {
                Rect field = EditorGUI.PrefixLabel(row, new GUIContent(name, "The FABRIK Chain component. Select it to edit its joints, tolerance, and poles."),
                    EditorStyles.boldLabel);
                DrawChainReference(field, item.FindPropertyRelative("chain"));
            }

            row.y += Line + Gap;
            DrawChainFields(row, chain);

            row.y += 2f * (Line + Gap);
            string pullReason = treeIndex >= 0 ? PullDisabledReason(_chains, treeIndex, Solver.RootPin) : null;
            using (new EditorGUI.DisabledScope(chain == null || !chain.enabled || pullReason != null))
                EditorGUI.PropertyField(row, item.FindPropertyRelative("pull"), new GUIContent("Pull", pullReason ?? "How strongly this chain pulls the joint it starts from, from 0 to 1."));

            row.y += Line + Gap;
            bool branches = treeIndex >= 0 && HasOwnedBranches(_chains, treeIndex);
            using (new EditorGUI.DisabledScope(chain == null || !chain.enabled || !branches))
                EditorGUI.PropertyField(row, item.FindPropertyRelative("pin"), new GUIContent("Pin", branches
                    ? "How strongly this chain resists the chains branching from its joints, from 0 to 1."
                    : "No other chain branches from this chain's joints, so Pin has no effect."));

            row.y += Line + Gap;
            string status = Status(chain, out bool problem);
            if (status != null)
                EditorGUI.LabelField(row, new GUIContent(status,
                    problem ? EditorGUIUtility.IconContent("console.erroricon.sml").image : null,
                    problem ? string.Join("\n", _own) : null), EditorStyles.miniLabel);
        }

        // Keep Unity's reference editing, picker, and prefab menus; redraw only the displayed value.
        private void DrawChainReference(Rect rect, SerializedProperty property)
        {
            string controlName = "OpenIKChain:" + property.propertyPath;
            GUI.SetNextControlName(controlName);
            EditorGUI.PropertyField(rect, property, GUIContent.none);
            if (Event.current.type != EventType.Repaint || property.hasMultipleDifferentValues) return;
            var chain = property.objectReferenceValue as MultiFABRIKChain;
            // Preserve Unity's diagnostic for a missing component reference.
            if (chain == null && property.objectReferenceInstanceIDValue != 0) return;

            var content = new GUIContent(EditorGUIUtility.ObjectContent(chain, typeof(MultiFABRIKChain)))
            {
                text = chain != null ? chain.name : "None (FABRIK Chain)"
            };
            _chainReferenceStyle ??= new GUIStyle(EditorStyles.objectField);
            _chainReferenceStyle.fontStyle = property.prefabOverride ? FontStyle.Bold : EditorStyles.objectField.fontStyle;
            Rect valueRect = _chainReferenceStyle.padding.Remove(rect);
            // Leave the native field border and its 19-point object-picker button visible.
            valueRect.xMax = Mathf.Min(valueRect.xMax, rect.xMax - 19f);
            if (valueRect.width <= 0f || valueRect.height <= 0f) return;
            bool hover = rect.Contains(Event.current.mousePosition);
            bool focused = GUI.GetNameOfFocusedControl() == controlName;
            Vector2 iconSize = EditorGUIUtility.GetIconSize();
            EditorGUIUtility.SetIconSize(new Vector2(12f, 12f));
            GUI.BeginClip(valueRect);
            try
            {
                _chainReferenceStyle.Draw(new Rect(rect.position - valueRect.position, rect.size), content,
                    hover, false, hover && DragAndDrop.activeControlID != 0, focused);
            }
            finally
            {
                GUI.EndClip();
                EditorGUIUtility.SetIconSize(iconSize);
            }
        }

        // Edits the chain's target and junction mode.
        private void DrawChainFields(Rect row, MultiFABRIKChain chain)
        {
            var junctionLabel = new GUIContent("Junction Mode", MultiFABRIKEditorUtility.BranchModeTooltip(chain != null ? chain.transform : null));
            if (chain == null)
            {
                using (new EditorGUI.DisabledScope(true))
                {
                    EditorGUI.ObjectField(row, "Target", null, typeof(Transform), true);
                    row.y += Line + Gap;
                    EditorGUI.EnumPopup(row, junctionLabel, MultiFABRIKJunctionMode.Rigid);
                }
                return;
            }

            SerializedObject chainObject = ChainObject(chain);
            chainObject.Update();
            EditorGUI.BeginChangeCheck();
            EditorGUI.PropertyField(row, chainObject.FindProperty("target"), new GUIContent("Target", "Optional position target for the chain's tip."));
            if (EditorGUI.EndChangeCheck())
            {
                chainObject.ApplyModifiedProperties();
                InternalEditorUtility.RepaintAllViews();
            }

            // The mode belongs to the chain's joint, so changing it sets every chain there. The tree
            // reads it when it is built, so it cannot change on a running solver.
            row.y += Line + Gap;
            EditorGUI.BeginChangeCheck();
            MultiFABRIKJunctionMode mode;
            using (new EditorGUI.DisabledScope(Application.isPlaying))
                mode = (MultiFABRIKJunctionMode)EditorGUI.EnumPopup(row, junctionLabel, chain.Junction);
            if (EditorGUI.EndChangeCheck())
                MultiFABRIKEditorUtility.SetJunction(chain.transform, mode);
        }

        private MultiFABRIKChain ChainAt(int row) => row < _order.Count
            ? _items.GetArrayElementAtIndex(_order[row]).FindPropertyRelative("chain").objectReferenceValue as MultiFABRIKChain
            : null;

        // The line below an item: its setup problem, or its latest result in Play mode. Null when
        // there is nothing to show.
        private string Status(MultiFABRIKChain chain, out bool problem)
        {
            problem = false;
            if (chain == null) return "Assign a chain or remove this item.";
            MultiFABRIKEditorUtility.ProblemsFor(chain, _problems, _own);
            if (_own.Count > 0)
            {
                problem = true;
                return _own.Count == 1 ? "Setup problem: hover for details." : $"{_own.Count} setup problems: hover for details.";
            }
            if (!Application.isPlaying) return null;
            MultiFABRIKChainResult result = chain.LastResult;
            return result.HasTarget ? $"Error {result.Error:0.####}, {(result.Converged ? "converged" : "unreached")}" : "No active target";
        }

        private bool HasStatus(int row)
        {
            return row < _order.Count && Status(ChainAt(row), out _) != null;
        }

        // Adds the chains in objects dragged onto the rect to the list.
        private void HandleDrop(Rect rect)
        {
            Event current = Event.current;
            if (Application.isPlaying || !rect.Contains(current.mousePosition)) return;
            if (current.type != EventType.DragUpdated && current.type != EventType.DragPerform) return;
            List<MultiFABRIKChain> dropped = MultiFABRIKEditorUtility.ChainsIn(DragAndDrop.objectReferences);
            if (dropped.Count == 0) return;
            DragAndDrop.visualMode = DragAndDropVisualMode.Copy;
            if (current.type == EventType.DragPerform)
            {
                DragAndDrop.AcceptDrag();
                serializedObject.ApplyModifiedProperties();
                MultiFABRIKEditorUtility.AddToSolver(Solver, dropped);
                serializedObject.Update();
                _items.isExpanded = true;
            }
            current.Use();
        }

        internal static bool HasOwnedBranches(IReadOnlyList<MultiFABRIKChain> chains, int index)
        {
            if (index >= chains.Count) return false;
            for (int c = 0; c < chains.Count; c++)
                for (int j = 1; j < chains[index].Joints.Count; j++)
                    if (c != index && chains[c].transform == chains[index].Joints[j]) return true;
            return false;
        }

        internal static bool HasUnpinnedTarget(IReadOnlyList<MultiFABRIKChain> chains, int index, float pin)
        {
            MultiFABRIKChain chain = chains[index];
            if (!chain.Enabled || chain.Target == null || pin > 0f || chain.Joints.Count < 2) return false;
            Transform tip = chain.Joints[chain.Joints.Count - 1];
            foreach (MultiFABRIKChain other in chains)
                if (other.transform == tip) return true;
            return false;
        }

        internal static string PullDisabledReason(IReadOnlyList<MultiFABRIKChain> chains, int index, float rootPin)
        {
            if (index >= chains.Count) return null;
            if (!chains[index].Enabled) return "Disabled chains do not pull the joint they start from.";
            Transform root = chains[0].transform;
            if (rootPin < 1f || chains[index].transform != root) return null;
            if (chains[index].Junction == MultiFABRIKJunctionMode.Point)
                return "Root Pin is 1, so the common attachment is fixed. Point roots keep their incoming local reference frame.";
            int rootChains = 0;
            foreach (MultiFABRIKChain chain in chains)
                if (chain.transform == root) rootChains++;
            return rootChains > 1 ? "Root Pin is 1, so this shared root keeps its position and rotation." : null;
        }
    }
}
