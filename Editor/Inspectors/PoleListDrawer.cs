using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEditorInternal;
using UnityEngine;

namespace OpenIK.Editor
{
    // Draws a solver's Poles foldout: collapsed by default, with a count in the header, a list of pole
    // entries, and one note when joint constraints make the poles best effort.
    internal sealed class PoleListDrawer
    {
        private const string BendJointField = "bendJoint";
        private const string TargetField = "target";
        private const string WeightField = "weight";
        private const float CreateButtonWidth = 56f;

        private readonly SerializedObject _serializedObject;
        private readonly SerializedProperty _poles;
        private readonly ReorderableList _list;
        private readonly List<PoleSection> _sections = new();
        private readonly Func<IReadOnlyList<Transform>> _joints;
        private readonly Func<IReadOnlyList<IKPole>> _entries;
        private readonly Func<Transform> _target;
        private readonly Func<bool[]> _blocked;

        public PoleListDrawer(SerializedObject serializedObject, SerializedProperty poles,
            Func<IReadOnlyList<Transform>> joints = null, Func<IReadOnlyList<IKPole>> entries = null,
            Func<Transform> target = null, Func<bool[]> blocked = null)
        {
            _serializedObject = serializedObject;
            _poles = poles;
            _joints = joints;
            _entries = entries;
            _target = target;
            _blocked = blocked;
            _list = new ReorderableList(serializedObject, poles, true, false, true, true)
            {
                drawElementCallback = DrawElement,
                elementHeightCallback = GetElementHeight,
                onAddCallback = AddPole
            };
        }

        private SingleChainSolverBase Solver => (SingleChainSolverBase)_serializedObject.targetObject;
        private IReadOnlyList<Transform> Joints => _joints != null ? _joints() : Solver.ChainJoints;
        private IReadOnlyList<IKPole> Entries => _entries != null ? _entries() : Solver.Poles;

        private static float Line => EditorGUIUtility.singleLineHeight;

        private static float Spacing => EditorGUIUtility.standardVerticalSpacing;

        public void Draw()
        {
            Rect rect = EditorGUI.IndentedRect(GUILayoutUtility.GetRect(0f, GetHeight(), GUILayout.ExpandWidth(true)));
            Draw(rect);
        }

        internal float GetHeight()
        {
            if (!_poles.isExpanded) return Line;
            if (_serializedObject.isEditingMultipleObjects) return Line * 3f + Spacing;
            return Line + Spacing + _list.GetHeight()
                + (_poles.arraySize > 0 && PoleSectionResolver.HasConstrainedJoint(Joints) ? Line * 3f + Spacing : 0f);
        }

        internal void Draw(Rect rect)
        {
            int count = _poles.arraySize;
            var label = new GUIContent(count > 0 ? $"Poles ({count})" : "Poles", _poles.tooltip);
            _poles.isExpanded = EditorGUI.Foldout(new Rect(rect.x, rect.y, rect.width, Line), _poles.isExpanded, label, true);
            if (!_poles.isExpanded)
                return;

            rect.y += Line + Spacing;
            rect.x += 12f;
            rect.width -= 12f;
            if (_serializedObject.isEditingMultipleObjects)
            {
                EditorGUI.HelpBox(new Rect(rect.x, rect.y, rect.width, Line * 2f), "Select one solver to edit poles.", MessageType.None);
                return;
            }

            PoleSectionResolver.Resolve(Joints, Entries, _sections, null);

            Rect listRect = new Rect(rect.x, rect.y, rect.width, _list.GetHeight());
            _list.DoList(listRect);

            if (count > 0 && PoleSectionResolver.HasConstrainedJoint(Joints))
            {
                EditorGUI.HelpBox(new Rect(rect.x, listRect.yMax + Spacing, rect.width, Line * 3f),
                    "Joint limits take priority over poles. A constrained chain may not reach the target or fully face its poles.",
                    MessageType.Warning);
            }

        }

        private static float GetElementHeight(int index)
        {
            return 4 * (Line + Spacing) + Spacing;
        }

        private void DrawElement(Rect rect, int index, bool isActive, bool isFocused)
        {
            SerializedProperty element = _poles.GetArrayElementAtIndex(index);
            SerializedProperty bendJoint = element.FindPropertyRelative(BendJointField);
            SerializedProperty target = element.FindPropertyRelative(TargetField);
            SerializedProperty weight = element.FindPropertyRelative(WeightField);

            // List elements draw their own indentation.
            int indent = EditorGUI.indentLevel;
            EditorGUI.indentLevel = 0;

            Rect row = new Rect(rect.x, rect.y + Spacing, rect.width, Line);
            DrawBendJointPopup(row, bendJoint);

            row.y += Line + Spacing;
            if (target.objectReferenceValue == null)
            {
                Rect field = new Rect(row.x, row.y, row.width - CreateButtonWidth - Spacing, row.height);
                EditorGUI.PropertyField(field, target, new GUIContent("Pole Target", target.tooltip));
                Rect button = new Rect(field.xMax + Spacing, row.y, CreateButtonWidth, row.height);
                using (new EditorGUI.DisabledScope(bendJoint.objectReferenceValue == null))
                {
                    if (GUI.Button(button, new GUIContent("Create", "Create a pole target on the side the section bends toward.")))
                        target.objectReferenceValue = CreatePoleTarget(bendJoint.objectReferenceValue as Transform);
                }
            }
            else
            {
                EditorGUI.PropertyField(row, target, new GUIContent("Pole Target", target.tooltip));
            }

            row.y += Line + Spacing;
            EditorGUI.Slider(row, weight, 0f, 1f, new GUIContent("Weight", weight.tooltip));

            row.y += Line + Spacing;
            EditorGUI.LabelField(row, GetSectionLabel(index), EditorStyles.miniLabel);

            EditorGUI.indentLevel = indent;
        }

        // A dropdown of the chain's middle joints. A joint outside that range stays visible so the
        // problem is clear.
        private void DrawBendJointPopup(Rect rect, SerializedProperty bendJoint)
        {
            IReadOnlyList<Transform> chain = Joints;
            var options = new List<Transform>();
            var labels = new List<GUIContent>();
            for (int i = 1; i < chain.Count - 1; i++)
            {
                if (chain[i] == null)
                    continue;

                options.Add(chain[i]);
                labels.Add(new GUIContent(HasDuplicateName(chain, i) ? $"{chain[i].name} ({i})" : chain[i].name));
            }

            var current = bendJoint.objectReferenceValue as Transform;
            int selected = options.IndexOf(current);
            if (selected < 0)
            {
                options.Insert(0, current);
                labels.Insert(0, new GUIContent(current == null ? "None" : $"{current.name} (not a middle joint)"));
                selected = 0;
            }

            var label = new GUIContent("Bend Joint", bendJoint.tooltip);
            int chosen = EditorGUI.Popup(rect, label, selected, labels.ToArray());
            if (chosen != selected)
                bendJoint.objectReferenceValue = options[chosen];
        }

        private GUIContent GetSectionLabel(int index)
        {
            IReadOnlyList<Transform> chain = Joints;
            foreach (PoleSection section in _sections)
            {
                if (section.EntryIndex != index || chain[section.Start] == null || chain[section.End] == null)
                    continue;

                string blocked = PoleSectionResolver.GetBlockedProblem(chain, section, _blocked?.Invoke());
                if (blocked != null)
                    return new GUIContent("Not used: section starts at or crosses a branch point.", blocked);

                return new GUIContent(
                    $"Section: {chain[section.Start].name} → {chain[section.End].name}",
                    "The joints that turn together so the bend joint faces the pole.");
            }

            // An entry without a section has a setup problem; show the reason on its own line.
            string problem = index < Entries.Count ? PoleSectionResolver.GetProblem(chain, Entries, index, _sections, out _) : null;
            return new GUIContent(problem ?? "Not used.");
        }

        // Adds an entry for the first middle joint without a pole, and creates its pole target.
        private void AddPole(ReorderableList list)
        {
            IReadOnlyList<Transform> chain = Joints;
            Transform bend = null;
            for (int i = 1; i < chain.Count - 1 && bend == null; i++)
            {
                if (chain[i] != null && !HasPoleFor(chain[i]))
                    bend = chain[i];
            }

            int index = _poles.arraySize;
            _poles.arraySize++;
            SerializedProperty element = _poles.GetArrayElementAtIndex(index);
            element.FindPropertyRelative(BendJointField).objectReferenceValue = bend;
            element.FindPropertyRelative(WeightField).floatValue = 1f;
            element.FindPropertyRelative(TargetField).objectReferenceValue = bend != null ? CreatePoleTarget(bend) : null;
            list.index = index;
        }

        private bool HasPoleFor(Transform joint)
        {
            for (int i = 0; i < _poles.arraySize; i++)
            {
                if (_poles.GetArrayElementAtIndex(i).FindPropertyRelative(BendJointField).objectReferenceValue == joint)
                    return true;
            }

            return false;
        }

        // Creates "<Bend Joint> Pole" next to the solver's target, one upper-segment length from the
        // bend joint, on the side the section bends toward.
        internal Transform CreatePoleTarget(Transform bendJoint)
        {
            Transform target = _target != null ? _target() : Solver.Target;
            Transform parent = target != null ? target.parent : ((Component)_serializedObject.targetObject).transform.parent;

            var pole = new GameObject($"{bendJoint.name} Pole");
            Undo.RegisterCreatedObjectUndo(pole, "Create Pole Target");
            pole.transform.SetParent(parent, false);
            pole.transform.position = GetPolePosition(Joints, bendJoint);
            return pole.transform;
        }

        private static Vector3 GetPolePosition(IReadOnlyList<Transform> chain, Transform bendJoint)
        {
            var sections = new List<PoleSection>();
            PoleSectionResolver.Resolve(chain, new[] { new IKPole(bendJoint, null) }, sections, null);
            if (sections.Count == 0)
                return bendJoint.position + Vector3.forward;

            PoleSection section = sections[0];
            Vector3 start = chain[section.Start].position;
            Vector3 bend = chain[section.Bend].position;
            Vector3 line = chain[section.End].position - start;
            float distance = Mathf.Max(Vector3.Distance(chain[section.Bend - 1].position, bend), 0.1f);
            if (line.sqrMagnitude < 1e-10f)
                return bend + Vector3.forward * distance;

            Vector3 axis = line.normalized;
            Vector3 direction = Vector3.ProjectOnPlane(bend - start, axis);
            if (direction.sqrMagnitude < 1e-8f && bendJoint.TryGetComponent(out HingeIKJoint hinge) && hinge.jointIsEnabled)
            {
                // A straight section bends to the side a positive hinge angle moves the bend joint.
                direction = Vector3.Cross(axis, bendJoint.rotation * hinge.hingeAxis);
                if (hinge.hingeMaxAngle < -hinge.hingeMinAngle)
                    direction = -direction;
            }

            if (direction.sqrMagnitude < 1e-8f)
                direction = Vector3.Cross(axis, Mathf.Abs(axis.y) < 0.99f ? Vector3.up : Vector3.right);

            return bend + direction.normalized * distance;
        }

        private static bool HasDuplicateName(IReadOnlyList<Transform> chain, int index)
        {
            for (int i = 0; i < chain.Count; i++)
            {
                if (i != index && chain[i] != null && chain[i].name == chain[index].name)
                    return true;
            }

            return false;
        }
    }
}
