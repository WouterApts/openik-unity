using UnityEditor;
using UnityEditorInternal;
using UnityEngine;

namespace OpenIK.Editor
{
    // Joint 0 is fixed in the list header. Only the stored joints after it can be edited or reordered.
    internal sealed class MultiFABRIKJointList
    {
        private readonly MultiFABRIKChain _chain;
        private readonly SerializedProperty _joints;
        private bool _expanded = true;
        internal ReorderableList List { get; }

        internal MultiFABRIKJointList(SerializedObject serialized)
        {
            _chain = (MultiFABRIKChain)serialized.targetObject;
            _joints = serialized.FindProperty("joints");
            List = new ReorderableList(serialized, _joints, true, true, true, true)
            {
                headerHeight = EditorGUIUtility.singleLineHeight + 6f,
                elementHeight = EditorGUIUtility.singleLineHeight + 4f,
                drawHeaderCallback = rect =>
                {
                    rect.xMin += 14f; // Align the fixed row with the draggable rows.
                    rect.y += 2f;
                    rect.height = EditorGUIUtility.singleLineHeight;
                    using (new EditorGUI.DisabledScope(true))
                        EditorGUI.ObjectField(rect, new GUIContent("0 (Junction)", MultiFABRIKEditorUtility.JunctionJointTooltip),
                            _chain.transform, typeof(Transform), true);
                },
                drawElementCallback = (rect, index, active, focused) =>
                {
                    rect.y += 2f;
                    rect.height = EditorGUIUtility.singleLineHeight;
                    bool pivot = index == 0 && _chain.Junction == MultiFABRIKJunctionMode.Point;
                    var label = new GUIContent(pivot ? "1 (Pivot)" : (index + 1).ToString(),
                        pivot ? MultiFABRIKEditorUtility.PivotTooltip : "Next joint in root-to-tip order.");
                    EditorGUI.PropertyField(rect, _joints.GetArrayElementAtIndex(index), label);
                },
                drawNoneElementCallback = rect => EditorGUI.LabelField(rect, "Add the next joint with +.", EditorStyles.miniLabel),
                onAddCallback = list =>
                {
                    int index = _joints.arraySize++;
                    _joints.GetArrayElementAtIndex(index).objectReferenceValue = null;
                    list.index = index;
                },
                onRemoveCallback = list =>
                {
                    if (list.index < 0 || list.index >= _joints.arraySize) return;
                    // Deleting a populated object reference otherwise just clears it.
                    _joints.GetArrayElementAtIndex(list.index).objectReferenceValue = null;
                    _joints.DeleteArrayElementAtIndex(list.index);
                    list.index = Mathf.Min(list.index, _joints.arraySize - 1);
                }
            };
        }

        internal void Draw()
        {
            _expanded = EditorGUILayout.Foldout(_expanded,
                new GUIContent($"Joints ({_joints.arraySize + 1})", "The complete chain used by Multi FABRIK, in root-to-tip order. "
                    + "Joint 0 is supplied by this component and stays in place."), true, EditorStyles.foldoutHeader);
            if (!_expanded) return;
            float labelWidth = EditorGUIUtility.labelWidth;
            EditorGUIUtility.labelWidth = 95f;
            try
            {
                List.draggable = List.displayAdd = List.displayRemove = !Application.isPlaying;
                using (new EditorGUI.DisabledScope(Application.isPlaying))
                    List.DoLayoutList();
            }
            finally { EditorGUIUtility.labelWidth = labelWidth; }
        }
    }
}
