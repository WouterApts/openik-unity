using UnityEditor;
using UnityEngine;

namespace OpenIK.Editor
{
    /// <summary>Inspector for a three-joint limb with one optional pole target.</summary>
    [CustomEditor(typeof(TwoBoneIKSolver))]
    public class TwoBoneIKSolverEditor : OpenIKSolverEditor
    {
        private PoleListDrawer _poleDrawer;

        protected override void DrawChainSection()
        {
            EditorGUILayout.PropertyField(serializedObject.FindProperty("target"));

            SerializedProperty joints = serializedObject.FindProperty("chainJoints");
            EditorGUILayout.LabelField("Joints");
            using (new EditorGUI.IndentLevelScope())
            {
                DrawJoint(joints, 0, "Root");
                DrawJoint(joints, 1, "Middle");
                DrawJoint(joints, 2, "End");
            }

            SerializedProperty poles = serializedObject.FindProperty("poles");
            Transform middle = joints.arraySize > 1
                ? joints.GetArrayElementAtIndex(1).objectReferenceValue as Transform
                : null;
            DrawPole(poles, middle);
        }

        private static void DrawJoint(SerializedProperty joints, int index, string label)
        {
            var content = new GUIContent(label, joints.tooltip);
            if (index < joints.arraySize)
            {
                EditorGUILayout.PropertyField(joints.GetArrayElementAtIndex(index), content);
                return;
            }

            EditorGUI.BeginChangeCheck();
            var joint = EditorGUILayout.ObjectField(content, null, typeof(Transform), true);
            if (!EditorGUI.EndChangeCheck())
                return;

            int previousSize = joints.arraySize;
            joints.arraySize = 3;
            for (int i = previousSize; i < joints.arraySize; i++)
                joints.GetArrayElementAtIndex(i).objectReferenceValue = null;
            joints.GetArrayElementAtIndex(index).objectReferenceValue = joint;
        }

        private void DrawPole(SerializedProperty poles, Transform middle)
        {
            SerializedProperty pole = poles.arraySize > 0 ? poles.GetArrayElementAtIndex(0) : null;
            SerializedProperty poleTarget = pole?.FindPropertyRelative("target");
            SerializedProperty weight = pole?.FindPropertyRelative("weight");
            if (pole != null)
            {
                SerializedProperty bendJoint = pole.FindPropertyRelative("bendJoint");
                if (bendJoint.objectReferenceValue != middle)
                    bendJoint.objectReferenceValue = middle;
            }

            var targetLabel = new GUIContent("Pole Target",
                "The middle joint bends toward this transform. Leave empty to keep the limb's current bend direction.");
            var weightLabel = new GUIContent("Weight", "How far the middle joint turns toward the pole. Zero turns the pole off.");
            Object newTarget = null;
            float newWeight = 1f;
            EditorGUI.BeginChangeCheck();
            using (new EditorGUILayout.HorizontalScope())
            {
                if (poleTarget != null)
                    EditorGUILayout.PropertyField(poleTarget, targetLabel);
                else
                    newTarget = EditorGUILayout.ObjectField(targetLabel, null, typeof(Transform), true);

                if (poleTarget == null || poleTarget.objectReferenceValue == null)
                {
                    using (new EditorGUI.DisabledScope(middle == null))
                    {
                        if (GUILayout.Button(new GUIContent("Create", "Create a target on the side the limb bends toward."), GUILayout.Width(56f)))
                        {
                            serializedObject.ApplyModifiedProperties();
                            _poleDrawer ??= new PoleListDrawer(serializedObject, poles);
                            newTarget = _poleDrawer.CreatePoleTarget(middle);
                            if (poleTarget != null)
                                poleTarget.objectReferenceValue = newTarget;
                            GUI.changed = true;
                        }
                    }
                }
            }

            using (new EditorGUI.IndentLevelScope())
            {
                if (weight != null)
                    EditorGUILayout.PropertyField(weight, weightLabel);
                else
                    newWeight = EditorGUILayout.Slider(weightLabel, newWeight, 0f, 1f);
            }

            if (EditorGUI.EndChangeCheck() && pole == null)
            {
                poles.arraySize = 1;
                pole = poles.GetArrayElementAtIndex(0);
                pole.FindPropertyRelative("bendJoint").objectReferenceValue = middle;
                pole.FindPropertyRelative("target").objectReferenceValue = newTarget;
                pole.FindPropertyRelative("weight").floatValue = newWeight;
            }
        }
    }
}
