using UnityEditor;

namespace OpenIK.Editor
{
    [CustomEditor(typeof(CCDIKSolver))]
    public class CCDIKSolverEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            SerializedProperty prop = serializedObject.GetIterator();
            prop.NextVisible(true);
            while (prop.NextVisible(false))
            {
                if (prop.name == "boneGizmoMode")
                    continue;

                EditorGUILayout.PropertyField(prop, true);
            }

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Gizmos", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("boneGizmoMode"));

            serializedObject.ApplyModifiedProperties();
        }
    }
}
