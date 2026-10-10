using UnityEditor;
using UnityEngine;

namespace OpenIK.Editor
{
    /// <summary>
    /// Shared Inspector for solver components: the solver's settings in declaration order, followed
    /// by the Motion, Performance, and Gizmos sections. Also used for custom
    /// <see cref="OpenIKSolverBase"/> types.
    /// </summary>
    [CustomEditor(typeof(OpenIKSolverBase), true)]
    public class OpenIKSolverEditor : UnityEditor.Editor
    {
        private const string SynchronizeProperty = "synchronizeLimitedJoints";
        private const string StaticConfigurationProperty = "staticSolverConfiguration";
        private const string BoneGizmoModeProperty = "boneGizmoMode";

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            SerializedProperty prop = serializedObject.GetIterator();
            prop.NextVisible(true); // skip script reference
            while (prop.NextVisible(false))
            {
                if (IsDrawnInSection(prop.name))
                    continue;

                EditorGUILayout.PropertyField(prop, GetLabel(prop), true);
            }

            DrawSection("Motion", SynchronizeProperty);

            SerializedProperty staticConfiguration = DrawSection("Performance", StaticConfigurationProperty);
            if (staticConfiguration != null)
                DrawStaticConfigurationNotice(staticConfiguration);

            DrawSection("Gizmos", BoneGizmoModeProperty);

            serializedObject.ApplyModifiedProperties();
        }

        // Label and tooltip for a solver field. Override to give one solver type its own tooltips.
        protected virtual GUIContent GetLabel(SerializedProperty property)
        {
            return new GUIContent(property.displayName, property.tooltip);
        }

        private static bool IsDrawnInSection(string propertyName)
        {
            return propertyName == SynchronizeProperty
                || propertyName == StaticConfigurationProperty
                || propertyName == BoneGizmoModeProperty;
        }

        private SerializedProperty DrawSection(string header, string propertyName)
        {
            SerializedProperty property = serializedObject.FindProperty(propertyName);
            if (property == null)
                return null;

            EditorGUILayout.Space();
            EditorGUILayout.LabelField(header, EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(property, GetLabel(property));
            return property;
        }

        // In Play mode, explains that a static solver configuration ignores later setting changes.
        private static void DrawStaticConfigurationNotice(SerializedProperty staticSolverConfiguration)
        {
            if (!Application.isPlaying)
                return;

            if (!staticSolverConfiguration.boolValue && !staticSolverConfiguration.hasMultipleDifferentValues)
                return;

            EditorGUILayout.HelpBox(
                "Static Solver Configuration is on. Joint constraints, speed limits, and chain settings were read " +
                "when the solver started, so changes made now have no effect. Turn this off to apply them.",
                MessageType.Info);
        }
    }
}
