using UnityEditor;
using UnityEngine;

namespace OpenIK.Editor
{
    /// <summary>
    /// Shared Inspector for solver components: the solver's own settings, a Motion section with
    /// Synchronize Limited Joints, and the Gizmos section. Also used for custom
    /// <see cref="OpenIKSolverBase"/> types.
    /// </summary>
    [CustomEditor(typeof(OpenIKSolverBase), true)]
    public class OpenIKSolverEditor : UnityEditor.Editor
    {
        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            SerializedProperty prop = serializedObject.GetIterator();
            prop.NextVisible(true); // skip script reference
            while (prop.NextVisible(false))
            {
                if (IsDrawnSeparately(prop.name))
                    continue;

                EditorGUILayout.PropertyField(prop, GetLabel(prop), true);
                if (prop.name == "staticSolverConfiguration")
                    DrawStaticConfigurationNotice(prop);
            }

            DrawMotionSection();

            SerializedProperty boneGizmoMode = serializedObject.FindProperty("boneGizmoMode");
            if (boneGizmoMode != null)
            {
                EditorGUILayout.Space();
                EditorGUILayout.LabelField("Gizmos", EditorStyles.boldLabel);
                EditorGUILayout.PropertyField(boneGizmoMode, GetLabel(boneGizmoMode));
            }

            serializedObject.ApplyModifiedProperties();
        }

        /// Label and tooltip for a solver field. Override to give one solver type its own tooltips.
        protected virtual GUIContent GetLabel(SerializedProperty property)
        {
            return new GUIContent(property.displayName, property.tooltip);
        }

        private static bool IsDrawnSeparately(string propertyName)
        {
            return propertyName == "boneGizmoMode" || propertyName == "synchronizeLimitedJoints";
        }

        /// In Play mode, explains that a static solver configuration ignores later setting changes.
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

        /// Draws Synchronize Limited Joints under a Motion header, matching the joints' Motion section.
        /// It only drives Solve And Apply; in Solve Only, scripts pass the choice to ApplyLastOutput.
        private void DrawMotionSection()
        {
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Motion", EditorStyles.boldLabel);

            SerializedProperty mode = serializedObject.FindProperty("mode");
            SerializedProperty synchronize = serializedObject.FindProperty("synchronizeLimitedJoints");
            bool solveAndApply = mode.hasMultipleDifferentValues || mode.intValue == (int)SolveMode.SolveAndApply;
            using (new EditorGUI.DisabledScope(!solveAndApply))
                EditorGUILayout.PropertyField(synchronize, GetLabel(synchronize));
        }
    }
}
