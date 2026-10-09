using UnityEditor;
using UnityEngine;

namespace OpenIK.Editor
{
    /// <summary>
    /// Shared Inspector for joint components: the joint's own settings, a Motion section showing
    /// only the speed limits its type supports, and the Gizmos section. Also used for custom
    /// <see cref="ConstrainedJoint"/> types.
    /// </summary>
    [CustomEditor(typeof(ConstrainedJoint), true)]
    public class ConstrainedJointEditor : UnityEditor.Editor
    {
        private static readonly GUIContent MaxAngularSpeedLabel = new(
            "Max Angular Speed (°/s)",
            "Maximum rotation speed relative to the IK parent, in degrees per second. Zero holds the joint still; its parent can still carry it.");

        private static readonly GUIContent MaxLinearSpeedLabel = new(
            "Max Linear Speed (m/s)",
            "Maximum travel speed along the slide axis, in metres per second. Zero holds the slide still; its parent can still carry it.");

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            SerializedProperty prop = serializedObject.GetIterator();
            prop.NextVisible(true); // skip script reference
            while (prop.NextVisible(false))
            {
                if (IsDrawnSeparately(prop.name))
                    continue;
                EditorGUILayout.PropertyField(prop, true);
            }

            EditorGUILayout.Space();
            DrawMotionSection();

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Gizmos", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("gizmoMode"));

            serializedObject.ApplyModifiedProperties();
        }

        private static bool IsDrawnSeparately(string propertyName)
        {
            return propertyName == "gizmoMode" ||
                   propertyName == nameof(ConstrainedJoint.limitSpeed) ||
                   propertyName == nameof(ConstrainedJoint.maxAngularSpeed) ||
                   propertyName == nameof(ConstrainedJoint.maxLinearSpeed);
        }

        private void DrawMotionSection()
        {
            EditorGUILayout.LabelField("Motion", EditorStyles.boldLabel);

            var joint = (ConstrainedJoint)target;
            JointMotionSupport support = joint.MotionSupport;
            SerializedProperty limitSpeed = serializedObject.FindProperty(nameof(ConstrainedJoint.limitSpeed));

            if (support == JointMotionSupport.None)
            {
                // Keep the toggle reachable so an unsupported setting can be switched off.
                if (limitSpeed.boolValue || limitSpeed.hasMultipleDifferentValues)
                {
                    EditorGUILayout.PropertyField(limitSpeed);
                    EditorGUILayout.HelpBox(
                        "This joint type does not support speed limits, so Limit Speed has no effect. " +
                        "Its runtime constraints must implement IAngularMotionProvider or ISegmentMotionProvider.",
                        MessageType.Warning);
                }
                else
                {
                    EditorGUILayout.HelpBox("This joint type does not support speed limits.", MessageType.None);
                }

                return;
            }

            EditorGUILayout.PropertyField(limitSpeed);
            using (new EditorGUI.DisabledScope(!limitSpeed.boolValue && !limitSpeed.hasMultipleDifferentValues))
            {
                if ((support & JointMotionSupport.Angular) != 0)
                    EditorGUILayout.PropertyField(serializedObject.FindProperty(nameof(ConstrainedJoint.maxAngularSpeed)), MaxAngularSpeedLabel);
                if ((support & JointMotionSupport.Linear) != 0)
                    EditorGUILayout.PropertyField(serializedObject.FindProperty(nameof(ConstrainedJoint.maxLinearSpeed)), MaxLinearSpeedLabel);
            }

            if (limitSpeed.boolValue && !serializedObject.FindProperty(nameof(ConstrainedJoint.jointIsEnabled)).boolValue)
            {
                EditorGUILayout.HelpBox(
                    "Joint Is Enabled is off, so this joint's constraints and speed limit are not used.",
                    MessageType.Info);
            }
        }
    }
}
