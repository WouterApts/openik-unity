using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace OpenIK.Editor
{
    /// <summary>
    /// Shared Inspector for solver components: Execution, the chain, Solving, the solver's own
    /// settings in declaration order, and then the Motion, Performance, and Gizmos sections. Also used
    /// for custom <see cref="OpenIKSolverBase"/> types.
    /// </summary>
    [CustomEditor(typeof(OpenIKSolverBase), true)]
    public class OpenIKSolverEditor : UnityEditor.Editor
    {
        private const string UpdateModeProperty = "updateMode";
        private const string InterpolateProperty = "interpolate";
        private const string ApplyModeProperty = "applyMode";
        private const string TargetProperty = "target";
        private const string JointsProperty = "chainJoints";
        private const string PolesProperty = "poles";
        private const string ToleranceProperty = "tolerance";
        private const string MaxIterationsProperty = "maxIterations";
        private const string SolveFromRestPoseProperty = "solveFromRestPose";
        private const string SynchronizeProperty = "synchronizeLimitedJoints";
        private const string StaticConfigurationProperty = "staticSolverConfiguration";
        private const string BoneGizmoModeProperty = "boneGizmoMode";

        // These are drawn in fixed places, because Unity lists the fields of OpenIKSolverBase before
        // those of SingleChainSolverBase, which would put Max Iterations above the chain.
        private static readonly HashSet<string> PlacedProperties = new()
        {
            UpdateModeProperty, InterpolateProperty, ApplyModeProperty,
            TargetProperty, JointsProperty, PolesProperty,
            ToleranceProperty, MaxIterationsProperty, SolveFromRestPoseProperty,
            SynchronizeProperty, StaticConfigurationProperty, BoneGizmoModeProperty
        };

        private PoleListDrawer _poleList;

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            DrawProperty(UpdateModeProperty);
            DrawInterpolate();
            DrawProperty(ApplyModeProperty);

            DrawChainSection();
            DrawSolvingSection();
            DrawSolverSettings();

            DrawSection("Motion", SynchronizeProperty);

            SerializedProperty staticConfiguration = DrawSection("Performance", StaticConfigurationProperty);
            if (staticConfiguration != null)
                DrawStaticConfigurationNotice(staticConfiguration);

            DrawSection("Gizmos", BoneGizmoModeProperty);

            serializedObject.ApplyModifiedProperties();
        }

        /// <summary>
        /// Draws the chain setup below the Execution settings. The default draws Target, Joints, and the
        /// Poles foldout of a <see cref="SingleChainSolverBase"/>.
        /// </summary>
        protected virtual void DrawChainSection()
        {
            DrawProperty(TargetProperty);

            SerializedProperty joints = serializedObject.FindProperty(JointsProperty);
            if (joints != null)
                DrawJoints(joints);

            SerializedProperty poles = serializedObject.FindProperty(PolesProperty);
            if (poles != null)
            {
                _poleList ??= new PoleListDrawer(serializedObject, poles);
                _poleList.Draw();
            }
        }

        // Label and tooltip for a solver field. Override to give one solver type its own tooltips.
        protected virtual GUIContent GetLabel(SerializedProperty property)
        {
            if (property.name == JointsProperty)
                return new GUIContent("Joints", property.tooltip);

            if (target is FABRIKSolver && property.name == "useSingularityHandling")
                return new GUIContent("Singularity Handling", property.tooltip);

            return new GUIContent(property.displayName, property.tooltip);
        }

        // Tolerance carries the Solving header. Solvers without one get the header on its own.
        private void DrawSolvingSection()
        {
            if (serializedObject.FindProperty(ToleranceProperty) != null)
                DrawProperty(ToleranceProperty);
            else
                DrawHeader("Solving");

            // The two-bone solver places the limb in one step and never iterates.
            if (target is not TwoBoneIKSolver)
                DrawProperty(MaxIterationsProperty);

            DrawProperty(SolveFromRestPoseProperty);
        }

        // The solver type's own settings, in declaration order.
        protected virtual void DrawSolverSettings()
        {
            SerializedProperty prop = serializedObject.GetIterator();
            prop.NextVisible(true); // skip script reference
            while (prop.NextVisible(false))
            {
                if (!PlacedProperties.Contains(prop.name))
                    EditorGUILayout.PropertyField(prop, GetLabel(prop), true);
            }
        }

        private void DrawProperty(string propertyName)
        {
            SerializedProperty property = serializedObject.FindProperty(propertyName);
            if (property != null)
                EditorGUILayout.PropertyField(property, GetLabel(property), true);
        }

        private void DrawJoints(SerializedProperty joints) => DrawJointList(joints, GetLabel(joints));

        // Draws a joint list with a regular-weight header, like the other fields around it.
        internal static void DrawJointList(SerializedProperty joints, GUIContent label)
        {
            // Unity's built-in list drawer uses this shared style for its header.
            GUIStyle headerStyle = EditorStyles.foldoutHeader;
            FontStyle previousFontStyle = headerStyle.fontStyle;
            try
            {
                headerStyle.fontStyle = FontStyle.Normal;
                EditorGUILayout.PropertyField(joints, label, true);
            }
            finally
            {
                headerStyle.fontStyle = previousFontStyle;
            }
        }

        // Interpolate only matters in FixedUpdate mode, so it is drawn indented below Update Mode.
        private void DrawInterpolate()
        {
            SerializedProperty interpolate = serializedObject.FindProperty(InterpolateProperty);
            SerializedProperty updateMode = serializedObject.FindProperty(UpdateModeProperty);
            if (interpolate == null || updateMode == null)
                return;

            if (!updateMode.hasMultipleDifferentValues && updateMode.intValue != (int)UpdateMode.FixedUpdate)
                return;

            EditorGUI.indentLevel++;
            EditorGUILayout.PropertyField(interpolate, GetLabel(interpolate));
            EditorGUI.indentLevel--;
        }

        // Matches the look of a [Header] attribute.
        private static void DrawHeader(string header)
        {
            EditorGUILayout.Space(EditorGUIUtility.singleLineHeight * 0.5f);
            EditorGUILayout.LabelField(header, EditorStyles.boldLabel);
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
                "Static Solver Configuration is on. Joint constraints, speed limits, and pole sections were read " +
                "when the solver started. Turn this off to apply changes to them.",
                MessageType.Info);
        }
    }
}
