using UnityEditor;
using UnityEngine;

namespace OpenIK.Editor
{
    // Adds Jacobian-specific tooltips: Tolerance also covers orientation error, and the Orientation
    // Mode tooltip describes the selected mode.
    [CustomEditor(typeof(JacobianIKSolver))]
    public class JacobianIKSolverEditor : OpenIKSolverEditor
    {
        protected override GUIContent GetLabel(SerializedProperty property)
        {
            return new GUIContent(property.displayName, GetTooltip(property));
        }

        private static string GetTooltip(SerializedProperty property)
        {
            switch (property.name)
            {
                case "tolerance":
                    return "Stop iterating when the combined position and weighted orientation error falls below this value.";
                case "orientationMode":
                    return GetOrientationModeInfo(property);
                default:
                    return property.tooltip;
            }
        }

        private static string GetOrientationModeInfo(SerializedProperty property)
        {
            if (property.enumValueIndex < 0 || property.enumValueIndex >= property.enumNames.Length)
                return string.Empty;

            switch (property.enumNames[property.enumValueIndex])
            {
                case "FullRotation":
                    return "Match the final bone's rotation to the target, including twist.";
                case "BoneDirection":
                    return "Align the direction from the second-last joint to the end effector with the target's forward axis (+Z), without matching twist.";
                case "None":
                    return "Solve for end-effector position only. Ignore target rotation.";
                default:
                    return string.Empty;
            }
        }
    }
}
