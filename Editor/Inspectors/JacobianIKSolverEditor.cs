using UnityEditor;
using UnityEngine;

namespace OpenIK.Editor
{
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
                case "mode":
                    return "Solve And Apply moves the joints toward the solved pose, respecting joint speed limits. Solve Only calculates the pose for use by scripts without moving the joints automatically.";
                case "target":
                    return "Transform the end effector moves toward.";
                case "tolerance":
                    return "Stop iterating when the combined position and weighted orientation error falls below this value.";
                case "maxIterations":
                    return "Maximum solver iterations per frame. Higher values allow more refinement but cost more processing time. Stops early when Tolerance is reached.";
                case "chainJoints":
                    return "Joint transforms ordered from the root to the end effector.";
                case "staticSolverConfiguration":
                    return "Skip per-frame joint configuration updates when constraints and speed limits stay unchanged.";
                case "damping":
                    return "Stabilizes solver steps near singularities. Higher values can slow convergence.";
                case "stepSize":
                    return "Scales each iteration's joint rotation and slider movement.";
                case "orientationMode":
                    return GetOrientationModeInfo(property);
                case "orientationWeight":
                    return "Controls the importance of matching orientation relative to position. Higher values favour orientation and also affect the tolerance check. Zero removes the orientation objective; unused when Orientation Mode is None.";
                case "boneGizmoMode":
                    return "Choose when to draw the chain's joints and bone connections.";
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
