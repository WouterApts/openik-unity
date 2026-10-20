using System;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace OpenIK.Editor
{
    [InitializeOnLoad]
    internal static class OpenIKSampleSceneView
    {
        private const string MenuPath = "Tools/OpenIK/Frame Sample Scene";

        static OpenIKSampleSceneView()
        {
            EditorSceneManager.sceneOpened += OnSceneOpened;
        }

        private static void OnSceneOpened(Scene scene, OpenSceneMode mode)
        {
            if (mode != OpenSceneMode.Single || !IsSampleScene(scene) ||
                EditorApplication.isPlayingOrWillChangePlaymode)
                return;

            // Wait until the scene has opened, including when the editor is unfocused.
            EditorApplication.update += FrameOpenedScene;
            void FrameOpenedScene()
            {
                EditorApplication.update -= FrameOpenedScene;
                if (scene.IsValid() && scene.isLoaded &&
                    SceneManager.GetActiveScene() == scene &&
                    !EditorApplication.isPlayingOrWillChangePlaymode)
                    FrameScene(scene);
            }
        }

        [MenuItem(MenuPath)]
        private static void FrameActiveScene()
        {
            FrameScene(SceneManager.GetActiveScene());
        }

        [MenuItem(MenuPath, true)]
        private static bool CanFrameActiveScene()
        {
            return !EditorApplication.isPlayingOrWillChangePlaymode &&
                   IsSampleScene(SceneManager.GetActiveScene());
        }

        private static bool IsSampleScene(Scene scene)
        {
            return scene.IsValid() &&
                   scene.path.StartsWith("Assets/Samples/OpenIK/", StringComparison.Ordinal) &&
                   TryGetView(scene.name, out _);
        }

        private static bool TryGetView(string sceneName, out Vector3 angles)
        {
            switch (sceneName)
            {
                case "Basic Chains":
                case "Multi FABRIK Chains":
                    angles = new Vector3(8f, 0f, 0f);
                    return true;
                case "Robotic Painter":
                    angles = new Vector3(20f, 25f, 0f);
                    return true;
                case "IK Spider":
                    angles = new Vector3(25f, 130f, 0f);
                    return true;
                case "Humanoid Walker":
                case "Humanoid Walker - Valley":
                    angles = new Vector3(15f, 150f, 0f);
                    return true;
                default:
                    angles = default;
                    return false;
            }
        }

        private static bool IncludeRoot(string sceneName, GameObject root)
        {
            switch (sceneName)
            {
                case "Humanoid Walker":
                case "Humanoid Walker - Valley":
                    return root.name == "Mecanim Robot";
                case "IK Spider":
                    return root.name == "Spider Bot" ||
                           root.name.StartsWith("Tip Target ", StringComparison.Ordinal);
                case "Robotic Painter":
                    return root.name != "Dark Studio Floor";
                default:
                    return true;
            }
        }

        private static void FrameScene(Scene scene)
        {
            if (!IsSampleScene(scene) || !TryGetView(scene.name, out Vector3 angles))
                return;

            Bounds bounds = default;
            bool hasBounds = false;
            foreach (GameObject root in scene.GetRootGameObjects())
            {
                if (!IncludeRoot(scene.name, root))
                    continue;

                foreach (Renderer renderer in root.GetComponentsInChildren<Renderer>())
                {
                    if (!renderer.enabled || !renderer.gameObject.activeInHierarchy)
                        continue;

                    if (hasBounds)
                        bounds.Encapsulate(renderer.bounds);
                    else
                    {
                        bounds = renderer.bounds;
                        hasBounds = true;
                    }
                }
            }

            if (!hasBounds)
                return;

            SceneView view = SceneView.lastActiveSceneView;
            if (view == null)
                view = EditorWindow.GetWindow<SceneView>();

            bounds.Expand(bounds.size * 0.15f);
            view.in2DMode = false;
            view.LookAt(bounds.center, Quaternion.Euler(angles), view.size, false, true);
            view.Frame(bounds, true);
            // Fit the viewport rather than a bounding sphere, which leaves wide chain rows tiny.
            float verticalSlope = Mathf.Tan(view.camera.fieldOfView * Mathf.Deg2Rad * 0.5f);
            float horizontalSlope = verticalSlope * view.camera.aspect;
            Quaternion inverseRotation = Quaternion.Inverse(view.rotation);
            float distance = 0.1f;
            for (int corner = 0; corner < 8; corner++)
            {
                Vector3 offset = Vector3.Scale(bounds.extents, new Vector3(
                    (corner & 1) == 0 ? -1f : 1f,
                    (corner & 2) == 0 ? -1f : 1f,
                    (corner & 4) == 0 ? -1f : 1f));
                Vector3 local = inverseRotation * offset;
                distance = Mathf.Max(distance,
                    Mathf.Abs(local.x) / horizontalSlope - local.z,
                    Mathf.Abs(local.y) / verticalSlope - local.z);
            }

            float size = distance * view.size / view.cameraDistance;
            view.LookAt(bounds.center, view.rotation, Mathf.Max(size, 0.1f), false, true);
            view.Repaint();
        }
    }
}
