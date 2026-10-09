using System;
using UnityEditor;
using UnityEngine;

namespace OpenIK.Editor
{
    [InitializeOnLoad]
    internal static class OpenIKComponentIconUtility
    {
        private const string SessionStateKey = "OpenIK.ComponentIconsApplied";

        private static readonly IconBinding[] IconBindings =
        {
            new("Runtime/Constraints/BallSocketIKJoint.cs", "Editor/Icons/Ball IKJoint Logo.png"),
            new("Runtime/Solvers/CCDIKSolver.cs", "Editor/Icons/CCD Solver Logo.png"),
            new("Runtime/Solvers/FABRIKSolver.cs", "Editor/Icons/FABRIK Solver Logo.png"),
            new("Runtime/Constraints/HingeIKJoint.cs", "Editor/Icons/Hinge IKJoint Logo.png"),
            new("Runtime/Constraints/SliderIKJoint.cs", "Editor/Icons/Prismatic IKJoint Logo.png"),
            new("Runtime/Solvers/JacobianIKSolver.cs", "Editor/Icons/Jacobian Solver Logo.png")
        };

        static OpenIKComponentIconUtility()
        {
            EditorApplication.delayCall += ApplyIconsOnLoad;
        }

        [MenuItem("Tools/OpenIK/Reapply Component Icons")]
        private static void ReapplyIconsMenu()
        {
            SessionState.SetBool(SessionStateKey, false);
            ApplyIcons(force: true);
        }

        private static void ApplyIconsOnLoad()
        {
            if (SessionState.GetBool(SessionStateKey, false))
                return;

            ApplyIcons(force: false);
        }

        private static void ApplyIcons(bool force)
        {
            bool changedAnyIcon = false;

            foreach (IconBinding binding in IconBindings)
            {
                if (!TryLoadPackageAsset(binding.ScriptRelativePath, out MonoScript script, out string scriptAssetPath))
                {
                    Debug.LogWarning($"[OpenIK] Could not find script asset '{binding.ScriptRelativePath}' while applying component icons.");
                    continue;
                }

                if (!TryLoadPackageAsset(binding.IconRelativePath, out Texture2D icon, out _))
                {
                    Debug.LogWarning($"[OpenIK] Could not find icon asset '{binding.IconRelativePath}' while applying component icons.");
                    continue;
                }

                if (!force && HasAssignedIcon(script, icon))
                    continue;

                SetScriptIcon(script, icon);
                AssetDatabase.ImportAsset(scriptAssetPath, ImportAssetOptions.ForceUpdate);
                changedAnyIcon = true;
            }

            if (changedAnyIcon)
                AssetDatabase.SaveAssets();

            SessionState.SetBool(SessionStateKey, true);
        }

        private static bool TryLoadPackageAsset<T>(string relativePath, out T asset, out string assetPath)
            where T : UnityEngine.Object
        {
            string root = PackageRoot;
            if (root != null)
            {
                assetPath = $"{root}/{relativePath}";
                asset = AssetDatabase.LoadAssetAtPath<T>(assetPath);
                return asset != null;
            }

            asset = null;
            assetPath = null;
            return false;
        }

        // Resolves the package's asset root at runtime so icons load regardless of how the package is
        // installed (embedded, local file:, Git, or registry) or what the package folder is named.
        private static string _packageRoot;
        private static bool _packageRootResolved;

        private static string PackageRoot
        {
            get
            {
                if (_packageRootResolved)
                    return _packageRoot;

                _packageRootResolved = true;
                UnityEditor.PackageManager.PackageInfo package =
                    UnityEditor.PackageManager.PackageInfo.FindForAssembly(typeof(OpenIKComponentIconUtility).Assembly);
                _packageRoot = package?.assetPath;

                if (_packageRoot == null)
                    Debug.LogWarning("[OpenIK] Could not resolve the OpenIK package root; component icons will not be applied.");

                return _packageRoot;
            }
        }

        private static bool HasAssignedIcon(MonoScript script, Texture2D expectedIcon)
        {
            Texture2D assignedIcon = GetAssignedIcon(script);
            return assignedIcon == expectedIcon;
        }

        private static Texture2D GetAssignedIcon(MonoScript script)
        {
            Type monoImporterType = typeof(MonoImporter);
            var staticGetIcon = monoImporterType.GetMethod(
                "GetIcon",
                System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic,
                null,
                new[] { typeof(MonoScript) },
                null);

            if (staticGetIcon != null)
                return staticGetIcon.Invoke(null, new object[] { script }) as Texture2D;

            string scriptPath = AssetDatabase.GetAssetPath(script);
            MonoImporter importer = AssetImporter.GetAtPath(scriptPath) as MonoImporter;
            if (importer == null)
                return null;

            var instanceGetIcon = monoImporterType.GetMethod(
                "GetIcon",
                System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic,
                null,
                Type.EmptyTypes,
                null);

            return instanceGetIcon?.Invoke(importer, null) as Texture2D;
        }

        private static void SetScriptIcon(MonoScript script, Texture2D icon)
        {
            Type monoImporterType = typeof(MonoImporter);
            var staticSetIcon = monoImporterType.GetMethod(
                "SetIcon",
                System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic,
                null,
                new[] { typeof(MonoScript), typeof(Texture2D) },
                null);

            if (staticSetIcon != null)
            {
                staticSetIcon.Invoke(null, new object[] { script, icon });
                return;
            }

            string scriptPath = AssetDatabase.GetAssetPath(script);
            MonoImporter importer = AssetImporter.GetAtPath(scriptPath) as MonoImporter;
            if (importer == null)
                throw new InvalidOperationException($"[OpenIK] Could not get MonoImporter for '{scriptPath}'.");

            var instanceSetIcon = monoImporterType.GetMethod(
                "SetIcon",
                System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic,
                null,
                new[] { typeof(Texture2D) },
                null);

            if (instanceSetIcon == null)
                throw new MissingMethodException("UnityEditor.MonoImporter", "SetIcon");

            instanceSetIcon.Invoke(importer, new object[] { icon });
        }

        private readonly struct IconBinding
        {
            public IconBinding(string scriptRelativePath, string iconRelativePath)
            {
                ScriptRelativePath = scriptRelativePath;
                IconRelativePath = iconRelativePath;
            }

            public string ScriptRelativePath { get; }
            public string IconRelativePath { get; }
        }
    }
}
