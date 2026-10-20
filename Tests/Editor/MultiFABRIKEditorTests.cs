using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;

namespace OpenIK.Editor.Tests
{
    public partial class MultiFABRIKEditorTests
    {
        private GameObject _rig;
        private MultiFABRIKSolver _solver;
        private Transform _root, _hub, _tip;
        private UnityEngine.SceneManagement.Scene _testScene;
        private int _undoGroup;

        [SetUp]
        public void SetUp()
        {
            _testScene = UnityEditor.SceneManagement.EditorSceneManager.NewPreviewScene();
            Undo.IncrementCurrentGroup();
            _undoGroup = Undo.GetCurrentGroup();
            _rig = new GameObject("Editor rig");
            UnityEngine.SceneManagement.SceneManager.MoveGameObjectToScene(_rig, _testScene);
            _rig.SetActive(false);
            _solver = _rig.AddComponent<MultiFABRIKSolver>();
            _root = Joint("Root", _rig.transform);
            _hub = Joint("Hub", _root);
            _tip = Joint("Tip", _hub);
        }

        [TearDown]
        public void TearDown()
        {
            // Reverting component Undo can recreate an object after its rig has been destroyed.
            // Keep all test objects and Undo records inside this fixture's scene and group.
            Undo.RevertAllDownToGroup(_undoGroup);
            Object.DestroyImmediate(_rig);
            UnityEditor.SceneManagement.EditorSceneManager.ClosePreviewScene(_testScene);
        }

        private static Transform Joint(string name, Transform parent)
        {
            Transform joint = new GameObject(name).transform;
            joint.SetParent(parent, false);
            joint.localPosition = Vector3.up;
            return joint;
        }

        [Test]
        public void RootPullOnlyGraysOutWhenPositionAndRotationArePinned()
        {
            _solver.AddChain("Trunk", new[] { _root, _hub, _tip }, _tip);
            Assert.That(MultiFABRIKSolverEditor.PullDisabledReason(_solver.Chains, 0, 1f), Is.Null);
            Transform sibling = Joint("Sibling", _root);
            _solver.AddChain("Root branch", new[] { _root, sibling }, sibling);
            Assert.That(MultiFABRIKSolverEditor.PullDisabledReason(_solver.Chains, 0, 1f), Is.Not.Null);
            Assert.That(MultiFABRIKSolverEditor.PullDisabledReason(_solver.Chains, 1, 1f), Is.Not.Null);
            Assert.That(MultiFABRIKSolverEditor.PullDisabledReason(_solver.Chains, 1, 0.5f), Is.Null);
        }

        [Test]
        public void PinAndWarningFollowOwnedBranchesAndTargetedTips()
        {
            var trunk = _solver.AddChain("Trunk", new[] { _root, _hub }, _hub);
            _solver.AddChain("Arm", new[] { _hub, _tip }, _tip);
            Assert.That(MultiFABRIKSolverEditor.HasOwnedBranches(_solver.Chains, 0), Is.True);
            Assert.That(MultiFABRIKSolverEditor.HasOwnedBranches(_solver.Chains, 1), Is.False);
            Assert.That(MultiFABRIKSolverEditor.HasUnpinnedTarget(_solver.Chains, 0, _solver.GetPin(trunk)), Is.True);
            _solver.SetPin(trunk, 0.1f);
            Assert.That(MultiFABRIKSolverEditor.HasUnpinnedTarget(_solver.Chains, 0, _solver.GetPin(trunk)), Is.False);
            _solver.SetPin(trunk, 0f);
            trunk.Target = null;
            Assert.That(MultiFABRIKSolverEditor.HasUnpinnedTarget(_solver.Chains, 0, _solver.GetPin(trunk)), Is.False);
        }

        [Test]
        public void MultiSolverSelectsItsCustomInspector()
        {
            UnityEditor.Editor inspector = UnityEditor.Editor.CreateEditor(_solver);
            try { Assert.That(inspector, Is.TypeOf<MultiFABRIKSolverEditor>()); }
            finally { Object.DestroyImmediate(inspector); }
        }

        [UnityTest]
        public System.Collections.IEnumerator SolverInspectorRenders()
        {
            _solver.AddChain("Trunk", new[] { _root, _hub }, _hub);
            Transform elbow = Joint("Elbow", _tip);
            Transform hand = Joint("Hand", elbow);
            _solver.AddChain("Arm", new[] { _hub, _tip, elbow, hand }, hand).AddPole(elbow, _root);
            yield return Render(_solver);
        }

        // Draws an inspector for the object in a window until it has painted once, and fails on exceptions.
        private static System.Collections.IEnumerator Render(Object target)
        {
            UnityEditor.Editor inspector = UnityEditor.Editor.CreateEditor(target);
            InspectorHost host = ScriptableObject.CreateInstance<InspectorHost>();
            try
            {
                host.Inspector = inspector;
                host.position = new Rect(100f, 100f, 450f, 800f);
                host.ShowUtility();
                for (int i = 0; i < 10 && !host.Painted; i++) { host.Repaint(); yield return null; }
                Assert.That(host.Failure, Is.Null);
                Assert.That(host.Painted, Is.True);
            }
            finally
            {
                host.Close();
                Object.DestroyImmediate(inspector);
            }
        }

        private sealed class InspectorHost : EditorWindow
        {
            internal UnityEditor.Editor Inspector;
            internal System.Exception Failure;
            internal bool Painted;
            private Vector2 _scroll;

            private void OnGUI()
            {
                if (Inspector == null) return;
                try
                {
                    _scroll = EditorGUILayout.BeginScrollView(_scroll);
                    Inspector.OnInspectorGUI();
                    EditorGUILayout.EndScrollView();
                    if (Event.current.type == EventType.Repaint) Painted = true;
                }
                catch (System.Exception exception) { Failure = exception; }
            }
        }
    }
}
