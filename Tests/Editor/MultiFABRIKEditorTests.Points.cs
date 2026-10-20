using System.Collections.Generic;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;

namespace OpenIK.Editor.Tests
{
    public partial class MultiFABRIKEditorTests
    {
        private Transform PointSetupBranches()
        {
            Transform other = Joint("Other", _hub);
            other.localPosition = Vector3.right;
            _solver.AddChain("Trunk", new[] { _root, _hub });
            var arm = _solver.AddChain("Arm", new[] { _hub, _tip }, _tip, 0.3f, 0.6f);
            arm.Tolerance = 0.02f;
            arm.UseSingularityHandling = false;
            _solver.AddChain("Other", new[] { _hub, other }, other);
            return other;
        }

        [Test]
        public void ChangingJunctionModePreservesHierarchyAndChainSettings()
        {
            Transform other = PointSetupBranches();
            var arm = _solver.Chains[1];
            Vector3 tipPosition = _tip.position, otherPosition = other.position;
            Quaternion hubRotation = _hub.rotation;
            int transforms = _rig.GetComponentsInChildren<Transform>(true).Length;
            Undo.IncrementCurrentGroup();

            foreach (MultiFABRIKJunctionMode mode in new[] { MultiFABRIKJunctionMode.Point, MultiFABRIKJunctionMode.Rigid })
            {
                MultiFABRIKEditorUtility.SetJunction(_hub, mode);
                Assert.That(_solver.Chains[1].Junction, Is.EqualTo(mode));
                Assert.That(_solver.Chains[2].Junction, Is.EqualTo(mode));
                Assert.That(_rig.GetComponentsInChildren<Transform>(true), Has.Length.EqualTo(transforms));
                Assert.That(_tip.parent, Is.SameAs(_hub));
                Assert.That(other.parent, Is.SameAs(_hub));
                Assert.That(_tip.position, Is.EqualTo(tipPosition));
                Assert.That(other.position, Is.EqualTo(otherPosition));
                Assert.That(_hub.rotation, Is.EqualTo(hubRotation));
                Assert.That(arm.Joints, Is.EqualTo(new[] { _hub, _tip }));
                Assert.That(arm.Target, Is.SameAs(_tip));
                Assert.That(_solver.GetPull(arm), Is.EqualTo(0.3f));
                Assert.That(_solver.GetPin(arm), Is.EqualTo(0.6f));
                Assert.That(arm.Tolerance, Is.EqualTo(0.02f));
                Assert.That(arm.UseSingularityHandling, Is.False);
            }
        }

        [TestCase(MultiFABRIKJunctionMode.Rigid)]
        [TestCase(MultiFABRIKJunctionMode.Point)]
        public void JointListEditsMatchRuntimeOrderAndKeepJointZero(MultiFABRIKJunctionMode mode)
        {
            var arm = _solver.AddChain("Arm", new[] { _root, _hub, _tip }, _tip);
            arm.Junction = mode;
            using var serialized = new SerializedObject(arm);
            var drawer = new MultiFABRIKJointList(serialized);
            var tail = drawer.List.serializedProperty;

            // Extending the list must leave an empty slot, not silently duplicate the tip.
            Undo.IncrementCurrentGroup();
            drawer.List.onAddCallback(drawer.List);
            serialized.ApplyModifiedProperties();
            Assert.That(arm.Joints, Is.EqualTo(new Transform[] { _root, _hub, _tip, null }));
            Assert.That(arm.transform, Is.SameAs(_root));

            tail.GetArrayElementAtIndex(2).objectReferenceValue = Joint("End", _tip);
            serialized.ApplyModifiedProperties();
            Transform end = arm.Joints[3];
            Undo.IncrementCurrentGroup();
            tail.MoveArrayElement(2, 0);
            serialized.ApplyModifiedProperties();
            Assert.That(arm.Joints, Is.EqualTo(new[] { _root, end, _hub, _tip }));
            Undo.PerformUndo();
            serialized.Update();
            Assert.That(arm.Joints, Is.EqualTo(new[] { _root, _hub, _tip, end }));
            Undo.PerformRedo();
            serialized.Update();
            Assert.That(arm.Joints, Is.EqualTo(new[] { _root, end, _hub, _tip }));

            Undo.IncrementCurrentGroup();
            drawer.List.index = 0; // First editable row is runtime joint 1.
            drawer.List.onRemoveCallback(drawer.List);
            serialized.ApplyModifiedProperties();
            Assert.That(arm.Joints, Is.EqualTo(new[] { _root, _hub, _tip }));
            Undo.PerformUndo();
            serialized.Update();
            Assert.That(arm.Joints, Is.EqualTo(new[] { _root, end, _hub, _tip }));

            while (tail.arraySize > 0)
            {
                drawer.List.index = 0;
                drawer.List.onRemoveCallback(drawer.List);
            }
            serialized.ApplyModifiedProperties();
            Assert.That(arm.Joints, Is.EqualTo(new[] { _root }));
        }

        [Test]
        public void JointListEditsOnPrefabInstancesPreserveTheImplicitStart()
        {
            PointSetupBranches();
            string path = AssetDatabase.GenerateUniqueAssetPath("Assets/OpenIKJointListTest.prefab");
            GameObject instance = null;
            try
            {
                var prefab = PrefabUtility.SaveAsPrefabAsset(_rig, path);
                instance = (GameObject)PrefabUtility.InstantiatePrefab(prefab, _rig.scene);
                var arm = instance.GetComponent<MultiFABRIKSolver>().Chains[1];
                Transform start = arm.transform, tip = arm.Joints[1];
                using var serialized = new SerializedObject(arm);
                var drawer = new MultiFABRIKJointList(serialized);
                Undo.IncrementCurrentGroup();
                drawer.List.onAddCallback(drawer.List);
                serialized.ApplyModifiedProperties();
                Assert.That(arm.Joints, Is.EqualTo(new Transform[] { start, tip, null }));
                Assert.That(PrefabUtility.HasPrefabInstanceAnyOverrides(instance, false), Is.True);
                Assert.That(prefab.GetComponent<MultiFABRIKSolver>().Chains[1].Joints.Count, Is.EqualTo(2));
                Undo.PerformUndo();
                Assert.That(arm.Joints, Is.EqualTo(new[] { start, tip }));
                Assert.That(tip.parent, Is.SameAs(start));
            }
            finally
            {
                if (instance != null) Object.DestroyImmediate(instance);
                AssetDatabase.DeleteAsset(path);
            }
        }

        [Test]
        public void PinnedPointRootDisablesPullEvenForOneBranch()
        {
            _solver.AddChain("Arm", new[] { _root, _hub, _tip }, _tip);
            _solver.Chains[0].Junction = MultiFABRIKJunctionMode.Point;
            Assert.That(MultiFABRIKSolverEditor.PullDisabledReason(_solver.Chains, 0, 1f), Does.Contain("common attachment"));
            Assert.That(MultiFABRIKSolverEditor.PullDisabledReason(_solver.Chains, 0, 0f), Is.Null);
        }

        [UnityTest]
        public System.Collections.IEnumerator ManuallyAuthoredPointContinuationRendersWithoutChangingTheRig()
        {
            Transform mainPivot = Joint("Main pivot", _hub);
            Transform sidePivot = Joint("Side pivot", _hub);
            mainPivot.localPosition = sidePivot.localPosition = Vector3.zero;
            _tip.SetParent(mainPivot, true);
            Transform sideTip = Joint("Side tip", sidePivot);
            mainPivot.gameObject.AddComponent<HingeIKJoint>();
            sidePivot.gameObject.AddComponent<BallSocketIKJoint>();
            var main = _solver.AddChain("Through", new[] { _root, _hub, mainPivot, _tip }, _tip, 0.4f, 0.7f);
            var side = _solver.AddChain("Side", new[] { _hub, sidePivot, sideTip }, sideTip,
                junction: MultiFABRIKJunctionMode.Point);
            var problems = new List<string>();
            Assert.That(new MultiFABRIKTree().Build(new List<MultiFABRIKChain>(_solver.Chains), problems), Is.True,
                string.Join("\n", problems));
            string mainBefore = EditorJsonUtility.ToJson(main), sideBefore = EditorJsonUtility.ToJson(side);
            int transforms = _rig.GetComponentsInChildren<Transform>(true).Length;
            yield return Render(_solver);
            yield return Render(main);
            yield return Render(side);
            Assert.That(EditorJsonUtility.ToJson(main), Is.EqualTo(mainBefore));
            Assert.That(EditorJsonUtility.ToJson(side), Is.EqualTo(sideBefore));
            Assert.That(_rig.GetComponentsInChildren<Transform>(true), Has.Length.EqualTo(transforms));
            Assert.That(_tip.parent, Is.SameAs(mainPivot));
            Assert.That(_solver.Chains.Count, Is.EqualTo(2));
        }

        [UnityTest]
        public System.Collections.IEnumerator PointInspectorRendersWithOnlyJointZero()
        {
            var chain = _root.gameObject.AddComponent<MultiFABRIKChain>();
            chain.Junction = MultiFABRIKJunctionMode.Point;
            MultiFABRIKEditorUtility.AddToSolver(_solver, new[] { chain });
            yield return Render(chain);
            yield return Render(_solver);
            Assert.That(chain.Joints, Is.EqualTo(new[] { _root }));
        }
    }
}
