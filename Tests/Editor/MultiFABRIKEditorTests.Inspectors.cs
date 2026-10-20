using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEditor;
using UnityEngine;
using UnityEngine.TestTools;

namespace OpenIK.Editor.Tests
{
    public partial class MultiFABRIKEditorTests
    {
        [UnityTest]
        public IEnumerator ChainInspector_RendersWithoutASolver()
        {
            var loose = new GameObject("Loose");
            UnityEngine.SceneManagement.SceneManager.MoveGameObjectToScene(loose, _testScene);
            var parent = new GameObject("Parent").transform;
            UnityEngine.SceneManagement.SceneManager.MoveGameObjectToScene(parent.gameObject, _testScene);
            try
            {
                Transform joint = Joint("Joint", loose.transform);
                var chain = joint.gameObject.AddComponent<MultiFABRIKChain>();
                Assert.That(MultiFABRIKEditorUtility.FindSolver(chain), Is.Null);
                loose.AddComponent<FABRIKSolver>();
                loose.transform.SetParent(parent, false);
                yield return Render(chain);
                Assert.That(parent.GetComponent<MultiFABRIKSolver>(), Is.Null);
                Assert.That(loose.GetComponent<MultiFABRIKSolver>(), Is.Null);
                Assert.That(chain.Joints, Is.EqualTo(new[] { joint }));
            }
            finally
            {
                Object.DestroyImmediate(loose);
                Object.DestroyImmediate(parent.gameObject);
            }
        }

        [UnityTest]
        public IEnumerator ChainInspector_RendersAnAttachedRootAndBranchWithPoles()
        {
            var trunk = _solver.AddChain("Trunk", new[] { _root, _hub }, _hub);
            Transform elbow = Joint("Elbow", _tip);
            Transform hand = Joint("Hand", elbow);
            var arm = _solver.AddChain("Arm", new[] { _hub, _tip, elbow, hand }, hand);
            arm.AddPole(elbow, _root);
            Assert.That(MultiFABRIKEditorUtility.FindSolver(arm), Is.SameAs(_solver));
            yield return Render(trunk);
            yield return Render(arm);
        }

        [UnityTest]
        public IEnumerator SolverInspector_RendersAPointJunctionThatNeedsPivots()
        {
            PointSetupBranches();
            MultiFABRIKEditorUtility.SetJunction(_hub, MultiFABRIKJunctionMode.Point);
            Assert.That(_solver.Chains[1].Junction, Is.EqualTo(MultiFABRIKJunctionMode.Point));
            Assert.That(_solver.Chains[2].Junction, Is.EqualTo(MultiFABRIKJunctionMode.Point));
            var problems = new List<string>();
            Assert.That(new MultiFABRIKTree().Build(new List<MultiFABRIKChain>(_solver.Chains), problems), Is.False);
            Assert.That(problems, Has.Some.Contains("same local offset"));
            yield return Render(_solver);
            yield return Render(_solver.Chains[1]);
        }

        [UnityTest]
        public IEnumerator SolverInspector_RendersWithoutChains()
        {
            yield return Render(_solver);
        }

        [Test]
        public void SolverListItems_HoldPullAndPin()
        {
            var arm = _solver.AddChain("Arm", new[] { _root, _hub, _tip }, _tip, 0.4f, 0.2f);
            using var serialized = new SerializedObject(_solver);
            SerializedProperty item = serialized.FindProperty("chains").GetArrayElementAtIndex(0);
            Assert.That(item.FindPropertyRelative("chain").objectReferenceValue, Is.SameAs(arm));
            Assert.That(item.FindPropertyRelative("pull").floatValue, Is.EqualTo(0.4f));
            item.FindPropertyRelative("pin").floatValue = 0.7f;
            serialized.ApplyModifiedPropertiesWithoutUndo();
            Assert.That(_solver.GetPin(arm), Is.EqualTo(0.7f));
            using var chainObject = new SerializedObject(arm);
            Assert.That(chainObject.FindProperty("pull"), Is.Null);
            Assert.That(chainObject.FindProperty("pin"), Is.Null);
        }

        [Test]
        public void SolverListAndChainInspector_EditTheChainsOwnTarget()
        {
            var arm = _solver.AddChain("Arm", new[] { _root, _hub, _tip });
            var solverInspector = (MultiFABRIKSolverEditor)UnityEditor.Editor.CreateEditor(_solver);
            var chainInspector = UnityEditor.Editor.CreateEditor(arm);
            try
            {
                SerializedObject item = solverInspector.ChainObject(arm);
                item.FindProperty("target").objectReferenceValue = _tip;
                item.ApplyModifiedPropertiesWithoutUndo();
                Assert.That(arm.Target, Is.SameAs(_tip));
                chainInspector.serializedObject.Update();
                Assert.That(chainInspector.serializedObject.FindProperty("target").objectReferenceValue, Is.SameAs(_tip));

                chainInspector.serializedObject.FindProperty("target").objectReferenceValue = _hub;
                chainInspector.serializedObject.ApplyModifiedPropertiesWithoutUndo();
                item.Update();
                Assert.That(item.FindProperty("target").objectReferenceValue, Is.SameAs(_hub));
            }
            finally
            {
                Object.DestroyImmediate(solverInspector);
                Object.DestroyImmediate(chainInspector);
            }
        }

        [Test]
        public void SetJunction_SetsEveryChainOnTheJointAsOneUndoStep()
        {
            PointSetupBranches();
            Undo.IncrementCurrentGroup();
            MultiFABRIKEditorUtility.SetJunction(_hub, MultiFABRIKJunctionMode.Point);
            Assert.That(_solver.Chains[1].Junction, Is.EqualTo(MultiFABRIKJunctionMode.Point));
            Assert.That(_solver.Chains[2].Junction, Is.EqualTo(MultiFABRIKJunctionMode.Point));
            Assert.That(_solver.Chains[0].Junction, Is.EqualTo(MultiFABRIKJunctionMode.Rigid));
            Undo.PerformUndo();
            Assert.That(_solver.Chains[1].Junction, Is.EqualTo(MultiFABRIKJunctionMode.Rigid));
            Assert.That(_solver.Chains[2].Junction, Is.EqualTo(MultiFABRIKJunctionMode.Rigid));
        }

        [Test]
        public void DroppedObjects_AddTheirChainsOnceAsOneUndoStep()
        {
            var trunk = _root.gameObject.AddComponent<MultiFABRIKChain>();
            trunk.Configure("Trunk", new[] { _root, _hub }, null);
            var arm = _hub.gameObject.AddComponent<MultiFABRIKChain>();
            arm.Configure("Arm", new[] { _hub, _tip }, _tip);
            var other = _hub.gameObject.AddComponent<MultiFABRIKChain>();
            other.Configure("Other", new[] { _hub, Joint("Other", _hub) }, null);

            // A dragged joint brings the chains on it and below it; a dragged chain brings itself.
            List<MultiFABRIKChain> dropped = MultiFABRIKEditorUtility.ChainsIn(new Object[] { _hub.gameObject, arm, _rig.transform });
            Assert.That(dropped, Is.EquivalentTo(new[] { trunk, arm, other }));
            Undo.IncrementCurrentGroup();
            Assert.That(MultiFABRIKEditorUtility.AddToSolver(_solver, new[] { arm }), Is.EqualTo(1));
            Undo.IncrementCurrentGroup();
            Assert.That(MultiFABRIKEditorUtility.AddToSolver(_solver, dropped), Is.EqualTo(2));
            Assert.That(_solver.Chains, Is.EqualTo(new[] { trunk, arm, other }));
            Assert.That(_solver.GetPull(other), Is.EqualTo(1f));
            Assert.That(_solver.GetPin(other), Is.EqualTo(0f));
            Undo.PerformUndo();
            Assert.That(_solver.Chains, Is.EqualTo(new[] { arm }));
        }

        [Test]
        public void Overlaps_ReportAnotherSolverOnTheSameJoints()
        {
            _solver.AddChain("Arm", new[] { _root, _hub, _tip }, _tip);
            var overlaps = new List<string>();
            MultiFABRIKEditorUtility.FindOverlaps(_solver, _solver.Chains, overlaps);
            Assert.That(overlaps, Is.Empty);

            var leftover = _rig.AddComponent<FABRIKSolver>();
            using (var serialized = new SerializedObject(leftover))
            {
                SerializedProperty joints = serialized.FindProperty("chainJoints");
                joints.arraySize = 2;
                joints.GetArrayElementAtIndex(0).objectReferenceValue = _hub;
                joints.GetArrayElementAtIndex(1).objectReferenceValue = _tip;
                serialized.ApplyModifiedPropertiesWithoutUndo();
            }
            MultiFABRIKEditorUtility.FindOverlaps(_solver, _solver.Chains, overlaps);
            Assert.That(overlaps, Has.Count.EqualTo(1));
            Assert.That(overlaps[0], Does.Contain("Hub").And.Contain("FABRIK Solver"));
        }
    }
}
