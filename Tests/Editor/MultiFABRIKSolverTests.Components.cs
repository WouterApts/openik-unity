using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using Object = UnityEngine.Object;

namespace OpenIK.Editor.Tests
{
    public partial class MultiFABRIKSolverTests
    {
        [Test]
        public void ChainsList_OnlyListedChainsAreSolved()
        {
            using var rig = Torso();
            // A chain component on the rig that is not in the list is ignored.
            Transform finger = new GameObject("Finger").transform;
            finger.SetParent(rig.Joints[5], false);
            finger.localPosition = Vector3.left * 0.1f;
            var unlisted = rig.Joints[5].gameObject.AddComponent<MultiFABRIKChain>();
            unlisted.Configure("Unlisted", new[] { rig.Joints[5], finger }, rig.Target(Vector3.zero));
            Assert.That(rig.Solver.Chains, Has.Count.EqualTo(3).And.No.Member(unlisted));
            Assert.That(rig.Solver.Lists(unlisted), Is.False);
            Assert.That(rig.Solver.Solve(), Is.True);
            Assert.That(rig.Solver.Chain.Count, Is.EqualTo(rig.Joints.Length));
            Assert.Throws<ArgumentException>(() => rig.Solver.GetPull(unlisted));

            rig.Solver.AddChain(unlisted, 0.5f, 0.25f);
            Assert.That(rig.Solver.Chain.Count, Is.EqualTo(rig.Joints.Length + 1));
            Assert.That(rig.Solver.GetPull(unlisted), Is.EqualTo(0.5f));
            Assert.That(rig.Solver.GetPin(unlisted), Is.EqualTo(0.25f));
            Assert.Throws<ArgumentException>(() => rig.Solver.AddChain(unlisted));
        }

        [Test]
        public void ChainsList_SortsByTreeRegardlessOfListOrder()
        {
            using var rig = new Rig(TorsoOffsets, TorsoParents);
            Transform finger = new GameObject("Finger").transform;
            finger.SetParent(rig.Joints[8], false);
            finger.localPosition = Vector3.right * 0.1f;
            var fingerChain = rig.Solver.AddChain("Finger", new[] { rig.Joints[8], finger }, rig.Target(finger.position + Vector3.up * 0.1f));
            var right = rig.Add("Right arm", null, 2, 6, 7, 8);
            var left = rig.Add("Left arm", rig.Target(new Vector3(-0.75f, 1.15f, 0.3f)), 2, 3, 4, 5);
            var trunk = rig.Add("Trunk", null, 0, 1, 2);
            // Listed in reverse. Each chain follows its owner; ready chains keep list order.
            Assert.That(rig.Solver.Chains, Is.EqualTo(new[] { trunk, right, fingerChain, left }));
            Assert.That(rig.Solver.Solve(), Is.True);
            AssertConverged(rig, 2, 3);
        }

        [Test]
        public void ChainsList_ReportsEmptyItemsAndRepeats()
        {
            using var rig = Torso();
            using (var serialized = new UnityEditor.SerializedObject(rig.Solver))
            {
                UnityEditor.SerializedProperty list = serialized.FindProperty("chains");
                list.arraySize += 2;
                list.GetArrayElementAtIndex(3).FindPropertyRelative("chain").objectReferenceValue = null;
                list.GetArrayElementAtIndex(4).FindPropertyRelative("chain").objectReferenceValue = rig.Solver.Chains[1];
                serialized.ApplyModifiedPropertiesWithoutUndo();
            }
            var problems = new List<string>();
            Assert.That(rig.Solver.BuildTree(new List<MultiFABRIKChain>(), new MultiFABRIKTree(), problems), Is.False);
            Assert.That(problems[0], Does.StartWith("Chains item 3 has no chain"));
            Assert.That(problems[1], Does.Contain("Left arm").And.Contain("more than once"));
            Assert.That(rig.Solver.Chains, Has.Count.EqualTo(3), "Chains skips empty items and repeats");
        }

        [Test]
        public void Sort_ReportsCycleSecondRootAndMissingRoot()
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.up, Vector3.up, Vector3.up, Vector3.right, Vector3.up },
                new[] { -1, 0, 1, 2, 0, 4 });
            Transform target = rig.Target(Vector3.up * 3f);
            var problems = new List<string>();

            // J4 starts a chain but is not a joint of any chain, while J0 is the root.
            var trunk = rig.Add("Trunk", target, 0, 1, 2, 3);
            var stray = rig.Add("Stray", null, 4, 5);
            Assert.That(MultiFABRIKTree.Sort(new List<MultiFABRIKChain> { trunk, stray }, problems), Is.False);
            Assert.That(problems, Has.Exactly(1).Matches<string>(p => p.StartsWith("Chain Stray on J4:") && p.Contains("root joint, J0")));

            // Stray and Loop each start on the other's joint: alone they have no root, and beside the
            // root chain they form a cycle.
            var loop = rig.Add("Loop", null, 5, 4);
            Assert.That(MultiFABRIKTree.Sort(new List<MultiFABRIKChain> { stray, loop }, problems), Is.False);
            Assert.That(problems[0], Does.StartWith("No root joint"));
            Assert.That(MultiFABRIKTree.Sort(new List<MultiFABRIKChain> { trunk, stray, loop }, problems), Is.False);
            Assert.That(problems, Has.Count.EqualTo(2).And.All.Contains("forms a cycle"));

            Assert.That(MultiFABRIKTree.Sort(new List<MultiFABRIKChain>(), problems), Is.False);
            Assert.That(problems[0], Does.Contain("Add the rig's FABRIK Chains"));
            using var empty = new Rig(new[] { Vector3.zero }, new[] { -1 });
            LogAssert.Expect(LogType.Error, new Regex("Add the rig's FABRIK Chains"));
            Assert.That(empty.Solver.Solve(), Is.False);
        }

        [Test]
        public void SeveralChainsOnOneJoint_AreSolvedTogether()
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.left, Vector3.right, Vector3.up }, new[] { -1, 0, 0, 0 });
            for (int i = 1; i < 4; i++) rig.Add($"Limb {i}", rig.Target(rig.Joints[i].position + Vector3.forward * 0.2f), 0, i);
            Assert.That(rig.Joints[0].GetComponents<MultiFABRIKChain>(), Has.Length.EqualTo(3));
            Assert.That(rig.Solver.Chains, Has.Count.EqualTo(3));
            rig.Solver.RootPin = 0f;
            rig.Solver.Solve();
            AssertConverged(rig, 0, 1, 2);
            AssertRigid(rig);
        }

        [Test]
        public void DisabledComponent_HoldsTheChainPose()
        {
            using var rig = Torso();
            rig.Solver.Chains[2].enabled = false;
            rig.Joints[6].localRotation = Quaternion.Euler(10f, -15f, 5f);
            rig.Solver.Solve();
            Assert.That(rig.Solver.Chains[2].Enabled, Is.False);
            Assert.That(rig.Solver.Chains[2].LastResult.HasTarget, Is.False);
            RotationNear(Quaternion.Inverse(rig.Rotation(2)) * rig.Rotation(6), rig.Joints[6].localRotation);
            AssertConverged(rig, 1);
        }

        [Test]
        public void PullAndPin_AreReadEachSolve()
        {
            using var rig = Torso();
            rig.Solver.Solve();
            Vector3 moved = rig.Position(2);
            rig.Solver.SetPull(rig.Solver.Chains[1], 0f);
            rig.Solver.SetPull(rig.Solver.Chains[2], 0f);
            rig.Solver.Solve();
            Near(rig.Position(2), rig.InitialPositions[2]);
            Assert.That(Vector3.Distance(moved, rig.InitialPositions[2]), Is.GreaterThan(0.01f));
            rig.Solver.SetPin(rig.Solver.Chains[0], 2f);
            Assert.That(rig.Solver.GetPin(rig.Solver.Chains[0]), Is.EqualTo(1f));
        }

        [Test]
        public void AddChain_RejectsInvalidTreesWithoutAddingAComponentOrItem()
        {
            using var rig = Torso();
            rig.Solver.Solve();
            Assert.Throws<ArgumentException>(() => rig.Add("Shares the elbow", rig.Target(Vector3.up), 3, 4));
            Assert.That(rig.Joints[3].GetComponent<MultiFABRIKChain>(), Is.Null);
            Assert.That(rig.Solver.Chains, Has.Count.EqualTo(3));
            Assert.That(rig.Solver.Solve(), Is.True);
        }

        [Test]
        public void RemoveChain_KeepsTheComponentAndRebuilds()
        {
            using var rig = Torso();
            rig.Solver.Solve();
            var right = rig.Solver.Chains[2];
            Assert.That(rig.Solver.RemoveChain(right), Is.True);
            Assert.That(right != null, Is.True);
            Assert.That(rig.Solver.Lists(right), Is.False);
            Assert.That(rig.Solver.Chain.Count, Is.EqualTo(6));
            Assert.That(rig.Solver.RemoveChain(right), Is.False);
            rig.Solver.AddChain(right);
            Assert.That(rig.Solver.Chain.Count, Is.EqualTo(rig.Joints.Length));
        }

        [Test]
        public void DestroyedChain_LeavesTheListAndRebuildsAnInitializedSolver()
        {
            using var rig = Torso();
            rig.Solver.Solve();
            rig.Solver.SnapToSolution();

            // Unity calls OnDestroy only for components that ran Awake in Play mode, so call the hook directly.
            var right = rig.Solver.Chains[2];
            rig.Solver.OnChainDestroyed(right);
            Object.DestroyImmediate(right);
            Assert.That(rig.Solver.Chains, Has.Count.EqualTo(2));
            Assert.That(rig.Solver.Chain.Count, Is.EqualTo(6));
            for (int i = 0; i < rig.Joints.Length; i++) Near(rig.Joints[i].position, rig.InitialPositions[i], "rest pose restored");
            Assert.That(rig.Solver.Solve(), Is.True);
            AssertConverged(rig, 1);
        }

        [Test]
        public void DestroyedChain_OnADisabledSolverRebuildsOnTheNextSolve()
        {
            using var rig = Torso();
            rig.Solver.Solve();
            rig.Solver.enabled = false;
            var right = rig.Solver.Chains[2];
            rig.Solver.OnChainDestroyed(right);
            Object.DestroyImmediate(right);
            rig.Solver.enabled = true;
            Assert.That(rig.Solver.Chains, Has.Count.EqualTo(2));
            Assert.That(rig.Solver.Solve(), Is.True);
            Assert.That(rig.Solver.Chain.Count, Is.EqualTo(6));
            AssertConverged(rig, 1);
        }
    }
}
