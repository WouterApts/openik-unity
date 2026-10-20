using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using Object = UnityEngine.Object;

namespace OpenIK.Editor.Tests
{
    public class SolverChainTreeTests
    {
        private const float Epsilon = 1e-5f;

        // Root, Spine, Chest, then a left and a right arm that both start at Chest.
        private static readonly string[] TreeNames = { "Root", "Spine", "Chest", "L Shoulder", "L Elbow", "L Hand", "R Shoulder", "R Elbow", "R Hand" };
        private static readonly int[] TreeParents = { -1, 0, 1, 2, 3, 4, 2, 6, 7 };

        private static readonly Vector3[] TreeOffsets =
        {
            Vector3.zero, new(0f, 0.5f, 0f), new(0f, 0.5f, 0f),
            new(-0.2f, 0.1f, 0f), new(-0.3f, 0f, 0.05f), new(-0.3f, 0f, -0.05f),
            new(0.2f, 0.1f, 0f), new(0.3f, 0f, 0.05f), new(0.3f, 0f, -0.05f)
        };

        [Test]
        public void Build_WithParentIndices_SetsParentsAndChildLists()
        {
            using var rig = new Rig(TreeOffsets, TreeParents);
            SolverChain chain = rig.Initialize(TreeParents);

            Assert.That(chain.IsTree, Is.True);
            for (int i = 0; i < TreeParents.Length; i++)
                Assert.That(chain.Joints[i].ParentIndex, Is.EqualTo(TreeParents[i]), TreeNames[i]);

            Assert.That(chain.GetChildCount(2), Is.EqualTo(2));
            Assert.That(chain.GetChild(2, 0), Is.EqualTo(3));
            Assert.That(chain.GetChild(2, 1), Is.EqualTo(6));
            Assert.That(chain.GetChildCount(0), Is.EqualTo(1));
            Assert.That(chain.GetChild(0, 0), Is.EqualTo(1));

            for (int i = 0; i < TreeParents.Length; i++)
                Assert.That(chain.IsLeaf(i), Is.EqualTo(i == 5 || i == 8), TreeNames[i]);
        }

        [Test]
        public void Build_RejectsAParentThatIsNotAnEarlierJoint()
        {
            using var rig = new Rig(TreeOffsets, TreeParents);
            var chain = new SolverChain();

            Assert.Throws<ArgumentException>(() => chain.Build(rig.Joints, new[] { -1, 0, 1, 2, 5, 4, 2, 6, 7 }));
            Assert.Throws<ArgumentException>(() => chain.Build(rig.Joints, new[] { 0, 0, 1, 2, 3, 4, 2, 6, 7 }));
            Assert.Throws<ArgumentException>(() => chain.Build(rig.Joints, new[] { -1, 0, 1 }));
        }

        [Test]
        public void SetupParents_AreTheParentJoints()
        {
            using var rig = new Rig(TreeOffsets, TreeParents);
            var shoulder = rig.Joints[6].gameObject.AddComponent<BallSocketIKJoint>();
            rig.Joints[6].localRotation = Quaternion.Euler(0f, 0f, -30f);
            SolverChain chain = rig.Initialize(TreeParents);

            Transform chest = rig.Joints[2];
            Assert.That(chain.GetSetupParentTransform(6), Is.SameAs(chest));
            Assert.That(chain.GetSetupParentTransform(3), Is.SameAs(chest));
            Assert.That(chain.GetSetupParentTransform(0), Is.SameAs(rig.Root.transform));
            Assert.That(shoulder.IKParentTransform, Is.SameAs(chest));

            AssertVector(chain.Joints[6].RestLocalOffset, TreeOffsets[6], "R Shoulder rest offset");
            Assert.That(Quaternion.Angle(chain.Joints[6].RestLocalRotation, Quaternion.Euler(0f, 0f, -30f)), Is.LessThan(1e-3f));
            Assert.That(chain.Joints[6].Segment.MaxReach, Is.EqualTo(TreeOffsets[6].magnitude).Within(Epsilon));
        }

        [Test]
        public void RotateAndRepositionDownstream_OnATree_MovesExactlyTheSubtree()
        {
            using var rig = new Rig(TreeOffsets, TreeParents);
            SolverChain chain = rig.Initialize(TreeParents);
            chain.SyncSolverStateFromTransforms();
            Vector3[] before = SolverPositions(chain);
            Quaternion delta = Quaternion.Euler(20f, -35f, 10f);

            // Turning the left shoulder moves only the left arm below it.
            chain.Joints[3].SolverRotation = delta * chain.Joints[3].SolverRotation;
            chain.RotateAndRepositionDownstream(3, delta);
            Vector3[] afterShoulder = SolverPositions(chain);
            for (int i = 0; i < before.Length; i++)
            {
                Vector3 expected = i == 4 || i == 5 ? before[3] + delta * (before[i] - before[3]) : before[i];
                AssertVector(afterShoulder[i], expected, TreeNames[i]);
            }

            // Turning the chest moves both arms around it, and nothing above it.
            chain.Joints[2].SolverRotation = delta * chain.Joints[2].SolverRotation;
            chain.RotateAndRepositionDownstream(2, delta);
            Vector3[] afterChest = SolverPositions(chain);
            for (int i = 0; i < before.Length; i++)
            {
                Vector3 expected = i > 2 ? afterShoulder[2] + delta * (afterShoulder[i] - afterShoulder[2]) : afterShoulder[i];
                AssertVector(afterChest[i], expected, TreeNames[i]);
            }

            // A second call starts from cleared state, so it moves the same subtree again.
            chain.Joints[6].SolverRotation = delta * chain.Joints[6].SolverRotation;
            chain.RotateAndRepositionDownstream(6, delta);
            Vector3[] afterRightShoulder = SolverPositions(chain);
            for (int i = 0; i < before.Length; i++)
            {
                Vector3 expected = i > 6 ? afterChest[6] + delta * (afterChest[i] - afterChest[6]) : afterChest[i];
                AssertVector(afterRightShoulder[i], expected, TreeNames[i]);
            }
        }

        [Test]
        public void GetPathLength_SumsTheSegmentsFromTheRoot()
        {
            using var rig = new Rig(TreeOffsets, TreeParents);
            SolverChain chain = rig.Initialize(TreeParents);

            float spine = TreeOffsets[1].magnitude + TreeOffsets[2].magnitude;
            Assert.That(chain.GetPathLength(0), Is.EqualTo(0f));
            Assert.That(chain.GetPathLength(2), Is.EqualTo(spine).Within(Epsilon));
            Assert.That(chain.GetPathLength(5),
                Is.EqualTo(spine + TreeOffsets[3].magnitude + TreeOffsets[4].magnitude + TreeOffsets[5].magnitude).Within(Epsilon));
            Assert.That(chain.GetPathLength(8),
                Is.EqualTo(spine + TreeOffsets[6].magnitude + TreeOffsets[7].magnitude + TreeOffsets[8].magnitude).Within(Epsilon));

            float total = 0f;
            for (int i = 1; i < TreeOffsets.Length; i++)
                total += TreeOffsets[i].magnitude;
            Assert.That(chain.ChainLength, Is.EqualTo(total).Within(Epsilon));
        }

        [Test]
        public void PlainChain_IsNotATree_AndWritesEveryRotationButTheLast()
        {
            Vector3[] offsets = { Vector3.zero, new(0f, 1f, 0f), new(0f, 0.8f, 0.1f), new(0f, 0.6f, 0f) };
            using var rig = new Rig(offsets, null);
            SolverChain chain = rig.Initialize(null);
            var output = new IKSolverOutput();

            Assert.That(chain.IsTree, Is.False);
            for (int i = 0; i < offsets.Length; i++)
                Assert.That(chain.IsLeaf(i), Is.EqualTo(i == offsets.Length - 1));

            output.Populate(chain, 1, 0f, true);
            Assert.That(output.WritesRotation, Is.EqualTo(new[] { true, true, true, false }));

            output.Populate(chain, 1, 0f, true, writeEndRotation: true);
            Assert.That(output.WritesRotation, Is.EqualTo(new[] { true, true, true, true }));
        }

        [Test]
        public void TreeOutput_WritesEveryJointWithChildren()
        {
            using var rig = new Rig(TreeOffsets, TreeParents);
            SolverChain chain = rig.Initialize(TreeParents);
            var output = new IKSolverOutput();

            output.Populate(chain, 1, 0f, true);

            for (int i = 0; i < TreeParents.Length; i++)
                Assert.That(output.WritesRotation[i], Is.EqualTo(!(i == 5 || i == 8)), TreeNames[i]);
        }

        [Test]
        public void TreeOutput_ApplyTo_KeepsTheLeavesLocalRotation()
        {
            using var rig = new Rig(TreeOffsets, TreeParents);
            rig.Joints[8].localRotation = Quaternion.Euler(15f, 0f, 40f);
            SolverChain chain = rig.Initialize(TreeParents);
            chain.SyncSolverStateFromTransforms();
            Quaternion handLocal = rig.Joints[8].localRotation;

            Quaternion delta = Quaternion.Euler(0f, 0f, 50f);
            chain.Joints[6].SolverRotation = delta * chain.Joints[6].SolverRotation;
            chain.RotateAndRepositionDownstream(6, delta);
            chain.Joints[8].SolverRotation = Quaternion.Euler(80f, 10f, 0f);
            var output = new IKSolverOutput();
            output.Populate(chain, 1, 0f, true);
            output.ApplyTo(new List<Transform>(rig.Joints));

            AssertVector(rig.Joints[8].position, chain.Joints[8].SolverPosition, "R Hand position");
            Assert.That(Quaternion.Angle(rig.Joints[6].rotation, chain.Joints[6].SolverRotation), Is.LessThan(1e-3f));
            Assert.That(Quaternion.Angle(rig.Joints[8].localRotation, handLocal), Is.LessThan(1e-3f), "The leaf keeps its local rotation.");
        }

        // ---------------------------------------------------------------------------------------
        // A solver that builds its own tree
        // ---------------------------------------------------------------------------------------

        [Test]
        public void SolverDerivedFromTheLifecycleBase_SolvesAndAppliesATreeWithSpeedLimits()
        {
            using var rig = new Rig(TreeOffsets, TreeParents);
            var shoulder = rig.Joints[6].gameObject.AddComponent<HingeIKJoint>();
            shoulder.hingeAxis = Vector3.right;
            shoulder.limitSpeed = true;
            shoulder.maxAngularSpeed = 90f;
            rig.Joints[8].localRotation = Quaternion.Euler(15f, 0f, 40f);
            var solver = rig.Root.AddComponent<TurningTreeSolver>();
            solver.Joints = rig.Joints;
            solver.Parents = TreeParents;
            solver.TurnJoint = 6;
            solver.TurnDegrees = 40f;

            Quaternion shoulderLocal = rig.Joints[6].localRotation;
            Quaternion elbowLocal = rig.Joints[7].localRotation;
            Quaternion handLocal = rig.Joints[8].localRotation;
            Vector3 leftHand = rig.Joints[5].position;

            Assert.That(solver.Solve(), Is.False, "Without a goal, nothing is solved.");
            solver.HasTarget = true;
            Assert.That(solver.Solve(), Is.True);
            IKApplicationStatus status = solver.Apply(0.1f);

            Assert.That(status.Applied, Is.True);
            Assert.That(status.AnyJointLimited, Is.True);
            Assert.That(status.PositionError, Is.EqualTo(TurningTreeSolver.ReportedError));
            Assert.That(Quaternion.Angle(rig.Joints[6].localRotation, shoulderLocal), Is.EqualTo(9f).Within(0.01f),
                "The shoulder turns at its speed limit.");
            Assert.That(Quaternion.Angle(rig.Joints[7].localRotation, elbowLocal), Is.LessThan(1e-3f));
            Assert.That(Quaternion.Angle(rig.Joints[8].localRotation, handLocal), Is.LessThan(1e-3f), "The leaf keeps its local rotation.");
            AssertVector(rig.Joints[5].position, leftHand, "The other arm stays put.");
        }

        // Turns one joint of its own tree around its local X axis, and reports a fixed error.
        private sealed class TurningTreeSolver : OpenIKSolverBase
        {
            public const float ReportedError = 0.25f;

            public List<Transform> Joints;
            public int[] Parents;
            public int TurnJoint;
            public float TurnDegrees;
            public bool HasTarget;

            protected override bool ValidateSetup() => Joints != null && Joints.Count > 1;

            protected override void BuildChain(SolverChain chain) => chain.Initialize(Joints, Parents);

            protected override bool HasGoal => HasTarget;

            protected override SolveResult SolveChain()
            {
                SolverJoint joint = Chain.Joints[TurnJoint];
                Quaternion turn = Quaternion.AngleAxis(TurnDegrees, joint.SolverRotation * Vector3.right);
                joint.SolverRotation = turn * joint.SolverRotation;
                Chain.RotateAndRepositionDownstream(TurnJoint, turn);
                return new SolveResult(1, 0f);
            }

            protected override bool IsConverged(in SolveResult result) => true;

            protected override float ComputeActualPositionError() => ReportedError;
        }

        // ---------------------------------------------------------------------------------------
        // Poles on a tree
        // ---------------------------------------------------------------------------------------

        [Test]
        public void IdentityMapResolve_GivesTheResolverSections()
        {
            // Hip (Ball Socket), Thigh (Hinge), Knee (Hinge), Ankle, Toe.
            Vector3[] offsets = { Vector3.zero, new(0f, -0.2f, 0f), new(0f, -0.5f, 0.2f), new(0f, -0.5f, -0.2f), new(0f, -0.1f, 0.2f) };
            using var rig = new Rig(offsets, null);
            rig.Joints[0].gameObject.AddComponent<BallSocketIKJoint>();
            rig.Joints[1].gameObject.AddComponent<HingeIKJoint>();
            rig.Joints[2].gameObject.AddComponent<HingeIKJoint>();
            SolverChain chain = rig.Initialize(null);
            var poles = new List<IKPole> { new(rig.Joints[2], rig.Root.transform), new(rig.Joints[1], rig.Root.transform) };

            var expected = new List<PoleSection>();
            PoleSectionResolver.Resolve(rig.Joints, poles, expected, null);
            Assert.That(expected.Count, Is.EqualTo(2));

            var chainPoles = new ChainPoles();
            chainPoles.Resolve(poles, rig.Joints, chain, null, "Test");
            AssertSections(chainPoles, expected);

            var mapped = new ChainPoles();
            mapped.Resolve(poles, rig.Joints, new[] { 0, 1, 2, 3, 4 }, 4, new bool[5], chain, null, "Test");
            AssertSections(mapped, expected);
        }

        [Test]
        public void TreeResolve_MapsSectionsToTreeIndices_AndSkipsSectionsAtABranchPoint()
        {
            Vector3[] offsets =
            {
                Vector3.zero, new(0f, 0.5f, 0f), new(0f, 0.5f, 0f),
                new(-0.2f, 0.1f, 0f), new(-0.3f, 0f, 0.05f), new(-0.3f, 0f, -0.05f),
                new(0.2f, 0.1f, 0f), new(0.3f, 0f, 0.05f), new(0.3f, 0f, -0.05f), new(0.1f, 0f, 0f)
            };
            int[] parents = { -1, 0, 1, 2, 3, 4, 2, 6, 7, 8 };
            using var rig = new Rig(offsets, parents);
            SolverChain chain = rig.Initialize(parents);

            // The right arm as its own chain: Chest, R Shoulder, R Elbow, R Hand, R Tip.
            var arm = new List<Transform> { rig.Joints[2], rig.Joints[6], rig.Joints[7], rig.Joints[8], rig.Joints[9] };
            int[] treeIndex = { 2, 6, 7, 8, 9 };
            bool[] blocked = { true, false, false, false, false };
            var poles = new List<IKPole> { new(rig.Joints[8], rig.Root.transform), new(rig.Joints[6], rig.Root.transform) };

            var chainPoles = new ChainPoles();
            LogAssert.Expect(LogType.Warning, new Regex("starts at \"Chest\""));
            chainPoles.Resolve(poles, arm, treeIndex, 9, blocked, chain, null, "Test");

            Assert.That(chainPoles.Count, Is.EqualTo(1), "The shoulder's section starts at the branch point and is skipped.");
            chainPoles.GetSection(0, out int start, out int bend, out int end);
            Assert.That(start, Is.EqualTo(7));
            Assert.That(bend, Is.EqualTo(8));
            Assert.That(end, Is.EqualTo(9));
        }

        [Test]
        public void BlockedSections_CanEndAtABranchPointButNotPassIt()
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.up, Vector3.up, Vector3.up, Vector3.up }, null);
            var sections = new List<PoleSection> { new(0, 0, 1, 2), new(1, 2, 3, 4) };
            var problems = new List<string>();

            PoleSectionResolver.RemoveBlockedSections(rig.Joints, new[] { false, false, true, false, false }, sections, problems);

            Assert.That(sections.Count, Is.EqualTo(1));
            Assert.That(sections[0].Bend, Is.EqualTo(1), "A section that ends at the blocked joint stays.");
            Assert.That(problems.Count, Is.EqualTo(1));

            sections = new List<PoleSection> { new(0, 1, 2, 4) };
            PoleSectionResolver.RemoveBlockedSections(rig.Joints, new[] { false, false, false, true, false }, sections, problems);
            Assert.That(sections, Is.Empty, "A section that passes the blocked joint is removed.");
            Assert.That(problems[1], Does.Contain("passes"));
        }

        // ---------------------------------------------------------------------------------------
        // Fixture
        // ---------------------------------------------------------------------------------------

        private static void AssertSections(ChainPoles chainPoles, List<PoleSection> expected)
        {
            Assert.That(chainPoles.Count, Is.EqualTo(expected.Count));
            for (int i = 0; i < expected.Count; i++)
            {
                chainPoles.GetSection(i, out int start, out int bend, out int end);
                Assert.That(start, Is.EqualTo(expected[i].Start), $"Section {i} start");
                Assert.That(bend, Is.EqualTo(expected[i].Bend), $"Section {i} bend");
                Assert.That(end, Is.EqualTo(expected[i].End), $"Section {i} end");
            }
        }

        private static Vector3[] SolverPositions(SolverChain chain)
        {
            var positions = new Vector3[chain.Count];
            for (int i = 0; i < positions.Length; i++)
                positions[i] = chain.Joints[i].SolverPosition;
            return positions;
        }

        private static void AssertVector(Vector3 actual, Vector3 expected, string message)
        {
            Assert.That(Vector3.Distance(actual, expected), Is.LessThan(Epsilon), $"{message}: {actual} vs {expected}");
        }

        // Joints parented as given (or one after another without parents), below a rotated root object.
        private sealed class Rig : IDisposable
        {
            public Rig(Vector3[] localOffsets, int[] parents)
            {
                Root = new GameObject("Rig");
                Root.transform.SetPositionAndRotation(new Vector3(0.3f, -0.1f, 0.2f), Quaternion.Euler(10f, 25f, -5f));
                for (int i = 0; i < localOffsets.Length; i++)
                {
                    Transform parent = i == 0 ? Root.transform : Joints[parents != null ? parents[i] : i - 1];
                    Transform joint = new GameObject(i < TreeNames.Length ? TreeNames[i] : $"Joint {i}").transform;
                    joint.SetParent(parent, false);
                    joint.localPosition = localOffsets[i];
                    Joints.Add(joint);
                }
            }

            public GameObject Root { get; }
            public List<Transform> Joints { get; } = new();

            public SolverChain Initialize(int[] parents)
            {
                var chain = new SolverChain();
                chain.Initialize(Joints, parents);
                return chain;
            }

            public void Dispose()
            {
                Object.DestroyImmediate(Root);
            }
        }
    }
}
