using System;
using System.Collections.Generic;
using System.Reflection;
using System.Text.RegularExpressions;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;
using Object = UnityEngine.Object;

namespace OpenIK.Editor.Tests
{
    public class PoleTests
    {
        private const float PositionEpsilon = 1e-4f;
        private const float AngleEpsilon = 0.01f;

        public enum SolverKind
        {
            FABRIK,
            CCD,
            Jacobian
        }

        // Five free joints along +Y; the bend joint J2 sticks out along +X.
        private static readonly Vector3[] BentChain =
        {
            Vector3.zero, new(0f, 1f, 0f), new(0.3f, 1f, 0f), new(-0.3f, 1f, 0f), new(0f, 0.5f, 0f)
        };

        // ---------------------------------------------------------------------------------------
        // Swivel
        // ---------------------------------------------------------------------------------------

        [Test]
        public void Swivel_KeepsLengthsAndJointsOutsideTheSection_AndFacesThePole()
        {
            using var rig = new Rig(BentChain, SolverKind.FABRIK);
            rig.AddPole(2, new Vector3(0f, 2f, 2f));
            rig.Solver.MaxIterations = 0;
            rig.Target.position = rig.Joints[4].position;
            rig.Initialize();

            Pose[] before = rig.WorldPoses();
            Assert.That(rig.Solver.Solve(), Is.True);
            Pose[] after = rig.SolvedPoses();

            foreach (int i in new[] { 0, 1, 3, 4 })
                AssertNear(after[i].position, before[i].position, $"J{i} position");
            foreach (int i in new[] { 3, 4 })
                Assert.That(Quaternion.Angle(after[i].rotation, before[i].rotation), Is.LessThan(AngleEpsilon), $"J{i} rotation");
            for (int i = 1; i < after.Length; i++)
            {
                Assert.That(Vector3.Distance(after[i].position, after[i - 1].position),
                    Is.EqualTo(Vector3.Distance(before[i].position, before[i - 1].position)).Within(PositionEpsilon));
            }

            Assert.That(BendAngleToPole(after, 1, 2, 3, rig.PoleTargets[0].position), Is.LessThan(AngleEpsilon));
        }

        [Test]
        public void HalfWeight_TurnsHalfwayFromTheIncomingBend()
        {
            using var rig = new Rig(BentChain, SolverKind.FABRIK);
            // The incoming bend points along +X and the pole along +Z, 90 degrees apart.
            rig.AddPole(2, new Vector3(0f, 2f, 2f), 0.5f);
            rig.Solver.MaxIterations = 0;
            rig.Target.position = rig.Joints[4].position;
            rig.Initialize();

            rig.Solver.Solve();

            Assert.That(BendAngleToPole(rig.SolvedPoses(), 1, 2, 3, rig.PoleTargets[0].position), Is.EqualTo(45f).Within(AngleEpsilon));
        }

        [Test]
        public void HalfWeight_OnASectionStartingAtTheRoot_UsesTheRootParentFrame()
        {
            using var rig = new Rig(BentChain, SolverKind.FABRIK);
            rig.Root.transform.rotation = Quaternion.Euler(20f, 35f, -10f);

            // Place the pole 90 degrees around the J0-J2 line from J1's incoming bend.
            Vector3 axis = (rig.Joints[2].position - rig.Joints[0].position).normalized;
            Vector3 incoming = Vector3.ProjectOnPlane(rig.Joints[1].position - rig.Joints[0].position, axis);
            rig.AddPole(1, rig.Joints[0].position + Vector3.Cross(axis, incoming).normalized * 2f, 0.5f);
            rig.Solver.MaxIterations = 0;
            rig.Target.position = rig.Joints[4].position;
            rig.Initialize();

            rig.Solver.Solve();

            Assert.That(BendAngleToPole(rig.SolvedPoses(), 0, 1, 2, rig.PoleTargets[0].position), Is.EqualTo(45f).Within(0.05f));
        }

        [Test]
        public void Apply_IsRepeatableWithinOneSolve()
        {
            using var rig = new Rig(BentChain, SolverKind.FABRIK);
            rig.AddPole(2, rig.Joints[2].position + new Vector3(0f, 0.4f, 2f), 0.5f);

            var chain = new SolverChain();
            chain.Initialize(rig.Joints);
            var poles = new ChainPoles();
            poles.Resolve(new List<IKPole>(rig.Solver.Poles), rig.Joints, chain, null, "Test");
            chain.SyncSolverStateFromTransforms();
            poles.Capture(chain);

            poles.Apply(chain);
            Vector3 once = chain.Joints[2].SolverPosition;
            poles.Apply(chain);

            AssertNear(chain.Joints[2].SolverPosition, once, "bend joint after a second application");
        }

        [Test]
        public void PoleOnTheSectionLine_AndStraightSection_ChangeNothing()
        {
            using var rig = new Rig(BentChain, SolverKind.FABRIK);
            Vector3 onLine = rig.Joints[1].position + (rig.Joints[3].position - rig.Joints[1].position) * 3f;
            rig.AddPole(2, onLine);
            rig.Solver.MaxIterations = 0;
            rig.Target.position = rig.Joints[4].position;
            rig.Initialize();
            Pose[] before = rig.WorldPoses();

            rig.Solver.Solve();
            AssertPosesNear(rig.SolvedPoses(), before);

            using var straight = new Rig(new[] { Vector3.zero, Vector3.up, Vector3.up, Vector3.up }, SolverKind.FABRIK);
            straight.AddPole(1, straight.Joints[1].position + Vector3.right);
            straight.Solver.MaxIterations = 0;
            straight.Target.position = straight.Joints[3].position;
            straight.Initialize();
            Pose[] straightBefore = straight.WorldPoses();

            straight.Solver.Solve();
            AssertPosesNear(straight.SolvedPoses(), straightBefore);
        }

        // ---------------------------------------------------------------------------------------
        // Solvers
        // ---------------------------------------------------------------------------------------

        [TestCase(SolverKind.FABRIK)]
        [TestCase(SolverKind.CCD)]
        [TestCase(SolverKind.Jacobian)]
        public void Solver_ReachesTheTargetWithTheBendFacingThePole(SolverKind kind)
        {
            using var rig = new Rig(new[] { Vector3.zero, new(0f, 1f, 0f), new(0.2f, 1f, 0f), new(-0.2f, 1f, 0f) }, kind);
            rig.AddPole(2, new Vector3(0f, 2f, 3f));
            rig.Target.position = new Vector3(0.5f, 2.4f, 0.4f);
            rig.Solver.MaxIterations = 100;
            rig.Initialize();

            rig.Solver.Solve();

            Pose[] solved = rig.SolvedPoses();
            Assert.That(Vector3.Distance(solved[3].position, rig.Target.position), Is.LessThan(0.01f));
            Assert.That(BendAngleToPole(solved, 1, 2, 3, rig.PoleTargets[0].position), Is.LessThan(0.5f));
        }

        [Test]
        public void MovingOnlyThePole_MovesTheBendJoint()
        {
            using var rig = new Rig(BentChain, SolverKind.FABRIK);
            rig.AddPole(2, new Vector3(0f, 2f, 2f));
            rig.Target.position = rig.Joints[4].position;
            rig.Initialize();
            for (int i = 0; i < 3; i++)
                rig.Solver.Step(0.02f);

            rig.PoleTargets[0].position = new Vector3(0f, 2f, -2f);
            rig.Solver.Step(0.02f);

            Assert.That(BendAngleToPole(rig.WorldPoses(), 1, 2, 3, rig.PoleTargets[0].position), Is.LessThan(AngleEpsilon));
        }

        [Test]
        public void FreeJoints_DoNotSpinAroundTheirBonesWhileTheTargetMoves()
        {
            Vector3[] offsets = { Vector3.zero, new(0.2f, 1f, 0f), new(-0.2f, 1f, 0f), new(0.2f, 1f, 0f), new(-0.2f, 1f, 0f), new(0f, 1f, 0f) };
            using var rig = new Rig(offsets, SolverKind.FABRIK);
            rig.AddPole(2, new Vector3(-3f, 2f, -3f));
            rig.AddPole(4, new Vector3(3f, 4f, -3f));
            Vector3 center = new(1.5f, 3.5f, 0.5f);
            rig.Target.position = center;
            rig.Initialize();
            rig.Solver.Step(1f / 60f);

            Transform bend = rig.Joints[2], child = rig.Joints[3];
            Vector3 bone = (child.position - bend.position).normalized;
            Vector3 reference = Vector3.ProjectOnPlane(bend.rotation * Vector3.right, bone).normalized;
            float largestSpin = 0f, totalSpin = 0f;
            for (int frame = 0; frame < 240; frame++)
            {
                float t = frame / 60f;
                rig.Target.position = center + new Vector3(Mathf.Cos(t * Mathf.PI), Mathf.Sin(t * Mathf.PI), 0f) * 1.2f;
                rig.Solver.Step(1f / 60f);

                // Carry the reference with the bone's smallest rotation; any remaining angle is spin.
                Vector3 newBone = (child.position - bend.position).normalized;
                reference = Vector3.ProjectOnPlane(Quaternion.FromToRotation(bone, newBone) * reference, newBone).normalized;
                Vector3 actual = Vector3.ProjectOnPlane(bend.rotation * Vector3.right, newBone).normalized;
                float spin = Vector3.SignedAngle(reference, actual, newBone);
                largestSpin = Mathf.Max(largestSpin, Mathf.Abs(spin));
                totalSpin += spin;
                reference = actual;
                bone = newBone;
            }

            // A rigid turn spun this joint by up to 165 degrees in one frame. FABRIK's own updates leave a
            // few degrees, because child joints inherit their parent's rotation.
            Assert.That(largestSpin, Is.LessThan(3f), "The pole turn spun the bend joint around its bone.");
            Assert.That(Mathf.Abs(totalSpin), Is.LessThan(10f), "Spin built up while the target moved.");
            Assert.That(BendAngleToPole(rig.SolvedPoses(), 1, 2, 3, rig.PoleTargets[0].position), Is.LessThan(AngleEpsilon));
        }

        // ---------------------------------------------------------------------------------------
        // Extended sections
        // ---------------------------------------------------------------------------------------

        [Test]
        public void HingeStart_ExtendsUpToABallSocket()
        {
            using var rig = Leg();
            var sections = Resolve(rig, 2, out List<string> problems);

            Assert.That(sections[0].Start, Is.EqualTo(0));
            Assert.That(sections[0].End, Is.EqualTo(3));
            Assert.That(problems, Is.Empty);
        }

        [Test]
        public void HingeEnd_ExtendsDownToAJointThatCanTwist()
        {
            using var rig = new Rig(BentChain, SolverKind.FABRIK);
            AddBallSocket(rig.Joints[0], 180f);
            AddHinge(rig.Joints[2]);
            var sections = Resolve(rig, 1, out List<string> problems);

            Assert.That(sections[0].Start, Is.EqualTo(0));
            Assert.That(sections[0].End, Is.EqualTo(3));
            Assert.That(problems, Is.Empty);
        }

        [Test]
        public void AllHingeChain_KeepsTheNeighbour()
        {
            using var rig = new Rig(BentChain, SolverKind.FABRIK);
            AddHinge(rig.Joints[0]);
            AddHinge(rig.Joints[1]);
            var sections = Resolve(rig, 2, out List<string> problems);

            Assert.That(sections[0].Start, Is.EqualTo(1));
            Assert.That(problems, Is.Empty, "A blocked section is not a setup mistake.");
        }

        [Test]
        public void CustomConstraint_StopsTheSearch()
        {
            using var rig = new Rig(BentChain, SolverKind.FABRIK);
            AddBallSocket(rig.Joints[0], 180f);
            rig.Joints[1].gameObject.AddComponent<CustomJoint>();
            AddHinge(rig.Joints[2]);
            var sections = Resolve(rig, 3, out List<string> problems);

            Assert.That(sections[0].Start, Is.EqualTo(2));
            Assert.That(problems, Is.Empty);
        }

        [Test]
        public void ExtendedSwivel_KeepsSkippedHingesAndTheSectionEnds()
        {
            using var rig = Leg();
            rig.AddPole(2, rig.Joints[2].position + new Vector3(2f, 0f, 0f));
            rig.Solver.MaxIterations = 0;
            rig.Target.position = rig.Joints[4].position;
            rig.Initialize();
            Pose[] before = rig.WorldPoses();

            rig.Solver.Solve();
            Pose[] after = rig.SolvedPoses();

            foreach (int i in new[] { 0, 3, 4 })
                AssertNear(after[i].position, before[i].position, $"J{i} position");
            foreach (int i in new[] { 1, 2 })
            {
                Quaternion localBefore = Quaternion.Inverse(before[i - 1].rotation) * before[i].rotation;
                Quaternion localAfter = Quaternion.Inverse(after[i - 1].rotation) * after[i].rotation;
                Assert.That(Quaternion.Angle(localBefore, localAfter), Is.LessThan(AngleEpsilon), $"J{i} local rotation");
            }

            Assert.That(BendAngleToPole(after, 0, 2, 3, rig.PoleTargets[0].position), Is.LessThan(AngleEpsilon));
        }

        [TestCase(false)]
        [TestCase(true)]
        public void DisablingABlockingHinge_ResolvesTheSectionAgain_UnlessStatic(bool staticConfiguration)
        {
            // Root (Ball Socket), Hip (Hinge), Knee, Ankle. The Hip hinge extends the Knee section to Root.
            using var rig = new Rig(new[] { Vector3.zero, new(0f, -0.2f, 0f), new(0f, -0.5f, 0.2f), new(0f, -0.5f, -0.2f) }, SolverKind.FABRIK);
            AddBallSocket(rig.Joints[0], 180f);
            HingeIKJoint hip = AddHinge(rig.Joints[1]);
            rig.AddPole(2, rig.Joints[2].position + new Vector3(2f, 0f, 0f));
            rig.Solver.MaxIterations = 0;
            rig.Solver.StaticSolverConfiguration = staticConfiguration;
            rig.Target.position = rig.Joints[3].position;
            rig.Initialize();
            Quaternion rootBefore = rig.Joints[0].rotation;

            hip.jointIsEnabled = false;
            rig.Solver.Solve();

            float rootTurn = Quaternion.Angle(rig.SolvedPoses()[0].rotation, rootBefore);
            if (staticConfiguration)
                Assert.That(rootTurn, Is.GreaterThan(1f), "The frozen section still starts at Root.");
            else
                Assert.That(rootTurn, Is.LessThan(AngleEpsilon), "The section now starts at the free Hip.");
        }

        // ---------------------------------------------------------------------------------------
        // Multiple poles
        // ---------------------------------------------------------------------------------------

        [Test]
        public void EntryOrder_DoesNotChangeTheResult()
        {
            Vector3[] offsets = { Vector3.zero, new(0.2f, 1f, 0f), new(-0.2f, 1f, 0f), new(0.2f, 1f, 0f), new(-0.2f, 1f, 0f) };
            using var forward = new Rig(offsets, SolverKind.FABRIK);
            using var reversed = new Rig(offsets, SolverKind.FABRIK);
            Vector3 pole1 = new(0f, 1f, 2f), pole3 = new(0f, 3f, -2f);
            forward.AddPole(1, pole1, 0.5f);
            forward.AddPole(3, pole3, 0.5f);
            reversed.AddPole(3, pole3, 0.5f);
            reversed.AddPole(1, pole1, 0.5f);
            foreach (Rig rig in new[] { forward, reversed })
            {
                rig.Solver.MaxIterations = 0;
                rig.Target.position = rig.Joints[4].position;
                rig.Initialize();
                rig.Solver.Solve();
            }

            AssertPosesNear(forward.SolvedPoses(), reversed.SolvedPoses());
        }

        [Test]
        public void NeighbouringPoles_BothSettleOverAFewFrames()
        {
            Vector3[] offsets = { Vector3.zero, new(0.3f, 1f, 0f), new(-0.3f, 1f, 0f), new(0.3f, 1f, 0f), new(0f, 0.5f, 0f) };
            using var rig = new Rig(offsets, SolverKind.FABRIK);
            rig.AddPole(1, new Vector3(0f, 1f, 2f));
            rig.AddPole(2, new Vector3(-1f, 2f, -2f));
            rig.Target.position = rig.Joints[4].position;

            rig.Initialize();
            for (int frame = 0; frame < 5; frame++)
                rig.Solver.Step(0.02f);

            Pose[] poses = rig.WorldPoses();
            Assert.That(BendAngleToPole(poses, 0, 1, 2, rig.PoleTargets[0].position), Is.LessThan(1f));
            Assert.That(BendAngleToPole(poses, 1, 2, 3, rig.PoleTargets[1].position), Is.LessThan(1f));
        }

        // ---------------------------------------------------------------------------------------
        // Setup and limits
        // ---------------------------------------------------------------------------------------

        [Test]
        public void SetupMistakes_AreLoggedOncePerSetupChange()
        {
            using var rig = new Rig(BentChain, SolverKind.FABRIK);
            rig.AddPole(0, new Vector3(0f, 0f, 2f));
            rig.Target.position = rig.Joints[4].position;

            LogAssert.Expect(LogType.Warning, new Regex("not a middle joint"));
            rig.Initialize();
            rig.Solver.Solve();
            rig.Solver.Solve();
            LogAssert.NoUnexpectedReceived();
        }

        [Test]
        public void ConstrainedChainWithPoles_LogsNothing()
        {
            using var rig = Leg();
            rig.AddPole(2, rig.Joints[2].position + new Vector3(2f, 0f, 0f));
            rig.Target.position = rig.Joints[4].position + new Vector3(0.3f, 0.4f, 0.2f);
            rig.Initialize();
            for (int frame = 0; frame < 5; frame++)
                rig.Solver.Step(0.02f);

            LogAssert.NoUnexpectedReceived();
        }

        [Test]
        public void StaticConfiguration_FreezesEntriesButReadsWeights()
        {
            using var rig = new Rig(BentChain, SolverKind.FABRIK);
            IKPole pole = rig.AddPole(2, new Vector3(0f, 2f, 2f), 0f);
            rig.Solver.MaxIterations = 0;
            rig.Solver.StaticSolverConfiguration = true;
            rig.Target.position = rig.Joints[4].position;
            rig.Initialize();
            Pose[] before = rig.WorldPoses();

            // An entry added after initialization is ignored.
            rig.Solver.AddPole(rig.Joints[1], rig.AddPoleTarget(new Vector3(0f, 1f, -3f)));
            rig.Solver.Solve();
            AssertPosesNear(rig.SolvedPoses(), before);

            // The existing entry's weight is goal data and is read every solve.
            pole.Weight = 1f;
            rig.Solver.Solve();
            Assert.That(BendAngleToPole(rig.SolvedPoses(), 1, 2, 3, pole.Target.position), Is.LessThan(AngleEpsilon));
        }

        [Test]
        public void AddedPole_AppliesOnTheNextSolveWithoutStaticConfiguration()
        {
            using var rig = new Rig(BentChain, SolverKind.FABRIK);
            rig.Solver.MaxIterations = 0;
            rig.Target.position = rig.Joints[4].position;
            rig.Initialize();
            Pose[] before = rig.WorldPoses();

            Transform poleTarget = rig.AddPoleTarget(new Vector3(0f, 2f, 2f));
            IKPole pole = rig.Solver.AddPole(rig.Joints[2], poleTarget);
            rig.Solver.Solve();
            Assert.That(BendAngleToPole(rig.SolvedPoses(), 1, 2, 3, poleTarget.position), Is.LessThan(AngleEpsilon));

            // Solve does not write transforms, so without the pole the next solve keeps the starting pose.
            Assert.That(rig.Solver.RemovePole(pole), Is.True);
            rig.Solver.Solve();
            AssertPosesNear(rig.SolvedPoses(), before);
        }

        [Test]
        public void TightJointLimits_MarkThePoleLimitedWithoutLogging()
        {
            using var rig = new Rig(BentChain, SolverKind.FABRIK);
            AddBallSocket(rig.Joints[1], 5f);
            IKPole pole = rig.AddPole(2, new Vector3(0f, 2f, 2f));
            rig.Solver.MaxIterations = 0;
            rig.Target.position = rig.Joints[4].position;
            rig.Initialize();

            Pose[] before = rig.WorldPoses();
            rig.Solver.Solve();
            Assert.That(pole.IsLimited, Is.True);

            // The limit shortens the turn instead of clamping afterward, so the end effector stays put.
            Pose[] after = rig.SolvedPoses();
            AssertNear(after[4].position, before[4].position, "end effector");
            Assert.That(BendAngleToPole(after, 1, 2, 3, pole.Target.position), Is.LessThan(89f), "The pole turned the bend partway.");

            rig.Solver.Solve();
            LogAssert.NoUnexpectedReceived();
        }

        [Test]
        public void SolvingWithPoles_DoesNotAllocateInSteadyState()
        {
            using var rig = new Rig(BentChain, SolverKind.FABRIK);
            rig.AddPole(2, new Vector3(0f, 2f, 2f), 0.5f);
            rig.Target.position = new Vector3(0.4f, 2.6f, 0.3f);
            rig.Initialize();
            rig.Solver.Solve();
            rig.Solver.Solve();

            Assert.That(() => { rig.Solver.Solve(); },
                UnityEngine.TestTools.Constraints.ConstraintExtensions.AllocatingGCMemory(Is.Not));
        }

        // ---------------------------------------------------------------------------------------
        // Fixture
        // ---------------------------------------------------------------------------------------

        // Hip (Ball Socket), Thigh (Hinge), Knee (Hinge), Ankle, Toe, bent forward at the knee.
        private static Rig Leg()
        {
            var rig = new Rig(new[] { Vector3.zero, new(0f, -0.2f, 0f), new(0f, -0.5f, 0.2f), new(0f, -0.5f, -0.2f), new(0f, -0.1f, 0.2f) },
                SolverKind.FABRIK);
            AddBallSocket(rig.Joints[0], 180f);
            AddHinge(rig.Joints[1]);
            AddHinge(rig.Joints[2]);
            return rig;
        }

        private static List<PoleSection> Resolve(Rig rig, int bendIndex, out List<string> problems)
        {
            var sections = new List<PoleSection>();
            problems = new List<string>();
            PoleSectionResolver.Resolve(rig.Joints, new[] { new IKPole(rig.Joints[bendIndex], null) }, sections, problems);
            return sections;
        }

        private static HingeIKJoint AddHinge(Transform joint)
        {
            var hinge = joint.gameObject.AddComponent<HingeIKJoint>();
            hinge.hingeAxis = Vector3.right;
            hinge.hingeMinAngle = -170f;
            hinge.hingeMaxAngle = 170f;
            return hinge;
        }

        private static void AddBallSocket(Transform joint, float limit)
        {
            var ball = joint.gameObject.AddComponent<BallSocketIKJoint>();
            ball.swingPitchHalfAngle = limit;
            ball.swingYawHalfAngle = limit;
            ball.twistHalfAngle = limit;
        }

        // The angle around the start-to-end line between the bend joint's direction and the pole's.
        private static float BendAngleToPole(Pose[] poses, int start, int bend, int end, Vector3 pole)
        {
            Vector3 axis = (poses[end].position - poses[start].position).normalized;
            Vector3 bendDirection = Vector3.ProjectOnPlane(poses[bend].position - poses[start].position, axis);
            Vector3 poleDirection = Vector3.ProjectOnPlane(pole - poses[start].position, axis);
            return Vector3.Angle(bendDirection, poleDirection);
        }

        private static void AssertNear(Vector3 actual, Vector3 expected, string message)
        {
            Assert.That(Vector3.Distance(actual, expected), Is.LessThan(PositionEpsilon), $"{message}: {actual} vs {expected}");
        }

        private static void AssertPosesNear(Pose[] actual, Pose[] expected)
        {
            for (int i = 0; i < expected.Length; i++)
            {
                AssertNear(actual[i].position, expected[i].position, $"J{i} position");
                Assert.That(Quaternion.Angle(actual[i].rotation, expected[i].rotation), Is.LessThan(AngleEpsilon), $"J{i} rotation");
            }
        }

        private class CustomJoint : ConstrainedJoint
        {
            public override void UpdateConstraints() { }
        }

        private sealed class Rig : IDisposable
        {
            private readonly List<GameObject> _extra = new();

            public Rig(Vector3[] offsets, SolverKind kind, int orientationMode = 0)
            {
                Root = new GameObject("Rig");
                Transform parent = Root.transform;
                for (int i = 0; i < offsets.Length; i++)
                {
                    Transform joint = new GameObject($"J{i}").transform;
                    joint.SetParent(parent, false);
                    joint.localPosition = offsets[i];
                    Joints.Add(joint);
                    parent = joint;
                }

                Target = new GameObject("Target").transform;
                Solver = kind switch
                {
                    SolverKind.FABRIK => Root.AddComponent<FABRIKSolver>(),
                    SolverKind.CCD => Root.AddComponent<CCDIKSolver>(),
                    _ => Root.AddComponent<JacobianIKSolver>()
                };
                Solver.Target = Target;
                SetField(Solver, "chainJoints", new List<Transform>(Joints));
                if (kind == SolverKind.Jacobian)
                {
                    SetField(Solver, "orientationMode", orientationMode);
                    SetField(Solver, "damping", 0.05f);
                }
            }

            public GameObject Root { get; }
            public Transform Target { get; }
            public SingleChainSolverBase Solver { get; }
            public List<Transform> Joints { get; } = new();
            public List<Transform> PoleTargets { get; } = new();

            public IKPole AddPole(int bendIndex, Vector3 polePosition, float weight = 1f)
            {
                return Solver.AddPole(Joints[bendIndex], AddPoleTarget(polePosition), weight);
            }

            public Transform AddPoleTarget(Vector3 position)
            {
                Transform pole = new GameObject("Pole").transform;
                pole.position = position;
                _extra.Add(pole.gameObject);
                PoleTargets.Add(pole);
                return pole;
            }

            public void Initialize()
            {
                MethodInfo awake = typeof(OpenIKSolverBase).GetMethod("Awake", BindingFlags.Instance | BindingFlags.NonPublic);
                awake.Invoke(Solver, null);
            }

            public Pose[] WorldPoses()
            {
                var poses = new Pose[Joints.Count];
                for (int i = 0; i < poses.Length; i++)
                    poses[i] = new Pose(Joints[i].position, Joints[i].rotation);
                return poses;
            }

            public Pose[] SolvedPoses()
            {
                IKSolverOutput output = Solver.LastSolution;
                var poses = new Pose[output.JointCount];
                for (int i = 0; i < poses.Length; i++)
                    poses[i] = new Pose(output.WorldPositions[i], Quaternion.Normalize(output.WorldRotations[i]));
                return poses;
            }

            public void Dispose()
            {
                foreach (GameObject extra in _extra)
                    Object.DestroyImmediate(extra);
                Object.DestroyImmediate(Target.gameObject);
                Object.DestroyImmediate(Root);
            }

            private static void SetField(object instance, string fieldName, object value)
            {
                for (Type type = instance.GetType(); type != null; type = type.BaseType)
                {
                    FieldInfo field = type.GetField(fieldName, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.DeclaredOnly);
                    if (field == null)
                        continue;

                    field.SetValue(instance, field.FieldType.IsEnum ? Enum.ToObject(field.FieldType, value) : value);
                    return;
                }

                Assert.Fail($"Expected field '{fieldName}' on {instance.GetType().Name}.");
            }
        }
    }
}
