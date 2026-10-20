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
    public class TwoBoneSolverTests
    {
        private const float PositionEpsilon = 1e-4f;
        private const float AngleEpsilon = 0.01f;

        // An arm hanging along -Y with the elbow bent slightly forward (+Z).
        private static readonly Vector3[] BentArm = { Vector3.zero, new(0f, -1f, 0.1f), new(0f, -1f, -0.1f) };

        [Test]
        public void RequiresExactlyThreeJoints()
        {
            using var limb = new Limb(new[] { Vector3.zero, Vector3.down, Vector3.down, Vector3.down });

            LogAssert.Expect(LogType.Error, new Regex("exactly 3 chain joints"));
            Assert.That(limb.Solver.Solve(), Is.False);
        }

        [Test]
        public void ReachableTarget_IsReachedInOneStepWithBoneLengthsKept()
        {
            using var limb = new Limb(BentArm);
            Pose[] before = limb.WorldPoses();
            limb.Target.position = new Vector3(0.5f, -1.2f, 0.6f);
            limb.Initialize();

            limb.Solver.Solve();

            IKSolverOutput output = limb.Solver.LastSolution;
            Assert.That(output.IterationsUsed, Is.EqualTo(1));
            Assert.That(output.FinalError, Is.LessThan(PositionEpsilon));
            Pose[] solved = limb.SolvedPoses();
            for (int i = 1; i < 3; i++)
            {
                Assert.That(Vector3.Distance(solved[i].position, solved[i - 1].position),
                    Is.EqualTo(Vector3.Distance(before[i].position, before[i - 1].position)).Within(PositionEpsilon));
            }
        }

        [Test]
        public void UnreachableTarget_StraightensTowardIt()
        {
            using var limb = new Limb(BentArm);
            limb.Target.position = new Vector3(1f, -5f, 0.5f);
            limb.Initialize();

            limb.Solver.Solve();

            Pose[] solved = limb.SolvedPoses();
            Vector3 direction = (limb.Target.position - solved[0].position).normalized;
            Assert.That(Vector3.ProjectOnPlane(solved[1].position - solved[0].position, direction).magnitude, Is.LessThan(PositionEpsilon));
            Assert.That(Vector3.Angle(solved[2].position - solved[0].position, direction), Is.LessThan(AngleEpsilon));
        }

        [Test]
        public void WithoutPole_KeepsTheCurrentBendSide()
        {
            using var limb = new Limb(BentArm);
            limb.Target.position = new Vector3(0f, -1.5f, 0f);
            limb.Initialize();

            limb.Solver.Solve();

            Assert.That(limb.SolvedPoses()[1].position.z, Is.GreaterThan(0.1f));
        }

        [Test]
        public void Pole_TurnsTheMiddleJointTowardIt()
        {
            using var limb = new Limb(BentArm);
            limb.Target.position = new Vector3(0.2f, -1.5f, 0.3f);
            Transform pole = limb.AddPole(new Vector3(2f, -1f, 0f));
            limb.Initialize();

            limb.Solver.Solve();

            Pose[] solved = limb.SolvedPoses();
            Vector3 axis = (solved[2].position - solved[0].position).normalized;
            float angle = Vector3.Angle(
                Vector3.ProjectOnPlane(solved[1].position - solved[0].position, axis),
                Vector3.ProjectOnPlane(pole.position - solved[0].position, axis));
            Assert.That(angle, Is.LessThan(AngleEpsilon));
            Assert.That(limb.Solver.LastSolution.FinalError, Is.LessThan(PositionEpsilon));
        }

        [Test]
        public void HingeElbow_OnlyBendsAroundItsAxis()
        {
            // Bent forward around the X axis, so the hinge axis is the bend plane's normal.
            using var limb = new Limb(BentArm);
            AddHinge(limb.Joints[1], -170f, 170f);
            limb.Target.position = new Vector3(0.6f, -1.1f, 0.5f);
            limb.Initialize();
            Quaternion restLocal = Quaternion.Inverse(limb.Joints[0].rotation) * limb.Joints[1].rotation;

            limb.Solver.Solve();

            Pose[] solved = limb.SolvedPoses();
            Quaternion local = Quaternion.Inverse(solved[0].rotation) * solved[1].rotation;
            Quaternion bend = Quaternion.Inverse(restLocal) * local;
            Assert.That(Mathf.Abs(bend.y) + Mathf.Abs(bend.z), Is.LessThan(1e-4f), "The elbow turned around another axis.");
            Assert.That(limb.Solver.LastSolution.FinalError, Is.LessThan(PositionEpsilon));
        }

        [Test]
        public void OverextendedLimb_BendsTowardItsRestSide()
        {
            using var limb = new Limb(BentArm);
            limb.Initialize();
            // Overextend the elbow, as a retargeted animation can, so it sits just behind the shoulder-wrist line.
            limb.Joints[1].localRotation = Quaternion.Euler(-20f, 0f, 0f);
            Vector3 wrist = limb.Joints[2].position;
            Assert.That(Vector3.Dot(Vector3.ProjectOnPlane(limb.Joints[1].position, wrist.normalized), Vector3.forward), Is.LessThan(0f));
            limb.Target.position = new Vector3(0f, -1.5f, 0.2f);

            limb.Solver.Solve();

            Pose[] solved = limb.SolvedPoses();
            Vector3 reach = (solved[2].position - solved[0].position).normalized;
            Assert.That(Vector3.Dot(Vector3.ProjectOnPlane(solved[1].position - solved[0].position, reach), Vector3.forward), Is.GreaterThan(0.1f));
            Assert.That(limb.Solver.LastSolution.FinalError, Is.LessThan(PositionEpsilon));
            // The elbow folds back through straight around its hinge; the upper arm does not twist half a turn.
            Assert.That(Vector3.Dot(solved[0].rotation * Vector3.right, Vector3.right), Is.GreaterThan(0.9f));
            Assert.That(Vector3.Dot(solved[1].rotation * Vector3.right, Vector3.right), Is.GreaterThan(0.9f));
        }

        [Test]
        public void StraightLimbWithoutPole_UsesAFinitePerpendicularFallback()
        {
            using var limb = new Limb(new[] { Vector3.zero, Vector3.down, Vector3.down });
            limb.Initialize();
            limb.Target.position = new Vector3(0f, -1.5f, 0f);

            limb.Solver.Solve();

            Assert.That(limb.SolvedPoses()[1].position.z, Is.GreaterThan(0.5f));
            Assert.That(limb.Solver.LastSolution.FinalError, Is.LessThan(PositionEpsilon));
        }

        [TestCase(false)]
        [TestCase(true)]
        public void Pole_ControlsAStraightRestPoseWithoutFlippingTheThigh(bool overextended)
        {
            using var limb = new Limb(new[] { Vector3.zero, Vector3.down, Vector3.down });
            limb.AddPole(new Vector3(0f, -0.5f, 2f));
            limb.Initialize();
            if (overextended) limb.Joints[1].localRotation = Quaternion.Euler(-20f, 0f, 0f);
            limb.Target.position = new Vector3(0f, -1.5f, 0.2f);

            limb.Solver.Solve();

            Pose[] solved = limb.SolvedPoses();
            Assert.That(solved[1].position.z, Is.GreaterThan(0.5f));
            Assert.That(Vector3.Dot(solved[0].rotation * Vector3.right, Vector3.right), Is.GreaterThan(0.9f));
            Assert.That(Vector3.Dot(solved[1].rotation * Vector3.right, Vector3.right), Is.GreaterThan(0.9f));
            Assert.That(limb.Solver.LastSolution.FinalError, Is.LessThan(PositionEpsilon));
        }

        [Test]
        public void StraightLimbWithHinge_BendsToTheHingesAllowedSide()
        {
            using var limb = new Limb(new[] { Vector3.zero, Vector3.down, Vector3.down });
            AddHinge(limb.Joints[1], 0f, 150f);
            limb.Target.position = new Vector3(0f, -1.5f, 0f);
            limb.Initialize();

            limb.Solver.Solve();

            Assert.That(limb.Solver.LastSolution.FinalError, Is.LessThan(1e-3f), "The bend went to the side the hinge does not allow.");
        }

        [Test]
        public void MatchTarget_TurnsTheEndJointWithTheTarget()
        {
            using var limb = new Limb(BentArm);
            limb.Solver.EndRotation = TwoBoneEndRotation.MatchTarget;
            limb.Target.position = new Vector3(0.3f, -1.4f, 0.4f);
            limb.Initialize();

            limb.Target.rotation = Quaternion.Euler(20f, 30f, 10f);
            limb.Solver.Solve();

            Assert.That(Quaternion.Angle(limb.SolvedPoses()[2].rotation, limb.Target.rotation), Is.LessThan(AngleEpsilon));
        }

        [Test]
        public void MatchTarget_TwistsABallSocketEndJointWithinItsLimit()
        {
            using var limb = new Limb(BentArm);
            var ankle = limb.Joints[2].gameObject.AddComponent<BallSocketIKJoint>();
            ankle.constraintAxis = Vector3.down;
            ankle.swingPitchHalfAngle = 60f;
            ankle.swingYawHalfAngle = 60f;
            ankle.twistHalfAngle = 40f;
            limb.Solver.EndRotation = TwoBoneEndRotation.MatchTarget;
            limb.Target.position = new Vector3(0f, -1.8f, 0f);
            limb.Initialize();

            // A turn around the end joint's own down axis is pure twist.
            limb.Target.rotation = Quaternion.AngleAxis(25f, Vector3.down);
            limb.Solver.Solve();
            Assert.That(Quaternion.Angle(limb.SolvedPoses()[2].rotation, limb.Target.rotation), Is.LessThan(0.1f));

            // Past the limit, the twist stops at it. The twist is measured relative to the middle joint.
            limb.Target.rotation = Quaternion.AngleAxis(70f, Vector3.down);
            limb.Solver.Solve();
            Quaternion local = Quaternion.Inverse(limb.SolvedPoses()[1].rotation) * limb.Target.rotation;
            float twist = Mathf.Abs(HingeIKJoint.ExtractHingeAngle(local, Vector3.down));
            Assert.That(twist, Is.GreaterThan(40f));
            Assert.That(Quaternion.Angle(limb.SolvedPoses()[2].rotation, limb.Target.rotation), Is.EqualTo(twist - 40f).Within(0.5f));
        }

        [TestCase(0f)]
        [TestCase(45f)]
        [TestCase(90f)]
        public void AlignUp_AlignsLocalUpRegardlessOfInitialTilt(float initialTilt)
        {
            using var limb = new Limb(BentArm);
            limb.Joints[2].localRotation = Quaternion.Euler(initialTilt, 20f, 0f);
            limb.Solver.EndRotation = TwoBoneEndRotation.AlignUp;
            limb.Target.position = new Vector3(0.3f, -1.4f, 0.4f);
            limb.Initialize();

            limb.Target.rotation = Quaternion.Euler(25f, 0f, -15f);
            limb.Solver.Solve();

            Vector3 endUp = limb.SolvedPoses()[2].rotation * Vector3.up;
            Assert.That(Vector3.Angle(endUp, limb.Target.up), Is.LessThan(AngleEpsilon));

            Quaternion solvedRotation = limb.SolvedPoses()[2].rotation;
            limb.Target.rotation *= Quaternion.Euler(0f, 70f, 0f);
            limb.Solver.Solve();
            Assert.That(Quaternion.Angle(limb.SolvedPoses()[2].rotation, solvedRotation), Is.LessThan(AngleEpsilon),
                "Turning the target around its up axis should not turn the end joint.");

            Assert.That(limb.Solver.SnapToSolution(), Is.True);
            Assert.That(Vector3.Angle(limb.Joints[2].up, limb.Target.up), Is.LessThan(AngleEpsilon));
            Assert.That(limb.Solver.ApplicationStatus.OrientationError, Is.LessThan(AngleEpsilon));
        }

        [Test]
        public void Softness_EasesTheReachWithoutJumps()
        {
            using var limb = new Limb(BentArm);
            limb.Solver.Softness = 0.2f;
            limb.Initialize();
            float maxReach = 2f * new Vector3(0f, -1f, 0.1f).magnitude;

            float previous = 0f;
            for (float distance = 1f; distance < 3f; distance += 0.01f)
            {
                limb.Target.position = new Vector3(0f, -distance, 0f);
                limb.Solver.Solve();
                float reach = Vector3.Distance(limb.SolvedPoses()[2].position, limb.SolvedPoses()[0].position);

                Assert.That(reach, Is.LessThan(maxReach));
                if (previous > 0f)
                    Assert.That(reach - previous, Is.InRange(-PositionEpsilon, 0.0101f), $"Reach jumped at distance {distance}.");
                previous = reach;
            }
        }

        private static void AddHinge(Transform joint, float min, float max)
        {
            var hinge = joint.gameObject.AddComponent<HingeIKJoint>();
            hinge.hingeAxis = Vector3.right;
            hinge.hingeMinAngle = min;
            hinge.hingeMaxAngle = max;
        }

        private sealed class Limb : IDisposable
        {
            private readonly List<GameObject> _extra = new();

            public Limb(Vector3[] offsets)
            {
                Root = new GameObject("Limb");
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
                Solver = Root.AddComponent<TwoBoneIKSolver>();
                Solver.Target = Target;
                typeof(SingleChainSolverBase).GetField("chainJoints", BindingFlags.Instance | BindingFlags.NonPublic)
                    .SetValue(Solver, new List<Transform>(Joints));
            }

            public GameObject Root { get; }
            public Transform Target { get; }
            public TwoBoneIKSolver Solver { get; }
            public List<Transform> Joints { get; } = new();

            public Transform AddPole(Vector3 position)
            {
                Transform pole = new GameObject("Pole").transform;
                pole.position = position;
                _extra.Add(pole.gameObject);
                Solver.AddPole(Joints[1], pole);
                return pole;
            }

            public void Initialize()
            {
                typeof(OpenIKSolverBase).GetMethod("Awake", BindingFlags.Instance | BindingFlags.NonPublic).Invoke(Solver, null);
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
        }
    }
}
