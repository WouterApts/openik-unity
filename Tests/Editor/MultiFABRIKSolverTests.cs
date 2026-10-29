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
    public partial class MultiFABRIKSolverTests
    {
        private const float Epsilon = 1e-5f;
        private static readonly int[] TorsoParents = { -1, 0, 1, 2, 3, 4, 2, 6, 7 };
        private static readonly Vector3[] TorsoOffsets =
        {
            Vector3.zero, new(0f, 0.5f, 0f), new(0f, 0.5f, 0f),
            new(-0.25f, 0f, 0f), new(-0.4f, 0f, 0.15f), new(-0.4f, 0f, -0.15f),
            new(0.25f, 0f, 0f), new(0.4f, 0f, 0.15f), new(0.4f, 0f, -0.15f)
        };

        [TestCase(false)]
        [TestCase(true)]
        public void OneChain_MatchesPlainFABRIK(bool constrained)
        {
            var offsets = new[] { Vector3.zero, new Vector3(0f, 0.5f, 0.1f), new Vector3(0f, 0.5f, -0.1f), Vector3.up * 0.3f };
            using var multi = new Rig(offsets, new[] { -1, 0, 1, 2 });
            using var plain = new Rig(offsets, new[] { -1, 0, 1, 2 });
            if (constrained)
            {
                // The root hinge covers the root limit correction at the end of the forward pass.
                AddHinge(multi.Joints[0]);
                AddHinge(plain.Joints[0]);
                AddHinge(multi.Joints[1]);
                AddHinge(plain.Joints[1]);
            }
            Transform target = multi.Target(new Vector3(0.2f, 0.7f, 0.5f));
            multi.Add("Arm", target, 0, 1, 2, 3);
            var fabrik = plain.Root.AddComponent<FABRIKSolver>();
            fabrik.Target = target;
            fabrik.MaxIterations = multi.Solver.MaxIterations = 30;
            typeof(SingleChainSolverBase).GetField("chainJoints", BindingFlags.Instance | BindingFlags.NonPublic)
                .SetValue(fabrik, new List<Transform>(plain.Joints));
            Assert.That(fabrik.Solve(), Is.True);
            Assert.That(multi.Solver.Solve(), Is.True);
            for (int i = 0; i < offsets.Length; i++)
            {
                Near(multi.Position(i), fabrik.LastSolution.WorldPositions[i], $"joint {i}");
                RotationNear(multi.Rotation(i), fabrik.LastSolution.WorldRotations[i]);
            }
            Assert.That(multi.Solver.LastSolution.IterationsUsed, Is.EqualTo(fabrik.LastSolution.IterationsUsed));
        }

        [Test]
        public void TrunkAndOneArm_MatchAJoinedChain()
        {
            var offsets = new[] { Vector3.zero, Vector3.up * 0.5f, new Vector3(0.1f, 0.4f, 0f), new Vector3(0.4f, 0.1f, 0f), Vector3.right * 0.4f };
            using var split = new Rig(offsets, new[] { -1, 0, 1, 2, 3 });
            using var joined = new Rig(offsets, new[] { -1, 0, 1, 2, 3 });
            var target = split.Target(new Vector3(0.6f, 0.7f, 0.3f));
            split.Add("Trunk", null, 0, 1, 2);
            split.Add("Arm", target, 2, 3, 4);
            joined.Add("Joined", target, 0, 1, 2, 3, 4);
            split.Solver.Solve();
            joined.Solver.Solve();
            for (int i = 0; i < offsets.Length; i++)
            {
                Near(split.Position(i), joined.Position(i));
                RotationNear(split.Rotation(i), joined.Rotation(i));
            }
        }

        [Test]
        public void Torso_ReachesBothHandsAndKeepsShoulderOffsetsRigid()
        {
            using var rig = Torso();
            rig.Solver.Solve();
            AssertConverged(rig, 1, 2);
            AssertRigid(rig);
            Assert.That(Vector3.Distance(rig.Position(3), rig.Position(6)), Is.EqualTo(0.5f).Within(Epsilon));
            for (int i = 0; i < rig.Joints.Length; i++) Near(rig.Joints[i].position, rig.InitialPositions[i], "Solve wrote transforms");
            rig.Solver.SnapToSolution();
            for (int i = 0; i < rig.Joints.Length; i++) Near(rig.Joints[i].position, rig.Position(i));
        }

        [TestCase(0f)]
        [TestCase(0.5f)]
        [TestCase(1f)]
        public void RootPin_BlendsPositionAndMultipleChainRotation(float pin)
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.left, Vector3.right, Vector3.up }, new[] { -1, 0, 0, 0 });
            Quaternion desired = Quaternion.Euler(0f, 0f, 25f);
            Vector3 translation = new(0.3f, 0.2f, 0f);
            for (int i = 1; i < 4; i++) rig.Add($"Limb {i}", rig.Target(translation + desired * rig.Joints[i].position), 0, i);
            rig.Solver.RootPin = pin;
            rig.Solver.MaxIterations = 1;
            rig.Solver.Solve();
            Near(rig.Position(0), translation * (1f - pin));
            float angle = Vector3.Angle(Vector3.right, rig.Rotation(0) * Vector3.right);
            if (pin == 1f) Assert.That(angle, Is.LessThan(0.001f));
            else Assert.That(angle, Is.InRange(5f, 30f));
            AssertRigid(rig);
        }

        [Test]
        public void DisabledFirstChain_DoesNotOwnOrPinTheRoot()
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.left, Vector3.right }, new[] { -1, 0, 0 });
            var first = rig.Add("First", rig.Target(Vector3.left), 0, 1);
            first.Enabled = false;
            rig.Add("Second", rig.Target(new Vector3(2f, 1f, 0f)), 0, 2);
            rig.Solver.RootPin = 0f;
            rig.Solver.Solve();
            Assert.That(rig.Position(0).magnitude, Is.GreaterThan(0.5f));
            AssertConverged(rig, 1);
            Near(rig.Position(1), rig.Position(0) + rig.Rotation(0) * Vector3.left);
            Assert.That(rig.Solver.Chains[0].LastResult.HasTarget, Is.False);
        }

        [Test]
        public void FloatingRoot_CanReachBeyondItsInitialPathLength()
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.up, Vector3.up }, new[] { -1, 0, 1 });
            rig.Add("Arm", rig.Target(new Vector3(5f, 4f, 0f)), 0, 1, 2);
            rig.Solver.RootPin = 0f;
            rig.Solver.Solve();
            AssertConverged(rig, 0);
        }

        [TestCase(0f, 0f)]
        [TestCase(1f, 1f)]
        public void PullZeroOrPinOne_KeepTheTrunkOwnPose(float pull, float pin)
        {
            using var rig = Torso();
            rig.Solver.SetPin(rig.Solver.Chains[0], pin);
            rig.Solver.SetPull(rig.Solver.Chains[1], pull);
            rig.Solver.SetPull(rig.Solver.Chains[2], pull);
            rig.Solver.Solve();
            for (int i = 0; i <= 2; i++) Near(rig.Position(i), rig.InitialPositions[i]);
            RotationNear(rig.Rotation(2), Quaternion.identity);
        }

        [Test]
        public void PinOne_KeepsATargetedRigidBranchPointOnItsTargetWhileBranchLimitsClamp()
        {
            // The base is locked, so hub can only swing on a unit sphere around J1. The arms reach far
            // above it, and their tight shoulder limits clamp during the forward pass.
            using var rig = new Rig(new[] { Vector3.zero, Vector3.up, Vector3.up, Vector3.left, Vector3.left, Vector3.right, Vector3.right },
                new[] { -1, 0, 1, 2, 3, 2, 5 });
            AddBall(rig.Joints[0], 0f);
            AddBall(rig.Joints[3], 10f);
            AddBall(rig.Joints[5], 10f);
            var goal = new Vector3(0.6f, 1.8f, 0f);
            rig.Solver.SetPin(rig.Add("Trunk", rig.Target(goal), 0, 1, 2), 1f);
            rig.Add("Left", rig.Target(new Vector3(-2f, 4f, 0f)), 2, 3, 4);
            rig.Add("Right", rig.Target(new Vector3(2f, 4f, 0f)), 2, 5, 6);
            rig.Solver.Solve();
            Near(rig.Position(2), goal);
            AssertRigid(rig);
        }

        [TestCase(1f, 1f, 0f)]
        [TestCase(1f, 0.25f, -0.6f)]
        public void ConflictingTargets_UsePullWeightedCompromiseAndStopEarly(float leftPull, float rightPull, float expectedX)
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.left, Vector3.right }, new[] { -1, 0, 0 });
            rig.Solver.SetPull(rig.Add("Left", rig.Target(Vector3.left * 2f), 0, 1), leftPull);
            rig.Solver.SetPull(rig.Add("Right", rig.Target(Vector3.right * 2f), 0, 2), rightPull);
            rig.Solver.RootPin = 0f;
            rig.Solver.Solve();
            Assert.That(rig.Position(0).x, Is.EqualTo(expectedX).Within(Epsilon));
            Assert.That(rig.Solver.LastSolution.Converged, Is.False);
            Assert.That(rig.Solver.LastSolution.IterationsUsed, Is.LessThan(rig.Solver.MaxIterations));
            AssertRigid(rig);
        }

        [Test]
        public void FreeArm_KeepsIncomingLocalPoseWhileChestMoves()
        {
            using var rig = Torso();
            rig.Solver.Chains[2].Target = null;
            rig.Joints[6].localRotation = Quaternion.Euler(10f, -15f, 5f);
            rig.Solver.Solve();
            Assert.That(Vector3.Distance(rig.Position(2), rig.InitialPositions[2]), Is.GreaterThan(0.01f));
            RotationNear(Quaternion.Inverse(rig.Rotation(2)) * rig.Rotation(6), rig.Joints[6].localRotation);
            RotationNear(Quaternion.Inverse(rig.Rotation(6)) * rig.Rotation(7), rig.Joints[7].localRotation);
            Assert.That(rig.Solver.Chains[2].LastResult.HasTarget, Is.False);
        }

        [Test]
        public void MidChainBranches_HoldTheFreeNeckAndHead()
        {
            var offsets = new[] { Vector3.zero, Vector3.up * 0.5f, Vector3.up * 0.5f, Vector3.up * 0.2f, Vector3.up * 0.2f,
                new Vector3(-0.3f, 0f, 0f), new Vector3(-0.4f, 0f, 0.1f), new Vector3(-0.4f, 0f, -0.1f),
                new Vector3(0.3f, 0f, 0f), new Vector3(0.4f, 0f, 0.1f), new Vector3(0.4f, 0f, -0.1f) };
            using var rig = new Rig(offsets, new[] { -1, 0, 1, 2, 3, 2, 5, 6, 2, 8, 9 });
            rig.Add("Spine to head", null, 0, 1, 2, 3, 4);
            rig.Add("Left", rig.Target(new Vector3(-0.75f, 1.1f, 0.3f)), 2, 5, 6, 7);
            rig.Add("Right", rig.Target(new Vector3(0.75f, 0.9f, 0.2f)), 2, 8, 9, 10);
            rig.Solver.Solve();
            AssertConverged(rig, 1, 2);
            RotationNear(Quaternion.Inverse(rig.Rotation(2)) * rig.Rotation(3), Quaternion.identity);
            RotationNear(Quaternion.Inverse(rig.Rotation(3)) * rig.Rotation(4), Quaternion.identity);
            AssertRigid(rig);
        }

        [Test]
        public void DisabledTrunk_HoldsItsPoseWhileItsArmsSolve()
        {
            using var rig = Torso();
            rig.Solver.Chains[0].Enabled = false;
            rig.Solver.Solve();
            AssertConverged(rig, 1, 2);
            for (int i = 0; i <= 2; i++)
            {
                Near(rig.Position(i), rig.InitialPositions[i]);
                RotationNear(rig.Rotation(i), Quaternion.identity);
            }
        }

        [Test]
        public void RuntimeAddRemove_PreservesRestPoseAndRejectsInvalidRemoval()
        {
            using var rig = Torso();
            AddHinge(rig.Joints[1]);
            rig.Solver.Solve();
            var restRotations = new Quaternion[rig.Joints.Length];
            var restOffsets = new Vector3[rig.Joints.Length];
            for (int i = 0; i < rig.Joints.Length; i++)
            {
                restRotations[i] = rig.Solver.Chain.Joints[i].RestLocalRotation;
                restOffsets[i] = rig.Solver.Chain.Joints[i].RestLocalOffset;
            }
            rig.Solver.SnapToSolution();
            MultiFABRIKChain removed = rig.Solver.Chains[2];
            Assert.That(rig.Solver.RemoveChain(removed), Is.True);
            for (int i = 0; i < rig.Joints.Length; i++) Near(rig.Joints[i].position, rig.InitialPositions[i]);
            rig.Add("Right again", removed.Target, 2, 6, 7, 8);
            for (int i = 0; i < rig.Joints.Length; i++)
            {
                Near(rig.Solver.Chain.Joints[i].RestLocalOffset, restOffsets[i]);
                RotationNear(rig.Solver.Chain.Joints[i].RestLocalRotation, restRotations[i]);
            }
            Assert.Throws<InvalidOperationException>(() => rig.Solver.RemoveChain(rig.Solver.Chains[0]));
            Assert.That(rig.Solver.Chains.Count, Is.EqualTo(3));
            rig.Solver.Solve();
            for (int i = 0; i < rig.Joints.Length; i++) Near(rig.Joints[i].position, rig.InitialPositions[i], "Solve wrote transforms after rebuild");
        }

        [Test]
        public void TreeValidation_SortsAndReportsSharingShortChainsAndHierarchy()
        {
            using var rig = Torso();
            var tree = new MultiFABRIKTree();
            var problems = new List<string>();
            var trunk = rig.Solver.Chains[0];
            var arm = rig.Solver.Chains[1];
            var sorted = new List<MultiFABRIKChain> { arm, trunk };
            Assert.That(tree.Build(sorted, problems), Is.True, string.Join("\n", problems));
            Assert.That(sorted, Is.EqualTo(new[] { trunk, arm }));
            var shared = rig.Add("Bad share", arm.Target, 2, 3);
            Assert.That(tree.Build(new List<MultiFABRIKChain> { trunk, arm, shared }, problems), Is.False);
            Assert.That(string.Join(" ", problems), Does.Contain("only the joint a chain starts from"));
            var shortChain = rig.Add("Short", arm.Target, 0);
            Assert.That(tree.Build(new List<MultiFABRIKChain> { shortChain }, problems), Is.False);
            Assert.That(problems[0], Does.Contain("Short").And.Contain("at least one joint"));
            var backwards = rig.Add("Backwards", arm.Target, 3, 2);
            Assert.That(tree.Build(new List<MultiFABRIKChain> { backwards }, problems), Is.False);
            Assert.That(problems[0], Does.Contain("hierarchy"));
        }

        [Test]
        public void Step_AllocatesNothingAfterWarmup()
        {
            using var rig = Torso();
            rig.Solver.SolveFromRestPose = true;
            rig.Solver.Chains[1].AddPole(rig.Joints[4], rig.Target(new Vector3(-0.5f, 1f, 2f)));
            for (int i = 0; i < 10; i++) rig.Solver.Step(1f / 60f);
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < 100; i++) rig.Solver.Step(1f / 60f);
            Assert.That(GC.GetAllocatedBytesForCurrentThread() - before, Is.Zero);
        }

        [Test]
        public void FiveFingers_FromAHandAtTheEndOfAnArm_Converge()
        {
            using var rig = Hand();
            rig.Solver.Solve();
            AssertConverged(rig, 1, 2, 3, 4, 5);
            AssertRigid(rig);
        }

        [Test]
        public void NestedBranchPoints_FingersOnOneArmAndTheOtherHandConvergeTogether()
        {
            var offsets = new List<Vector3>(TorsoOffsets);
            var parents = new List<int>(TorsoParents);
            for (int finger = 0; finger < 3; finger++)
            {
                int first = offsets.Count;
                offsets.Add(new Vector3(-0.1f, (finger - 1) * 0.08f, 0f));
                offsets.Add(new Vector3(-0.15f, 0f, 0.03f));
                offsets.Add(new Vector3(-0.15f, 0f, -0.03f));
                parents.Add(5); parents.Add(first); parents.Add(first + 1);
            }
            using var rig = new Rig(offsets.ToArray(), parents.ToArray());
            rig.Add("Trunk", null, 0, 1, 2);
            rig.Add("Left arm", null, 2, 3, 4, 5);
            rig.Add("Right arm", rig.Target(new Vector3(0.75f, 0.85f, 0.2f)), 2, 6, 7, 8);
            for (int finger = 0; finger < 3; finger++)
            {
                int first = 9 + finger * 3;
                rig.Add($"Finger {finger}", rig.Target(rig.Joints[first + 2].position + new Vector3(0.15f, 0.05f, 0.1f)),
                    5, first, first + 1, first + 2);
            }
            rig.Solver.Solve();
            AssertConverged(rig, 2, 3, 4, 5);
            AssertRigid(rig);
        }

        [TestCase(false)]
        [TestCase(true)]
        public void BranchLimits_StayInsideChestAndShoulderFrames(bool tight)
        {
            using var rig = Torso();
            foreach (int i in new[] { 2, 3, 6 })
            {
                BallSocketIKJoint ball = AddBall(rig.Joints[i], tight ? 8f : 70f);
                ball.constraintAxis = i == 2 ? Vector3.up : i == 3 ? Vector3.left : Vector3.right;
            }
            rig.Solver.Solve();
            if (!tight) AssertConverged(rig, 1, 2);
            foreach (int i in new[] { 2, 3, 6 })
            {
                SolverJoint joint = rig.Solver.Chain.Joints[i];
                Quaternion deviation = joint.ToConstraintDeviation(rig.Rotation(joint.ParentIndex), rig.Rotation(i));
                RotationNear(deviation, joint.Angular.ClampDeviation(deviation, deviation));
            }
            Assert.That(rig.Joints[6].GetComponent<BallSocketIKJoint>().IKParentTransform, Is.SameAs(rig.Joints[2]));
            AssertRigid(rig);
        }

        [Test]
        public void ConstrainedFiveFingerHand_KeepsLimitsAndReachesNearbyTargets()
        {
            using var rig = Hand();
            AddBall(rig.Joints[2], 50f);
            for (int i = 3; i < rig.Joints.Length; i += 3) AddBall(rig.Joints[i], 80f);
            rig.Solver.Solve();
            AssertConverged(rig, 1, 2, 3, 4, 5);
            AssertRigid(rig);
        }

        [TestCase(false)]
        [TestCase(true)]
        public void SingularityHandling_BendsStraightBranches(bool handling)
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.left * 0.2f, Vector3.left * 0.4f, Vector3.left * 0.4f,
                Vector3.right * 0.2f, Vector3.right * 0.4f, Vector3.right * 0.4f }, new[] { -1, 0, 1, 2, 0, 4, 5 });
            rig.Add("Left", rig.Target(Vector3.left * 0.7f), 0, 1, 2, 3).UseSingularityHandling = handling;
            rig.Add("Right", rig.Target(Vector3.right * 0.7f), 0, 4, 5, 6).UseSingularityHandling = handling;
            rig.Solver.Solve();
            if (handling) AssertConverged(rig, 0, 1);
            else Assert.That(rig.Solver.LastSolution.FinalError, Is.GreaterThan(0.1f));
            AssertRigid(rig);
        }

        [Test]
        public void ElbowPole_TurnsTheElbowTowardItsTarget()
        {
            using var rig = Torso();
            Transform pole = rig.Target(new Vector3(-0.5f, 1.5f, 2f));
            rig.Solver.Chains[1].AddPole(rig.Joints[4], pole);
            rig.Solver.Solve();
            AssertConverged(rig, 1, 2);
            Vector3 axis = (rig.Position(5) - rig.Position(3)).normalized;
            Vector3 bend = Vector3.ProjectOnPlane(rig.Position(4) - rig.Position(3), axis);
            Vector3 desired = Vector3.ProjectOnPlane(pole.position - rig.Position(3), axis);
            Assert.That(Vector3.Angle(bend, desired), Is.LessThan(0.05f));
            AssertRigid(rig);
        }

        [Test]
        public void PoleStartingAtBranchPoint_IsReportedAndSkipped()
        {
            using var rig = Torso();
            rig.Solver.Chains[1].AddPole(rig.Joints[3], rig.Target(new Vector3(-0.3f, 1f, 2f)));
            LogAssert.Expect(LogType.Warning, new Regex("Chain Left arm on J2.*section starts at.*J2"));
            rig.Solver.Solve();
            AssertConverged(rig, 1, 2);
            AssertRigid(rig);
            rig.Solver.Solve();
            LogAssert.NoUnexpectedReceived();
        }

        [Test]
        public void PoleEndingAtAConstrainedBranchPoint_RespectsItsLimit()
        {
            using var rig = Torso();
            rig.Joints[1].localPosition += Vector3.right * 0.15f;
            rig.Joints[2].localPosition -= Vector3.right * 0.15f;
            AddBall(rig.Joints[2], 0f);
            var pole = rig.Solver.Chains[0].AddPole(rig.Joints[1], rig.Target(new Vector3(0f, 0.5f, 2f)));
            rig.Solver.MaxIterations = 0;
            rig.Solver.Solve();
            Assert.That(pole.IsLimited, Is.True);
            RotationNear(Quaternion.Inverse(rig.Rotation(1)) * rig.Rotation(2), Quaternion.identity);
            AssertRigid(rig);
        }

        [TestCase(false)]
        [TestCase(true)]
        public void PolesOnFreeOrDisabledArms_DoNotTurnTheirJoints(bool disabled)
        {
            using var rig = Torso();
            var right = rig.Solver.Chains[2];
            right.AddPole(rig.Joints[7], rig.Target(new Vector3(0.5f, 3f, 0f)));
            if (disabled) right.Enabled = false;
            else right.Target = null;
            rig.Solver.Solve();
            RotationNear(Quaternion.Inverse(rig.Rotation(2)) * rig.Rotation(6), Quaternion.identity);
            RotationNear(Quaternion.Inverse(rig.Rotation(6)) * rig.Rotation(7), Quaternion.identity);
            AssertRigid(rig);
        }

        [Test]
        public void Apply_UsesSpeedLimitsAndReportsActualErrorWhileLeafRotationsRideAlong()
        {
            using var rig = Torso();
            BallSocketIKJoint ball = AddBall(rig.Joints[3], 180f);
            ball.limitSpeed = true;
            ball.maxAngularSpeed = 10f;
            rig.Joints[5].localRotation = Quaternion.Euler(15f, 25f, 35f);
            Quaternion leafLocal = rig.Joints[5].localRotation;
            rig.Solver.Step(0f);
            RotationNear(rig.Joints[3].localRotation, Quaternion.identity);
            Assert.That(rig.Solver.ApplicationStatus.AnyJointLimited || rig.Solver.ApplicationStatus.AnyJointBlocked, Is.True);
            Assert.That(rig.Solver.ApplicationStatus.PositionError, Is.GreaterThan(rig.Solver.LastSolution.FinalError));
            rig.Solver.Apply(0.1f);
            Assert.That(Quaternion.Angle(Quaternion.identity, rig.Joints[3].localRotation), Is.InRange(0.9f, 1.1f));
            RotationNear(rig.Joints[5].localRotation, leafLocal);
            Assert.That(rig.Solver.LastSolution.WritesRotation[2], Is.True);
            Assert.That(rig.Solver.LastSolution.WritesRotation[5], Is.False);
            Assert.That(rig.Solver.LastSolution.WritesRotation[8], Is.False);
            rig.Solver.SnapToSolution();
            Assert.That(rig.Solver.ApplicationStatus.PositionError, Is.EqualTo(rig.Solver.LastSolution.FinalError).Within(Epsilon));
        }

        [Test]
        public void Results_UseEachToleranceAndRealTargetError()
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.left, Vector3.right }, new[] { -1, 0, 0 });
            var left = rig.Add("Left", rig.Target(Vector3.left * 2f), 0, 1);
            var right = rig.Add("Right", rig.Target(Vector3.right * 2f), 0, 2);
            left.Tolerance = 1.1f;
            right.Tolerance = 0.01f;
            rig.Solver.Solve();
            Assert.That(rig.Solver.Chains[0].LastResult.Converged, Is.True);
            Assert.That(rig.Solver.Chains[1].LastResult.Converged, Is.False);
            Assert.That(rig.Solver.LastSolution.FinalError, Is.EqualTo(1f).Within(Epsilon));
            Assert.That(rig.Solver.LastSolution.Converged, Is.False);
            right.Enabled = false;
            rig.Solver.Solve();
            Assert.That(rig.Solver.Chains[1].LastResult.HasTarget, Is.False);
            Assert.That(float.IsNaN(rig.Solver.Chains[1].LastResult.Error), Is.True);
            Assert.That(rig.Solver.LastSolution.Converged, Is.True);
        }

        [Test]
        public void SliderBranches_RespectTravelAndRebuildRestOffsets()
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.left, Vector3.right }, new[] { -1, 0, 0 });
            var slider = rig.Joints[1].gameObject.AddComponent<SliderIKJoint>();
            slider.slideAxis = Vector3.left;
            slider.minLength = -0.2f;
            slider.maxLength = 0.4f;
            rig.Add("Left", rig.Target(Vector3.left * 1.3f), 0, 1);
            var right = rig.Add("Right", rig.Target(Vector3.right), 0, 2);
            rig.Solver.Solve();
            AssertConverged(rig, 0, 1);
            rig.Solver.SnapToSolution();
            Near(rig.Joints[1].localPosition, Vector3.left * 1.3f);
            rig.Solver.RemoveChain(right);
            Near(rig.Joints[1].localPosition, Vector3.left);
            Near(rig.Solver.Chain.Joints[1].RestLocalOffset, Vector3.left);
        }

        [Test]
        public void PullBelowOne_ScalesTheFirstFitBeforePin()
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.left, Vector3.right }, new[] { -1, 0, 0 });
            rig.Solver.SetPull(rig.Add("Left", rig.Target(Vector3.left + Vector3.up), 0, 1), 0.1f);
            rig.Solver.SetPull(rig.Add("Right", rig.Target(Vector3.right + Vector3.up), 0, 2), 0.2f);
            rig.Solver.RootPin = 0.5f;
            rig.Solver.MaxIterations = 1;
            rig.Solver.Solve();
            Near(rig.Position(0), Vector3.up * 0.15f);
        }

        [Test]
        public void TreeValidation_RejectsMissingTargetsNullJointsAndCrossedPaths()
        {
            using var rig = Torso();
            var tree = new MultiFABRIKTree();
            var problems = new List<string>();
            Assert.That(tree.Build(new List<MultiFABRIKChain> { rig.Solver.Chains[0] }, problems), Is.False);
            Assert.That(string.Join(" ", problems), Does.Contain("target"));
            Transform target = rig.Solver.Chains[1].Target;
            var missing = rig.Solver.AddChain("Missing", new[] { rig.Joints[0], null }, target);
            Assert.That(tree.Build(new List<MultiFABRIKChain> { missing }, problems), Is.False);
            Assert.That(string.Join(" ", problems), Does.Contain("joint 1 is missing"));
            var skipped = rig.Add("Skipped", target, 0, 1, 5);
            var crossing = rig.Add("Crossing", null, 1, 2, 6);
            Assert.That(tree.Build(new List<MultiFABRIKChain> { skipped, crossing }, problems), Is.False);
            Assert.That(string.Join(" ", problems), Does.Contain("crosses another tree joint"));
        }

        [TestCase(false)]
        [TestCase(true)]
        public void SlidingTwoArmRig_RespectsAllThreeJointTypes(bool unreachable)
        {
            var offsets = new[]
            {
                Vector3.zero, new Vector3(0f, 4f, 0f), new Vector3(0f, 2f, 0f),
                new Vector3(-1.5f, 0f, 0f), new Vector3(-2.5f, 1.4f, 0f), new Vector3(-2.5f, -1.4f, 0f),
                new Vector3(1.5f, 0f, 0f), new Vector3(2.5f, 1.4f, 0f), new Vector3(2.5f, -1.4f, 0f)
            };
            using var rig = new Rig(offsets, TorsoParents);
            AddBall(rig.Joints[0], 0f).constraintAxis = Vector3.up;
            var slider = rig.Joints[1].gameObject.AddComponent<SliderIKJoint>();
            slider.slideAxis = Vector3.up;
            slider.minLength = -1f;
            slider.maxLength = 3f;
            var hub = AddBall(rig.Joints[2], 25f);
            hub.constraintAxis = Vector3.up;
            hub.twistHalfAngle = 30f;
            foreach (int i in new[] { 3, 6 })
            {
                var shoulder = AddBall(rig.Joints[i], 70f);
                shoulder.constraintAxis = i == 3 ? Vector3.left : Vector3.right;
                shoulder.twistHalfAngle = 45f;
                var hinge = rig.Joints[i + 1].gameObject.AddComponent<HingeIKJoint>();
                hinge.hingeAxis = Vector3.forward;
                hinge.zeroAngleOffset = 180f;
                hinge.hingeMinAngle = -100f;
                hinge.hingeMaxAngle = 100f;
            }
            rig.Add("Lift and hub", null, 0, 1, 2);
            Quaternion hubRotation = Quaternion.Euler(5f, 0f, 8f);
            var hubPosition = new Vector3(0f, 7.2f, 0f);
            for (int side = 0; side < 2; side++)
            {
                float sign = side == 0 ? -1f : 1f;
                int shoulder = side == 0 ? 3 : 6;
                Quaternion armRotation = Quaternion.Euler(0f, sign * 12f, -sign * 12f);
                Quaternion elbowRotation = Quaternion.Euler(0f, 0f, sign * 20f);
                Vector3 target = unreachable ? new Vector3(sign * 14f, 20f, -sign * 8f)
                    : hubPosition + Vector3.up * 1.5f + hubRotation * (offsets[shoulder]
                        + armRotation * (offsets[shoulder + 1] + elbowRotation * offsets[shoulder + 2]));
                rig.Add(side == 0 ? "Left" : "Right", rig.Target(target), 2, shoulder, shoulder + 1, shoulder + 2).Tolerance = 0.005f;
            }
            rig.Solver.MaxIterations = 48;
            for (int frame = 0; frame < 100; frame++) rig.Solver.Step(1f / 60f);
            float travel = rig.Joints[1].localPosition.y - 4f;
            Assert.That(travel, Is.InRange(-1.00001f, 3.00001f));
            Assert.That(Mathf.Abs(rig.Joints[1].localPosition.x) + Mathf.Abs(rig.Joints[1].localPosition.z), Is.LessThan(Epsilon));
            if (unreachable)
            {
                Assert.That(travel, Is.EqualTo(3f).Within(Epsilon));
                Assert.That(rig.Solver.Chains[1].LastResult.Converged, Is.False);
                Assert.That(rig.Solver.Chains[2].LastResult.Converged, Is.False);
            }
            else
            {
                AssertConverged(rig, 1, 2);
                Assert.That(travel, Is.GreaterThan(0.5f));
                Assert.That(Quaternion.Angle(Quaternion.identity, rig.Joints[2].localRotation), Is.GreaterThan(1f));
                Assert.That(Quaternion.Angle(Quaternion.identity, rig.Joints[7].localRotation), Is.GreaterThan(1f));
            }
            foreach (SolverJoint joint in rig.Solver.Chain.Joints)
            {
                Quaternion parent = joint.ParentIndex < 0 ? Quaternion.identity : rig.Rotation(joint.ParentIndex);
                Quaternion deviation = joint.ToConstraintDeviation(parent, joint.Transform.rotation);
                RotationNear(deviation, joint.Angular.ClampDeviation(deviation, deviation));
            }
            AssertRigid(rig);
        }

        private static Rig Hand()
        {
            var offsets = new List<Vector3> { Vector3.zero, new(0f, 0.4f, 0.05f), new(0f, 0.4f, -0.05f) };
            var parents = new List<int> { -1, 0, 1 };
            for (int finger = 0; finger < 5; finger++)
            {
                int first = offsets.Count;
                offsets.Add(new Vector3((finger - 2) * 0.12f, 0.12f, 0f));
                offsets.Add(new Vector3(0f, 0.2f, 0.04f));
                offsets.Add(new Vector3(0f, 0.2f, -0.04f));
                parents.Add(2); parents.Add(first); parents.Add(first + 1);
            }
            var rig = new Rig(offsets.ToArray(), parents.ToArray());
            rig.Add("Arm", null, 0, 1, 2);
            for (int finger = 0; finger < 5; finger++)
            {
                int first = 3 + finger * 3;
                Vector3 goal = rig.Joints[first + 2].position + new Vector3(0.04f, -0.06f, 0.08f + finger * 0.012f);
                rig.Add($"Finger {finger}", rig.Target(goal), 2, first, first + 1, first + 2);
            }
            return rig;
        }

        private static BallSocketIKJoint AddBall(Transform joint, float angle)
        {
            var ball = joint.gameObject.AddComponent<BallSocketIKJoint>();
            ball.swingPitchHalfAngle = ball.swingYawHalfAngle = ball.twistHalfAngle = angle;
            return ball;
        }

        private static Rig Torso()
        {
            var rig = new Rig(TorsoOffsets, TorsoParents);
            rig.Add("Trunk", null, 0, 1, 2);
            rig.Add("Left arm", rig.Target(new Vector3(-0.75f, 1.15f, 0.3f)), 2, 3, 4, 5);
            rig.Add("Right arm", rig.Target(new Vector3(0.75f, 0.85f, 0.2f)), 2, 6, 7, 8);
            return rig;
        }

        private static void AddHinge(Transform joint)
        {
            var hinge = joint.gameObject.AddComponent<HingeIKJoint>();
            hinge.hingeAxis = Vector3.right;
            hinge.hingeMinAngle = -60f;
            hinge.hingeMaxAngle = 60f;
        }

        private static void AssertConverged(Rig rig, params int[] chains)
        {
            foreach (int c in chains)
                Assert.That(rig.Solver.Chains[c].LastResult.Error, Is.LessThanOrEqualTo(rig.Solver.Chains[c].Tolerance),
                    $"{rig.Solver.Chains[c].ChainName}, iterations {rig.Solver.LastSolution.IterationsUsed}");
        }

        private static void AssertRigid(Rig rig)
        {
            for (int i = 1; i < rig.Joints.Length; i++)
            {
                int parent = rig.Solver.Chain.Joints[i].ParentIndex;
                Near(Quaternion.Inverse(rig.Rotation(parent)) * (rig.Position(i) - rig.Position(parent)),
                    rig.Solver.Chain.Joints[i].Segment.GetCurrentLocalOffset(), $"offset {i}");
            }
        }

        private static void Near(Vector3 actual, Vector3 expected, string message = null) =>
            Assert.That(Vector3.Distance(actual, expected), Is.LessThan(Epsilon), message);

        private static void RotationNear(Quaternion actual, Quaternion expected)
        {
            Near(actual * Vector3.right, expected * Vector3.right);
            Near(actual * Vector3.up, expected * Vector3.up);
        }

        private sealed class Rig : IDisposable
        {
            private readonly List<GameObject> _extras = new();
            private readonly HashSet<Transform> _points = new();
            internal readonly GameObject Root = new("Multi FABRIK test");
            internal readonly Transform[] Joints;
            internal readonly Vector3[] InitialPositions;
            internal readonly MultiFABRIKSolver Solver;

            internal Rig(Vector3[] offsets, int[] parents)
            {
                Joints = new Transform[offsets.Length];
                InitialPositions = new Vector3[offsets.Length];
                for (int i = 0; i < offsets.Length; i++)
                {
                    Joints[i] = new GameObject($"J{i}").transform;
                    Joints[i].SetParent(parents[i] < 0 ? Root.transform : Joints[parents[i]], false);
                    Joints[i].localPosition = offsets[i];
                    InitialPositions[i] = Joints[i].position;
                }
                Solver = Root.AddComponent<MultiFABRIKSolver>();
                Solver.UpdateMode = UpdateMode.Manual;
                Solver.MaxIterations = 100;
            }

            internal Transform Target(Vector3 position)
            {
                var go = new GameObject("Target");
                _extras.Add(go);
                go.transform.position = position;
                return go.transform;
            }

            // Chains added later that start at this joint use Point mode.
            internal void Point(int index) => _points.Add(Joints[index]);

            internal MultiFABRIKChain Add(string name, Transform target, params int[] indices)
            {
                var joints = new Transform[indices.Length];
                for (int i = 0; i < indices.Length; i++) joints[i] = Joints[indices[i]];
                return Solver.AddChain(name, joints, target, junction: _points.Contains(joints[0])
                    ? MultiFABRIKJunctionMode.Point : MultiFABRIKJunctionMode.Rigid);
            }

            internal List<MultiFABRIKChain> ChainList() => new(Solver.Chains);

            internal Vector3 Position(int index) => Solver.LastSolution.WorldPositions[index];
            internal Quaternion Rotation(int index) => Solver.LastSolution.WorldRotations[index];

            public void Dispose()
            {
                foreach (var extra in _extras) Object.DestroyImmediate(extra);
                Object.DestroyImmediate(Root);
            }
        }
    }
}
