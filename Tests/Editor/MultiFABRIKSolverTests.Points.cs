using System;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;

namespace OpenIK.Editor.Tests
{
    public partial class MultiFABRIKSolverTests
    {
        [TestCase(0f)]
        [TestCase(2f)]
        public void PointTree_AcceptsEqualOffsetsAndIndependentAngularLimits(float offset)
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.up, Vector3.up * offset,
                Vector3.left, Vector3.up * offset, Vector3.right }, new[] { -1, 0, 1, 2, 1, 4 });
            rig.Point(1);
            AddHinge(rig.Joints[2]);
            AddBall(rig.Joints[4], 45f);
            rig.Add("Lift", null, 0, 1);
            rig.Add("Left", rig.Target(rig.Joints[3].position), 1, 2, 3);
            rig.Add("Right", rig.Target(rig.Joints[5].position), 1, 4, 5);
            var tree = new MultiFABRIKTree();
            var problems = new List<string>();
            Assert.That(tree.Build(rig.ChainList(), problems), Is.True, string.Join("\n", problems));
            Assert.That(tree.PointJunctions[1], Is.True);
            Assert.That(tree.PointPivots[2] && tree.PointPivots[4], Is.True);
        }

        [TestCase("offset")]
        [TestCase("slider")]
        [TestCase("shared limit")]
        [TestCase("indirect")]
        public void PointTree_RejectsIncompatibleAttachments(string problem)
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.up, Vector3.up }, new[] { -1, 0, 0 });
            rig.Point(0);
            rig.Add("Left", rig.Target(Vector3.up), 0, 1);
            rig.Add("Right", rig.Target(Vector3.up), 0, 2);
            if (problem == "offset") rig.Joints[2].localPosition = Vector3.right;
            if (problem == "slider") rig.Joints[2].gameObject.AddComponent<SliderIKJoint>().jointIsEnabled = false;
            if (problem == "shared limit") AddBall(rig.Joints[0], 30f).jointIsEnabled = false;
            if (problem == "indirect")
            {
                var intermediate = new GameObject("Intermediate").transform;
                intermediate.SetParent(rig.Joints[0], false);
                rig.Joints[2].SetParent(intermediate, true);
            }
            var problems = new List<string>();
            Assert.That(new MultiFABRIKTree().Build(rig.ChainList(), problems), Is.False);
            Assert.That(problems.Exists(p => p.Contains("Point junction")), Is.True);
        }

        [Test]
        public void PointTree_OneOutgoingPathStillBlocksPolesAtTheAttachment()
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.up, Vector3.up }, new[] { -1, 0, 1 });
            rig.Point(0);
            rig.Add("Arm", rig.Target(Vector3.up), 0, 1, 2);
            var tree = new MultiFABRIKTree();
            Assert.That(tree.Build(rig.ChainList(), new List<string>()), Is.True);
            Assert.That(tree.Chains[0].Blocked[0], Is.True);
        }

        [Test]
        public void PointTree_SettingOneChainsJunctionSetsEveryChainOnThatJoint()
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.up, Vector3.up }, new[] { -1, 0, 0 });
            var left = rig.Add("Left", rig.Target(Vector3.up), 0, 1);
            var right = rig.Add("Right", rig.Target(Vector3.up), 0, 2);
            var tree = new MultiFABRIKTree();
            var problems = new List<string>();
            right.Junction = MultiFABRIKJunctionMode.Point;
            Assert.That(left.Junction, Is.EqualTo(MultiFABRIKJunctionMode.Point));
            Assert.That(tree.Build(rig.ChainList(), problems), Is.True, string.Join("\n", problems));
            Assert.That(tree.PointJunctions[0], Is.True);
            left.Junction = MultiFABRIKJunctionMode.Rigid;
            Assert.That(right.Junction, Is.EqualTo(MultiFABRIKJunctionMode.Rigid));
            Assert.That(tree.Build(rig.ChainList(), problems), Is.True);
            Assert.That(tree.HasPointJunctions, Is.False);
        }

        [Test]
        public void PointTree_MismatchedJunctionsFromSerializedEditsAreReported()
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.up, Vector3.up }, new[] { -1, 0, 0 });
            rig.Add("Left", rig.Target(Vector3.up), 0, 1);
            var right = rig.Add("Right", rig.Target(Vector3.up), 0, 2);
            using (var serialized = new UnityEditor.SerializedObject(right))
            {
                serialized.FindProperty("junction").enumValueIndex = (int)MultiFABRIKJunctionMode.Point;
                serialized.ApplyModifiedPropertiesWithoutUndo();
            }
            var problems = new List<string>();
            Assert.That(new MultiFABRIKTree().Build(rig.ChainList(), problems), Is.False);
            Assert.That(problems.Exists(p => p.StartsWith("Chain Right on J0:")), Is.True, string.Join("\n", problems));
        }

        [Test]
        public void PointTree_AddChainKeepsTheJointsModeUnlessGivenOne()
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.up, Vector3.up, Vector3.up }, new[] { -1, 0, 0, 0 });
            var first = rig.Solver.AddChain("First", new[] { rig.Joints[0], rig.Joints[1] }, rig.Target(Vector3.up),
                junction: MultiFABRIKJunctionMode.Point);
            var second = rig.Solver.AddChain("Second", new[] { rig.Joints[0], rig.Joints[2] });
            Assert.That(second.Junction, Is.EqualTo(MultiFABRIKJunctionMode.Point));
            rig.Solver.AddChain("Third", new[] { rig.Joints[0], rig.Joints[3] }, junction: MultiFABRIKJunctionMode.Rigid);
            Assert.That(first.Junction, Is.EqualTo(MultiFABRIKJunctionMode.Rigid));
            Assert.That(second.Junction, Is.EqualTo(MultiFABRIKJunctionMode.Rigid));
        }

        [TestCase(1, 0f)]
        [TestCase(2, 0f)]
        [TestCase(5, 0f)]
        [TestCase(1, 2f)]
        [TestCase(2, 2f)]
        [TestCase(5, 2f)]
        public void PointSolve_ReachesIndependentTargetsAndKeepsAttachmentsCoincident(int count, float offset)
        {
            using var rig = PointArms(count, offset);
            Assert.That(rig.Solver.Solve(), Is.True);
            for (int c = 0; c < count; c++) AssertConverged(rig, c);
            RotationNear(rig.Rotation(0), Quaternion.identity);
            AssertRigid(rig);
            for (int c = 0; c < count; c++) Near(rig.Position(1 + c * 3), Vector3.up * offset);
            for (int i = 0; i < rig.Joints.Length; i++) Near(rig.Joints[i].position, rig.InitialPositions[i]);
            rig.Solver.SnapToSolution();
            for (int c = 0; c < count; c++) Near(rig.Joints[1 + c * 3].position, Vector3.up * offset);
        }

        [TestCase(0f, 1f, 1f, 0f)]
        [TestCase(2f, 1f, 0.25f, -0.6f)]
        [TestCase(2f, 0f, 0f, 0f)]
        [TestCase(2f, 0.25f, 0f, -0.25f)]
        public void PointSolve_FloatingRootUsesWeightedBaseProposals(float offset, float left, float right, float x)
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.up * offset, Vector3.up * offset }, new[] { -1, 0, 0 });
            rig.Point(0);
            rig.Solver.SetPull(rig.Add("Left", rig.Target(Vector3.up * offset + Vector3.left), 0, 1), left);
            rig.Solver.SetPull(rig.Add("Right", rig.Target(Vector3.up * offset + Vector3.right), 0, 2), right);
            rig.Solver.RootPin = 0f;
            rig.Solver.MaxIterations = 1;
            rig.Solver.Solve();
            Near(rig.Position(0), Vector3.right * x);
            Near(rig.Position(1), rig.Position(2));
            RotationNear(rig.Rotation(0), Quaternion.identity);
        }

        [TestCase(0f)]
        [TestCase(0.5f)]
        [TestCase(1f)]
        public void PointSolve_RootPinBlendsOnlyPosition(float pin)
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.up, Vector3.up }, new[] { -1, 0, 0 });
            rig.Point(0);
            rig.Add("Left", rig.Target(Vector3.up + Vector3.right), 0, 1);
            rig.Add("Right", rig.Target(Vector3.up + Vector3.right), 0, 2);
            rig.Solver.RootPin = pin;
            rig.Solver.MaxIterations = 1;
            rig.Solver.Solve();
            Near(rig.Position(0), Vector3.right * (1f - pin));
            RotationNear(rig.Rotation(0), Quaternion.identity);
        }

        [TestCase(false)]
        [TestCase(true)]
        public void PointSolve_ParentFrameFollowsExternalRotationAndRespectsStartingPose(bool rest)
        {
            using var rig = PointArms(2, 1f);
            rig.Solver.SolveFromRestPose = rest;
            rig.Solver.Solve();
            rig.Solver.SnapToSolution();
            rig.Joints[0].localRotation = Quaternion.Euler(15f, 0f, 0f);
            rig.Root.transform.rotation = Quaternion.Euler(20f, 35f, 10f);
            Quaternion expected = rig.Root.transform.rotation * (rest ? Quaternion.identity : rig.Joints[0].localRotation);
            rig.Solver.Solve();
            RotationNear(rig.Rotation(0), expected);
            Near(rig.Position(1), rig.Position(4));
            AssertRigid(rig);
        }

        [TestCase(false)]
        [TestCase(true)]
        public void PointSolve_FreeOrDisabledSiblingKeepsItsOwnRotation(bool disabled)
        {
            using var rig = PointArms(2, 2f);
            if (disabled) rig.Solver.Chains[1].Enabled = false;
            else rig.Solver.Chains[1].Target = null;
            Quaternion pose = Quaternion.Euler(15f, 20f, 30f);
            rig.Joints[4].localRotation = pose;
            rig.Solver.Chains[0].Target.position += Vector3.up * 0.3f;
            for (int frame = 0; frame < 5; frame++) rig.Solver.Step(1f / 60f);
            RotationNear(rig.Rotation(0), Quaternion.identity);
            RotationNear(rig.Joints[4].localRotation, pose);
            Near(rig.Joints[1].position, rig.Joints[4].position);
        }

        [Test]
        public void PointSolve_BranchPolesDoNotRotateTheCommonFrame()
        {
            using var rig = PointArms(2, 2f);
            rig.Solver.Chains[0].AddPole(rig.Joints[2], rig.Target(new Vector3(0f, 5f, 3f)));
            rig.Solver.Chains[1].AddPole(rig.Joints[5], rig.Target(new Vector3(0f, 5f, -3f)));
            rig.Solver.Solve();
            AssertConverged(rig, 0, 1);
            RotationNear(rig.Rotation(0), Quaternion.identity);
            Near(rig.Position(1), rig.Position(4));
            AssertRigid(rig);
        }

        [Test]
        public void PointSolve_ZeroDeltaAndBranchSpeedLimitKeepActualBasesTogether()
        {
            using var rig = PointArms(2, 2f);
            var ball = AddBall(rig.Joints[1], 180f);
            ball.limitSpeed = true;
            ball.maxAngularSpeed = 10f;
            rig.Solver.Solve();
            Quaternion before = rig.Joints[1].localRotation;
            rig.Solver.Apply(0f);
            RotationNear(rig.Joints[1].localRotation, before);
            rig.Solver.Apply(0.1f);
            Assert.That(Quaternion.Angle(before, rig.Joints[1].localRotation), Is.LessThanOrEqualTo(1.001f));
            Near(rig.Joints[1].position, rig.Joints[4].position);
            Assert.That(rig.Solver.ApplicationStatus.AnyJointLimited, Is.True);
            rig.Solver.SnapToSolution();
            Near(rig.Joints[1].position, rig.Joints[4].position);
        }

        [Test]
        public void PointSolve_RejectsAReboundSlidingPivotBeforeApplyingIt()
        {
            using var rig = PointArms(2, 2f);
            var component = rig.Joints[1].gameObject.AddComponent<SwitchingPointSegment>();
            component.jointIsEnabled = false;
            rig.Solver.Solve();
            component.jointIsEnabled = true;
            Assert.Throws<InvalidOperationException>(() => rig.Solver.Solve());
            Near(rig.Joints[1].position, rig.Joints[4].position);
        }

        [Test]
        public void PointSolve_AllocatesNothingAfterWarmupWithLimitsAndPoles()
        {
            using var rig = PointArms(2, 2f);
            AddBall(rig.Joints[1], 80f);
            rig.Solver.Chains[1].AddPole(rig.Joints[5], rig.Target(new Vector3(0f, 3f, -3f)));
            for (int i = 0; i < 20; i++) rig.Solver.Solve();
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < 50; i++) rig.Solver.Solve();
            Assert.That(GC.GetAllocatedBytesForCurrentThread() - before, Is.Zero);
        }

        [TestCase(0f, 2f)]
        [TestCase(0.5f, 1.5f)]
        [TestCase(1f, 1f)]
        public void PointSolve_TrunkPinBlendsItsTargetWithBranchProposals(float pin, float height)
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.up, Vector3.zero, Vector3.zero }, new[] { -1, 0, 1, 1 });
            AddBall(rig.Joints[0], 0f);
            var slider = rig.Joints[1].gameObject.AddComponent<SliderIKJoint>();
            slider.slideAxis = Vector3.up;
            slider.minLength = -0.8f;
            slider.maxLength = 3f;
            rig.Point(1);
            rig.Solver.SetPin(rig.Add("Trunk", rig.Target(Vector3.up), 0, 1), pin);
            rig.Add("Left", rig.Target(Vector3.up * 2f), 1, 2);
            rig.Add("Right", rig.Target(Vector3.up * 2f), 1, 3);
            rig.Solver.MaxIterations = 1;
            rig.Solver.Solve();
            Near(rig.Position(1), Vector3.up * height);
            Near(rig.Position(2), rig.Position(3));
        }

        [Test]
        public void PointSolve_TwoHingesAtTheSamePositionUseDifferentPlanes()
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.zero, Vector3.right, Vector3.right,
                Vector3.zero, Vector3.right, Vector3.right }, new[] { -1, 0, 1, 2, 0, 4, 5 });
            rig.Point(0);
            foreach (int i in new[] { 1, 2, 4, 5 })
            {
                var hinge = rig.Joints[i].gameObject.AddComponent<HingeIKJoint>();
                hinge.hingeAxis = i < 4 ? Vector3.forward : Vector3.up;
                hinge.hingeMinAngle = -120f;
                hinge.hingeMaxAngle = 120f;
            }
            rig.Add("XY", rig.Target(new Vector3(1.2f, 1f, 0f)), 0, 1, 2, 3);
            rig.Add("XZ", rig.Target(new Vector3(1.2f, 0f, 1f)), 0, 4, 5, 6);
            for (int frame = 0; frame < 10; frame++) rig.Solver.Step(1f / 60f);
            AssertConverged(rig, 0, 1);
            Assert.That(Mathf.Abs(rig.Position(3).z), Is.LessThan(Epsilon));
            Assert.That(Mathf.Abs(rig.Position(6).y), Is.LessThan(Epsilon));
            AssertPointLimits(rig);
        }

        [TestCase(false)]
        [TestCase(true)]
        public void PointSolve_SliderAndIndependentHubsRespectAllThreeLimits(bool unreachable)
        {
            var offsets = new[] { Vector3.zero, Vector3.up * 4f,
                Vector3.up * 2f, Vector3.left * 1.5f, new Vector3(-2.5f, 1.4f, 0f), new Vector3(-2.5f, -1.4f, 0f),
                Vector3.up * 2f, Vector3.right * 1.5f, new Vector3(2.5f, 1.4f, 0f), new Vector3(2.5f, -1.4f, 0f) };
            using var rig = new Rig(offsets, new[] { -1, 0, 1, 2, 3, 4, 1, 6, 7, 8 });
            AddBall(rig.Joints[0], 0f);
            var slider = rig.Joints[1].gameObject.AddComponent<SliderIKJoint>();
            slider.slideAxis = Vector3.up;
            slider.minLength = -1f;
            slider.maxLength = 3f;
            rig.Point(1);
            rig.Add("Lift", null, 0, 1);
            for (int side = 0; side < 2; side++)
            {
                float sign = side == 0 ? -1f : 1f;
                int hub = side == 0 ? 2 : 6;
                var ball = AddBall(rig.Joints[hub], 25f);
                ball.constraintAxis = Vector3.up;
                ball.twistHalfAngle = 30f;
                var shoulder = AddBall(rig.Joints[hub + 1], 70f);
                shoulder.constraintAxis = Vector3.right * sign;
                shoulder.twistHalfAngle = 45f;
                var hinge = rig.Joints[hub + 2].gameObject.AddComponent<HingeIKJoint>();
                hinge.hingeAxis = Vector3.forward;
                hinge.hingeMinAngle = -100f;
                hinge.hingeMaxAngle = 100f;
                Vector3 target = unreachable ? new Vector3(sign * 14f, 20f, -sign * 8f)
                    : Vector3.up * 7.5f + Quaternion.Euler(0f, sign * 8f, sign * 12f)
                        * (offsets[hub + 1] + Quaternion.Euler(0f, sign * 12f, -sign * 12f)
                            * (offsets[hub + 2] + Quaternion.Euler(0f, 0f, sign * 20f) * offsets[hub + 3]));
                rig.Add(side == 0 ? "Left" : "Right", rig.Target(target), 1, hub, hub + 1, hub + 2, hub + 3).Tolerance = 0.005f;
            }
            rig.Solver.MaxIterations = 48;
            for (int frame = 0; frame < 100; frame++) rig.Solver.Step(1f / 60f);
            Assert.That(rig.Joints[1].localPosition.y - 4f, Is.InRange(-1.00001f, 3.00001f));
            Near(rig.Joints[2].position, rig.Joints[6].position);
            Near(rig.Joints[2].localPosition, Vector3.up * 2f);
            Near(rig.Joints[6].localPosition, Vector3.up * 2f);
            AssertPointLimits(rig);
            if (!unreachable) AssertConverged(rig, 1, 2);
            else
            {
                Assert.That(rig.Solver.Chains[1].LastResult.Converged, Is.False);
                Assert.That(float.IsNaN(rig.Solver.Chains[2].LastResult.Error), Is.False);
            }
        }

        [TestCase(false)]
        [TestCase(true)]
        public void PointSolve_NestedPointAndRigidJunctionsKeepTheirFrames(bool outerPoint)
        {
            var offsets = new List<Vector3>(TorsoOffsets);
            var parents = new List<int>(TorsoParents);
            if (outerPoint) offsets[3] = offsets[6] = Vector3.zero;
            for (int finger = 0; finger < 2; finger++)
            {
                int pivot = offsets.Count;
                offsets.Add(Vector3.zero);
                offsets.Add(new Vector3(-0.15f, 0.04f, 0f));
                offsets.Add(new Vector3(-0.15f, -0.04f, 0f));
                parents.Add(5); parents.Add(pivot); parents.Add(pivot + 1);
            }
            using var rig = new Rig(offsets.ToArray(), parents.ToArray());
            if (outerPoint) rig.Point(2);
            rig.Point(5);
            rig.Add("Trunk", null, 0, 1, 2);
            rig.Add("Left", null, 2, 3, 4, 5);
            rig.Add("Right", rig.Target(rig.Joints[8].position + Vector3.up * 0.1f), 2, 6, 7, 8);
            for (int finger = 0; finger < 2; finger++)
            {
                int pivot = 9 + finger * 3;
                rig.Add($"Finger {finger}", rig.Target(rig.Joints[pivot + 2].position + new Vector3(0.03f, 0.04f, finger * 0.04f)),
                    5, pivot, pivot + 1, pivot + 2);
            }
            for (int frame = 0; frame < 10; frame++) rig.Solver.Step(1f / 60f);
            AssertConverged(rig, 2, 3, 4);
            RotationNear(Quaternion.Inverse(rig.Rotation(4)) * rig.Rotation(5), Quaternion.identity);
            Near(rig.Position(9), rig.Position(12));
            if (outerPoint)
            {
                RotationNear(Quaternion.Inverse(rig.Rotation(1)) * rig.Rotation(2), Quaternion.identity);
                Near(rig.Position(3), rig.Position(6));
            }
            AssertRigid(rig);
        }

        [Test]
        public void PointSolve_TrunkPoleEndingAtPointPreservesItsLocalFrame()
        {
            using var rig = new Rig(new[] { Vector3.zero, new Vector3(0.2f, 0.5f, 0f),
                new Vector3(-0.2f, 0.5f, 0f), Vector3.zero, Vector3.left, Vector3.zero, Vector3.right },
                new[] { -1, 0, 1, 2, 3, 2, 5 });
            rig.Point(2);
            rig.Add("Trunk", null, 0, 1, 2).AddPole(rig.Joints[1], rig.Target(new Vector3(0f, 0.5f, 2f)));
            rig.Add("Left", rig.Target(rig.Joints[4].position), 2, 3, 4);
            rig.Add("Right", rig.Target(rig.Joints[6].position), 2, 5, 6);
            rig.Solver.MaxIterations = 0;
            rig.Solver.Solve();
            Assert.That(rig.Position(1).z, Is.GreaterThan(0.15f));
            RotationNear(Quaternion.Inverse(rig.Rotation(1)) * rig.Rotation(2), Quaternion.identity);
            Near(rig.Position(3), rig.Position(5));
            AssertRigid(rig);
        }

        [Test]
        public void PointSolve_AddAndRemoveBranchesRebuildThePointMetadata()
        {
            using var rig = PointArms(2, 2f);
            var branch = rig.Solver.Chains[1];
            rig.Solver.Step(1f / 60f);
            Assert.That(rig.Solver.RemoveChain(branch), Is.True);
            rig.Solver.Solve();
            Assert.That(rig.Solver.Chain.IsTree, Is.False);
            AssertConverged(rig, 0);
            rig.Add("Restored", branch.Target, 0, 4, 5, 6);
            rig.Solver.Solve();
            AssertConverged(rig, 0, 1);
            Near(rig.Position(1), rig.Position(4));
        }

        [Test]
        public void PointSolve_MidChainContinuationKeepsItsOriginalChainAndTarget()
        {
            using var rig = new Rig(new[] { Vector3.zero, Vector3.up, Vector3.up, Vector3.right,
                Vector3.up, Vector3.left }, new[] { -1, 0, 1, 2, 1, 4 });
            AddBall(rig.Joints[0], 0f);
            var slider = rig.Joints[1].gameObject.AddComponent<SliderIKJoint>();
            slider.slideAxis = Vector3.up;
            slider.maxLength = 3f;
            rig.Point(1);
            var continuation = rig.Add("Lift and right", rig.Target(Vector3.up * 3f + Vector3.right * 0.8f), 0, 1, 2, 3);
            rig.Add("Left", rig.Target(Vector3.up * 3f + Vector3.left * 0.8f), 1, 4, 5);
            for (int frame = 0; frame < 10; frame++) rig.Solver.Step(1f / 60f);
            AssertConverged(rig, 0, 1);
            Assert.That(continuation.Joints.Count, Is.EqualTo(4));
            Near(rig.Position(2), rig.Position(4));
            AssertPointLimits(rig);
        }

        private static void AssertPointLimits(Rig rig)
        {
            for (int i = 0; i < rig.Joints.Length; i++)
            {
                SolverJoint joint = rig.Solver.Chain.Joints[i];
                Quaternion deviation = joint.ToConstraintDeviation(rig.Solver.Chain.GetParentSolverRotation(i), rig.Rotation(i));
                RotationNear(deviation, joint.Angular.ClampDeviation(deviation, deviation));
            }
            AssertRigid(rig);
        }

        private static Rig PointArms(int count, float offset)
        {
            var offsets = new Vector3[1 + count * 3];
            var parents = new int[offsets.Length];
            parents[0] = -1;
            for (int c = 0; c < count; c++)
            {
                int pivot = 1 + c * 3;
                Quaternion direction = Quaternion.Euler(0f, c * 360f / count, 0f);
                offsets[pivot] = Vector3.up * offset;
                offsets[pivot + 1] = direction * new Vector3(1f, 0.3f, 0f);
                offsets[pivot + 2] = direction * new Vector3(0.7f, -0.3f, 0.3f);
                parents[pivot] = 0;
                parents[pivot + 1] = pivot;
                parents[pivot + 2] = pivot + 1;
            }
            var rig = new Rig(offsets, parents);
            rig.Point(0);
            for (int c = 0; c < count; c++)
            {
                int pivot = 1 + c * 3;
                Vector3 target = Vector3.up * offset + Quaternion.Euler(15f, 0f, 20f)
                    * (offsets[pivot + 1] + Quaternion.Euler(0f, 30f, 0f) * offsets[pivot + 2]);
                rig.Add($"Arm {c}", rig.Target(target), 0, pivot, pivot + 1, pivot + 2);
            }
            return rig;
        }

        private sealed class SwitchingPointSegment : ConstrainedJoint
        {
            public override void UpdateConstraints() { }
            public override ISegmentConstraint CreateSegmentConstraint(in ISegmentConstraint.SetupData setupData) =>
                new SliderSegmentConstraint(setupData, new SliderSegmentConstraint.Config(Vector3.up, -1f, 1f));
        }
    }
}
