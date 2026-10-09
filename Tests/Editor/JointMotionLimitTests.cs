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
    public class JointMotionLimitTests
    {
        private const float AngleEpsilon = 1e-3f;
        private const float Epsilon = 1e-4f;

        // ---------------------------------------------------------------------------------------
        // Shared constraint-frame conversion
        // ---------------------------------------------------------------------------------------

        [TestCase(0f, 0f, 1f)]
        [TestCase(1f, 0f, 1f)]
        [TestCase(0f, 1f, 0f)]
        public void ConstraintDeviation_HingeAngleFollowsRightHandRotationAboutHingeAxis(float x, float y, float z)
        {
            var go = new GameObject("Hinge");
            try
            {
                var hinge = go.AddComponent<HingeIKJoint>();
                hinge.hingeAxis = new Vector3(x, y, z);
                hinge.zeroAngleOffset = 180f;
                hinge.hingeMinAngle = 0f;
                hinge.hingeMaxAngle = 90f;
                hinge.UpdateConstraints();
                var joint = new SolverJoint
                {
                    RestLocalRotation = Quaternion.identity,
                    Angular = hinge.CreateAngularConstraint(new IAngularConstraint.SetupData(Quaternion.identity))
                };

                Vector3 axis = hinge.hingeAxis.normalized;
                Quaternion allowed = Quaternion.AngleAxis(45f, axis);
                Quaternion forbidden = Quaternion.AngleAxis(-45f, axis);

                // Positive rotation about the hinge axis lies inside [0, 90], as drawn by the gizmo.
                Assert.That(Quaternion.Angle(allowed, joint.ClampRotation(Quaternion.identity, Quaternion.identity, allowed)),
                    Is.LessThan(AngleEpsilon));
                Assert.That(Quaternion.Angle(Quaternion.identity, joint.ClampRotation(Quaternion.identity, Quaternion.identity, forbidden)),
                    Is.LessThan(AngleEpsilon));
            }
            finally
            {
                Object.DestroyImmediate(go);
            }
        }

        [Test]
        public void ConstraintDeviation_RoundTripsThroughWorldRotation()
        {
            var joint = new SolverJoint
            {
                RestLocalRotation = Quaternion.Euler(10f, 20f, 30f),
                Angular = new HingeAngularConstraint(new HingeAngularConstraint.Config(
                    Vector3.forward, Quaternion.FromToRotation(Vector3.forward, Vector3.right), -90f, 90f))
            };
            Quaternion parent = Quaternion.Euler(-40f, 5f, 70f);
            Quaternion world = Quaternion.Euler(15f, -60f, 25f);

            Quaternion roundTrip = joint.FromConstraintDeviation(parent, joint.ToConstraintDeviation(parent, world));

            Assert.That(Quaternion.Angle(world, roundTrip), Is.LessThan(AngleEpsilon));
        }

        // ---------------------------------------------------------------------------------------
        // Joint motion providers
        // ---------------------------------------------------------------------------------------

        [Test]
        public void HingeStep_RestrictedRangeMovesThroughAllowedArcNotShortestPath()
        {
            var hinge = new HingeAngularConstraint(new HingeAngularConstraint.Config(Vector3.up, Quaternion.identity, -90f, 90f));

            JointMotionStep step = hinge.StepDeviation(
                Quaternion.AngleAxis(-80f, Vector3.up),
                Quaternion.AngleAxis(80f, Vector3.up),
                30f,
                out Quaternion applied);

            Assert.That(step.Status, Is.EqualTo(JointMotionStatus.Limited));
            Assert.That(step.StartedOutsideLimits, Is.False);
            Assert.That(HingeIKJoint.ExtractHingeAngle(applied, Vector3.up), Is.EqualTo(-50f).Within(AngleEpsilon));
        }

        [Test]
        public void HingeStep_FullTurnRangeWrapsAlongShortestPath()
        {
            var hinge = new HingeAngularConstraint(new HingeAngularConstraint.Config(Vector3.up, Quaternion.identity, -180f, 180f));

            hinge.StepDeviation(
                Quaternion.AngleAxis(170f, Vector3.up),
                Quaternion.AngleAxis(-170f, Vector3.up),
                5f,
                out Quaternion applied);

            Assert.That(Mathf.DeltaAngle(175f, HingeIKJoint.ExtractHingeAngle(applied, Vector3.up)),
                Is.EqualTo(0f).Within(AngleEpsilon));
        }

        [Test]
        public void HingeStep_ReachesDesiredWithinBudgetAndReportsOutsideStart()
        {
            var hinge = new HingeAngularConstraint(new HingeAngularConstraint.Config(Vector3.up, Quaternion.identity, -45f, 45f));
            Quaternion desired = Quaternion.AngleAxis(40f, Vector3.up);

            JointMotionStep reached = hinge.StepDeviation(Quaternion.AngleAxis(35f, Vector3.up), desired, 10f, out Quaternion applied);
            Assert.That(reached.Status, Is.EqualTo(JointMotionStatus.Reached));
            Assert.That(applied, Is.EqualTo(desired));

            JointMotionStep outside = hinge.StepDeviation(Quaternion.AngleAxis(80f, Vector3.up), desired, 10f, out applied);
            Assert.That(outside.StartedOutsideLimits, Is.True);
            Assert.That(HingeIKJoint.ExtractHingeAngle(applied, Vector3.up), Is.EqualTo(70f).Within(AngleEpsilon));
        }

        [Test]
        public void SliderStep_CapsTravelAndPreservesLateralOffset()
        {
            // Rest offset (0.5, 1, 0) from the parent; slide axis is parent-local up.
            var slider = new SliderSegmentConstraint(
                new ISegmentConstraint.SetupData(new Vector3(0.5f, 1f, 0f), 1.118f, Quaternion.identity, Quaternion.identity, true),
                new SliderSegmentConstraint.Config(Vector3.up, 0f, 2f));

            JointMotionStep step = slider.StepLocalOffset(
                new Vector3(0.5f, 1f, 0f),
                new Vector3(0.5f, 2.5f, 0f),
                0.25f,
                out Vector3 applied);

            Assert.That(step.Status, Is.EqualTo(JointMotionStatus.Limited));
            Assert.That(step.StartedOutsideLimits, Is.False);
            AssertVector3(new Vector3(0.5f, 1.25f, 0f), applied);
            Assert.That(slider.CurrentSlide, Is.EqualTo(0f).Within(Epsilon), "Stepping must not change solver state.");

            JointMotionStep reached = slider.StepLocalOffset(new Vector3(0.5f, 2.4f, 0f), new Vector3(0.5f, 2.5f, 0f), 0.25f, out applied);
            Assert.That(reached.Status, Is.EqualTo(JointMotionStatus.Reached));
            AssertVector3(new Vector3(0.5f, 2.5f, 0f), applied);
        }

        [Test]
        public void HingeStep_RemovesOffAxisRotationImmediatelyAndReportsOutsideStart()
        {
            // Off-axis rotation is not a hinge degree of freedom, so removing it spends no budget.
            var hinge = new HingeAngularConstraint(new HingeAngularConstraint.Config(Vector3.up, Quaternion.identity, -45f, 45f));
            Quaternion current = Quaternion.AngleAxis(30f, Vector3.right) * Quaternion.AngleAxis(20f, Vector3.up);
            float currentAngle = HingeIKJoint.ExtractHingeAngle(current, Vector3.up);

            JointMotionStep step = hinge.StepDeviation(current, Quaternion.AngleAxis(40f, Vector3.up), 0f, out Quaternion applied);

            // The hinge angle holds at zero budget, but the result lies on the hinge axis.
            Assert.That(step.Status, Is.EqualTo(JointMotionStatus.Limited));
            Assert.That(step.StartedOutsideLimits, Is.True);
            Assert.That(Quaternion.Angle(Quaternion.AngleAxis(currentAngle, Vector3.up), applied), Is.LessThan(AngleEpsilon));
        }

        [Test]
        public void SliderStep_RemovesSidewaysOffsetImmediatelyAndReportsOutsideStart()
        {
            // Sideways offset is not slider travel, so removing it spends no budget.
            var slider = new SliderSegmentConstraint(
                new ISegmentConstraint.SetupData(new Vector3(0.5f, 1f, 0f), 1.118f, Quaternion.identity, Quaternion.identity, true),
                new SliderSegmentConstraint.Config(Vector3.up, 0f, 2f));

            JointMotionStep step = slider.StepLocalOffset(
                new Vector3(0.8f, 1f, 0f),
                new Vector3(0.5f, 1f, 0f),
                0f,
                out Vector3 applied);

            Assert.That(step.Status, Is.EqualTo(JointMotionStatus.Reached));
            Assert.That(step.StartedOutsideLimits, Is.True);
            AssertVector3(new Vector3(0.5f, 1f, 0f), applied);
        }

        [Test]
        public void BallSocketStep_StaysInsideConeAndTwistWithinBudget()
        {
            float pitchHalfSin = Mathf.Sin(Mathf.Deg2Rad * 50f * 0.5f);
            float yawHalfSin = Mathf.Sin(Mathf.Deg2Rad * 30f * 0.5f);
            var ball = new BallSocketAngularConstraint(new BallSocketAngularConstraint.Config(Quaternion.identity, pitchHalfSin, yawHalfSin, 40f));
            var random = new System.Random(1234);

            for (int sample = 0; sample < 300; sample++)
            {
                Quaternion current = ball.ClampDeviation(RandomRotation(random));
                Quaternion desired = ball.ClampDeviation(RandomRotation(random));
                float budget = (float)random.NextDouble() * 20f;

                JointMotionStep step = ball.StepDeviation(current, desired, budget, out Quaternion applied);

                Assert.That(step.StartedOutsideLimits, Is.False);
                Assert.That(step.Status, Is.Not.EqualTo(JointMotionStatus.Blocked));
                Assert.That(Quaternion.Angle(current, applied), Is.LessThanOrEqualTo(budget + 0.01f), $"Sample {sample} exceeded its budget.");
                Assert.That(Quaternion.Angle(ball.ClampDeviation(applied), applied), Is.LessThan(0.05f), $"Sample {sample} left the cone or twist range.");
                if (step.Status == JointMotionStatus.Limited && budget > 0.5f)
                    Assert.That(Quaternion.Angle(applied, desired), Is.LessThan(Quaternion.Angle(current, desired)), $"Sample {sample} made no progress.");
            }
        }

        [Test]
        public void BallSocketStep_RepeatedStepsConverge()
        {
            float halfSin = Mathf.Sin(Mathf.Deg2Rad * 60f * 0.5f);
            var ball = new BallSocketAngularConstraint(new BallSocketAngularConstraint.Config(Quaternion.identity, halfSin, halfSin, 90f));
            Quaternion current = ball.ClampDeviation(Quaternion.Euler(50f, 0f, 60f));
            Quaternion desired = ball.ClampDeviation(Quaternion.Euler(-50f, 0f, -60f));

            JointMotionStatus status = JointMotionStatus.Limited;
            for (int i = 0; i < 200 && status != JointMotionStatus.Reached; i++)
            {
                status = ball.StepDeviation(current, desired, 5f, out Quaternion applied).Status;
                current = applied;
            }

            Assert.That(status, Is.EqualTo(JointMotionStatus.Reached));
        }

        [Test]
        public void BallSocketStep_WideTwistRangeMeasuresAndFollowsAllowedArc()
        {
            // +160 to -160 with a +/-170 range must turn 320 degrees through zero, not 40 across the seam.
            var ball = WideTwistBallSocket(170f);
            Quaternion current = Quaternion.AngleAxis(160f, Vector3.forward);
            Quaternion desired = Quaternion.AngleAxis(-160f, Vector3.forward);

            Assert.That(ball.GetMotionDistance(current, desired), Is.EqualTo(320f).Within(0.01f));

            JointMotionStep step = ball.StepDeviation(current, desired, 1f, out Quaternion applied);

            Assert.That(step.Status, Is.EqualTo(JointMotionStatus.Limited), "Moving along the allowed arc is progress.");
            Assert.That(step.StartedOutsideLimits, Is.False);
            Assert.That(TwistAngle(applied), Is.EqualTo(159f).Within(0.01f));
        }

        [Test]
        public void BallSocketStep_LargeBudgetDoesNotShortcutAcrossTwistSeam()
        {
            var ball = WideTwistBallSocket(170f);
            Quaternion current = Quaternion.AngleAxis(160f, Vector3.forward);
            Quaternion desired = Quaternion.AngleAxis(-160f, Vector3.forward);

            JointMotionStep step = ball.StepDeviation(current, desired, 50f, out Quaternion applied);

            Assert.That(step.Status, Is.EqualTo(JointMotionStatus.Limited));
            Assert.That(TwistAngle(applied), Is.EqualTo(110f).Within(0.01f));
        }

        [Test]
        public void BallSocketStep_FullTwistRangeTakesShortWayAcrossSeam()
        {
            var ball = WideTwistBallSocket(180f);
            Quaternion current = Quaternion.AngleAxis(160f, Vector3.forward);
            Quaternion desired = Quaternion.AngleAxis(-160f, Vector3.forward);

            Assert.That(ball.GetMotionDistance(current, desired), Is.EqualTo(40f).Within(0.01f));

            ball.StepDeviation(current, desired, 10f, out Quaternion applied);

            Assert.That(TwistAngle(applied), Is.EqualTo(170f).Within(0.01f));
        }

        [Test]
        public void BallSocketStep_RepeatedStepsAcrossWideTwistRangeStayLegalAndConverge()
        {
            var ball = WideTwistBallSocket(170f);
            Quaternion current = Quaternion.AngleAxis(160f, Vector3.forward);
            Quaternion desired = Quaternion.AngleAxis(-160f, Vector3.forward);

            int steps = 0;
            JointMotionStatus status = JointMotionStatus.Limited;
            while (status != JointMotionStatus.Reached && steps < 100)
            {
                status = ball.StepDeviation(current, desired, 5f, out Quaternion applied).Status;
                Assert.That(status, Is.Not.EqualTo(JointMotionStatus.Blocked), $"Step {steps} was reported as blocked.");
                Assert.That(Mathf.Abs(TwistAngle(applied)), Is.LessThanOrEqualTo(170f + AngleEpsilon), $"Step {steps} left the twist range.");
                Assert.That(PreciseAngle(current, applied), Is.LessThanOrEqualTo(5f + AngleEpsilon), $"Step {steps} exceeded its budget.");
                current = applied;
                steps++;
            }

            Assert.That(status, Is.EqualTo(JointMotionStatus.Reached));
            Assert.That(steps, Is.InRange(64, 65), "320 degrees at 5 degrees per step.");
        }

        [Test]
        public void BallSocketMotionDistance_IsTheLengthOfTheSwingTwistPath()
        {
            // Swing slerps while twist moves along its allowed arc. Sum the rotation between closely
            // spaced points of that path and compare with the closed-form distance.
            var ball = WideTwistBallSocket(150f);
            var random = new System.Random(4321);

            for (int sample = 0; sample < 100; sample++)
            {
                RandomLegalPose(random, 150f, out Quaternion currentSwing, out float currentTwist);
                RandomLegalPose(random, 150f, out Quaternion desiredSwing, out float desiredTwist);
                Quaternion current = currentSwing * Quaternion.AngleAxis(currentTwist, Vector3.forward);
                Quaternion desired = desiredSwing * Quaternion.AngleAxis(desiredTwist, Vector3.forward);

                const int PathSamples = 1000;
                float pathLength = 0f;
                Quaternion previous = current;
                for (int i = 1; i <= PathSamples; i++)
                {
                    float t = i / (float)PathSamples;
                    Quaternion point = Quaternion.Slerp(currentSwing, desiredSwing, t)
                        * Quaternion.AngleAxis(currentTwist + (desiredTwist - currentTwist) * t, Vector3.forward);
                    pathLength += PreciseAngle(previous, point);
                    previous = point;
                }

                Assert.That(ball.GetMotionDistance(current, desired), Is.EqualTo(pathLength).Within(0.1f), $"Sample {sample}");
            }
        }

        [Test]
        public void BallSocketStep_BudgetFractionCoversThatFractionOfThePath()
        {
            // Synchronization gives each joint the budget k * distance and relies on it covering exactly the fraction k.
            var ball = WideTwistBallSocket(150f);
            var random = new System.Random(8765);

            for (int sample = 0; sample < 100; sample++)
            {
                RandomLegalPose(random, 150f, out Quaternion currentSwing, out float currentTwist);
                RandomLegalPose(random, 150f, out Quaternion desiredSwing, out float desiredTwist);
                Quaternion current = currentSwing * Quaternion.AngleAxis(currentTwist, Vector3.forward);
                Quaternion desired = desiredSwing * Quaternion.AngleAxis(desiredTwist, Vector3.forward);
                float distance = ball.GetMotionDistance(current, desired);
                if (distance < 1f)
                    continue;

                JointMotionStep step = ball.StepDeviation(current, desired, 0.3f * distance, out Quaternion applied);

                Assert.That(step.Status, Is.EqualTo(JointMotionStatus.Limited), $"Sample {sample}");
                Assert.That(ball.GetMotionDistance(applied, desired), Is.EqualTo(0.7f * distance).Within(0.05f), $"Sample {sample} did not cover 30% of its path.");
                Assert.That(PreciseAngle(current, applied), Is.LessThanOrEqualTo(0.3f * distance + 0.01f), $"Sample {sample} exceeded its budget.");
            }
        }

        // ---------------------------------------------------------------------------------------
        // Solver integration
        // ---------------------------------------------------------------------------------------

        [TestCase(SolverKind.CCD)]
        [TestCase(SolverKind.FABRIK)]
        [TestCase(SolverKind.Jacobian)]
        public void LimitsDisabled_ApplicationMatchesImmediateApplyTo(SolverKind kind)
        {
            using var arm = PlanarArm.Create(kind);
            arm.Initialize();

            arm.Solver.Mode = SolveMode.SolveOnly;
            arm.Solve();
            IKSolverOutput output = arm.Solver.LastOutput;
            var expectedRotations = new List<Quaternion>(output.WorldRotations);

            IKApplicationStatus status = arm.Solver.ApplyLastOutput(0.1f);

            Assert.That(status.Applied, Is.True);
            Assert.That(status.UsedSpeedLimits, Is.False);
            Assert.That(status.ReachedSolution, Is.True);
            int rotationCount = output.WriteEndRotation ? output.JointCount : output.JointCount - 1;
            for (int i = 0; i < rotationCount; i++)
                Assert.That(Quaternion.Angle(arm.Joints[i].rotation, Quaternion.Normalize(expectedRotations[i])), Is.LessThan(AngleEpsilon), $"Joint {i} rotation differs from ApplyTo.");
        }

        [TestCase(SolverKind.CCD, 1)]
        [TestCase(SolverKind.CCD, 40)]
        [TestCase(SolverKind.FABRIK, 1)]
        [TestCase(SolverKind.FABRIK, 40)]
        [TestCase(SolverKind.Jacobian, 1)]
        [TestCase(SolverKind.Jacobian, 40)]
        public void HingeSpeedLimit_CapsEveryStepIndependentOfIterations(SolverKind kind, int iterations)
        {
            using var arm = PlanarArm.Create(kind);
            arm.SetMaxIterations(iterations);
            arm.LimitAll(90f);
            arm.Initialize();
            arm.Solver.Mode = SolveMode.SolveOnly;

            const float dt = 0.1f;
            float cap = 90f * dt;
            float totalRootTravel = 0f;
            for (int step = 0; step < 6; step++)
            {
                Quaternion[] before = arm.LocalRotations();
                arm.Solve();
                IKApplicationStatus status = arm.Solver.ApplyLastOutput(dt);
                Quaternion[] after = arm.LocalRotations();

                Assert.That(status.UsedSpeedLimits, Is.True);
                for (int i = 0; i < 2; i++)
                    Assert.That(Quaternion.Angle(before[i], after[i]), Is.LessThanOrEqualTo(cap + AngleEpsilon), $"Joint {i} exceeded its cap at step {step}.");
                totalRootTravel += Quaternion.Angle(before[0], after[0]);
            }

            Assert.That(totalRootTravel, Is.GreaterThan(cap), "The limited arm should still move toward the target.");
        }

        [TestCase(SolverKind.CCD)]
        [TestCase(SolverKind.FABRIK)]
        [TestCase(SolverKind.Jacobian)]
        public void SpeedLimitedArm_EventuallyReachesTarget(SolverKind kind)
        {
            using var arm = PlanarArm.Create(kind);
            arm.LimitAll(120f);
            arm.Initialize();
            arm.Solver.Mode = SolveMode.SolveOnly;

            IKApplicationStatus status = default;
            var history = new System.Text.StringBuilder();
            for (int step = 0; step < 300 && !(status.ReachedSolution && status.PositionError < 0.005f); step++)
            {
                arm.Solve();
                status = arm.Solver.ApplyLastOutput(0.05f);
                if (step < 12 || step % 20 == 0)
                {
                    IKSolverOutput o = arm.Solver.LastOutput;
                    float desiredRoot = o.WorldRotations[0].eulerAngles.y;
                    float desiredMid = (Quaternion.Inverse(o.WorldRotations[0]) * o.WorldRotations[1]).eulerAngles.y;
                    history.AppendLine($"step {step}: limited={status.AnyJointLimited} posErr={status.PositionError:F4} solveErr={o.FinalError:F4} desired=({desiredRoot:F1},{desiredMid:F1}) actual=({arm.LocalRotations()[0].eulerAngles.y:F1},{arm.LocalRotations()[1].eulerAngles.y:F1})");
                }
            }

            Assert.That(status.ReachedSolution, Is.True, history.ToString());
            Assert.That(status.PositionError, Is.LessThan(0.005f), history.ToString());
        }

        [Test]
        public void SynchronizedJoints_CoverTheSameFractionWithinTheirOwnCaps()
        {
            float[] fractions = ApplyOneLimitedStep(solverSetting: false, synchronizeArgument: true);

            Assert.That(fractions[1], Is.EqualTo(fractions[0]).Within(0.01f), "Synchronized joints should progress together.");
        }

        [Test]
        public void ApplyLastOutput_UsesItsArgumentNotTheSolverSetting()
        {
            // In SolveOnly the caller of ApplyLastOutput decides; the solver setting only drives SolveAndApply.
            float[] fractions = ApplyOneLimitedStep(solverSetting: true, synchronizeArgument: false);

            Assert.That(fractions[1], Is.GreaterThan(fractions[0] + 0.05f), "Without the argument, each joint should use its own speed.");
        }

        /// Solves a slow-root, fast-tip arm in SolveOnly, applies one 0.1 s step, and returns the
        /// fraction of its remaining travel each joint covered.
        private static float[] ApplyOneLimitedStep(bool solverSetting, bool synchronizeArgument)
        {
            using var arm = PlanarArm.Create(SolverKind.CCD);
            arm.Hinges[0].limitSpeed = true;
            arm.Hinges[0].maxAngularSpeed = 30f;
            arm.Hinges[1].limitSpeed = true;
            arm.Hinges[1].maxAngularSpeed = 300f;
            arm.Initialize();
            arm.Solver.Mode = SolveMode.SolveOnly;
            arm.Solver.SynchronizeLimitedJoints = solverSetting;

            Quaternion[] before = arm.LocalRotations();
            arm.Solve();
            IKSolverOutput output = arm.Solver.LastOutput;
            Quaternion[] desired =
            {
                Quaternion.Inverse(arm.Joints[0].parent.rotation) * output.WorldRotations[0],
                Quaternion.Inverse(output.WorldRotations[0]) * output.WorldRotations[1]
            };
            IKApplicationStatus status = arm.Solver.ApplyLastOutput(0.1f, synchronizeArgument);
            Quaternion[] after = arm.LocalRotations();

            Assert.That(status.AnyJointLimited, Is.True);
            float[] fractions = new float[2];
            for (int i = 0; i < 2; i++)
            {
                float remaining = Quaternion.Angle(before[i], desired[i]);
                float moved = Quaternion.Angle(before[i], after[i]);
                Assert.That(remaining, Is.GreaterThan(1f), $"Joint {i} should need to move.");
                Assert.That(moved, Is.LessThanOrEqualTo((i == 0 ? 3f : 30f) + AngleEpsilon), $"Joint {i} exceeded its cap.");
                fractions[i] = moved / remaining;
            }

            return fractions;
        }

        [Test]
        public void LaggingArm_SolveStaysAnchoredToMovedRoot()
        {
            using var arm = PlanarArm.Create(SolverKind.Jacobian);
            arm.LimitAll(30f);
            arm.Initialize();
            arm.Solver.Mode = SolveMode.SolveOnly;

            arm.Solve();
            Assert.That(arm.Solver.ApplyLastOutput(0.05f).AnyJointLimited, Is.True);

            // Externally driven root motion while the arm still lags its previous solution.
            arm.Joints[0].parent.SetPositionAndRotation(new Vector3(3f, 0f, 0f), Quaternion.Euler(0f, 40f, 0f));
            arm.Solve();

            IKSolverOutput output = arm.Solver.LastOutput;
            Assert.That(Vector3.Distance(output.WorldPositions[0], arm.Joints[0].position), Is.LessThan(Epsilon));
            Assert.That(Vector3.Distance(output.WorldPositions[1], output.WorldPositions[0]), Is.EqualTo(1f).Within(Epsilon));
        }

        [Test]
        public void ZeroDeltaTime_LeavesLimitedJointsStill()
        {
            using var arm = PlanarArm.Create(SolverKind.CCD);
            arm.LimitAll(90f);
            arm.Initialize();
            arm.Solver.Mode = SolveMode.SolveOnly;

            Quaternion[] before = arm.LocalRotations();
            arm.Solve();
            IKApplicationStatus status = arm.Solver.ApplyLastOutput(0f);
            Quaternion[] after = arm.LocalRotations();

            Assert.That(status.AnyJointLimited, Is.True);
            for (int i = 0; i < 2; i++)
                Assert.That(Quaternion.Angle(before[i], after[i]), Is.LessThan(AngleEpsilon));
        }

        [Test]
        public void MixedChain_UnlimitedJointTakesDesiredRelativePose()
        {
            using var arm = PlanarArm.Create(SolverKind.CCD);
            arm.Hinges[0].limitSpeed = true;
            arm.Hinges[0].maxAngularSpeed = 10f;
            arm.Initialize();
            arm.Solver.Mode = SolveMode.SolveOnly;

            arm.Solve();
            IKSolverOutput output = arm.Solver.LastOutput;
            Quaternion desiredRelative = Quaternion.Inverse(output.WorldRotations[0]) * output.WorldRotations[1];
            arm.Solver.ApplyLastOutput(0.1f);
            Quaternion actualRelative = Quaternion.Inverse(arm.Joints[0].rotation) * arm.Joints[1].rotation;

            Assert.That(Quaternion.Angle(desiredRelative, actualRelative), Is.LessThan(0.01f));
            Assert.That(Vector3.Distance(arm.Joints[1].position, arm.Joints[0].position), Is.EqualTo(1f).Within(Epsilon),
                "Segment geometry must be preserved.");
        }

        [Test]
        public void IntermediateHierarchyTransform_IsRespected()
        {
            using var arm = PlanarArm.Create(SolverKind.Jacobian, withIntermediateTransform: true);
            arm.LimitAll(45f);
            arm.Initialize();
            arm.Solver.Mode = SolveMode.SolveOnly;

            for (int step = 0; step < 4; step++)
            {
                Quaternion[] before = arm.LocalRotations();
                arm.Solve();
                arm.Solver.ApplyLastOutput(0.1f);
                Quaternion[] after = arm.LocalRotations();
                for (int i = 0; i < 2; i++)
                    Assert.That(Quaternion.Angle(before[i], after[i]), Is.LessThanOrEqualTo(4.5f + AngleEpsilon));
                Assert.That(Vector3.Distance(arm.Joints[1].position, arm.Joints[0].position), Is.EqualTo(1f).Within(Epsilon));
                Assert.That(Vector3.Distance(arm.Joints[2].position, arm.Joints[1].position), Is.EqualTo(1f).Within(Epsilon));
            }
        }

        [Test]
        public void RestPoseSolving_StillLimitsFromActualPose()
        {
            using var arm = PlanarArm.Create(SolverKind.CCD);
            SetPrivateField(arm.Solver, "solveFromRestPose", true);
            arm.LimitAll(90f);
            arm.Initialize();
            arm.Solver.Mode = SolveMode.SolveOnly;

            // Move the actual arm away from its rest pose before solving.
            arm.Joints[0].rotation = Quaternion.AngleAxis(-70f, Vector3.up);
            Quaternion[] before = arm.LocalRotations();
            arm.Solve();
            arm.Solver.ApplyLastOutput(0.1f);
            Quaternion[] after = arm.LocalRotations();

            Assert.That(Quaternion.Angle(before[0], after[0]), Is.LessThanOrEqualTo(9f + AngleEpsilon));
        }

        [Test]
        public void SolveOnly_DoesNotWriteTransformsAutomatically()
        {
            using var arm = PlanarArm.Create(SolverKind.Jacobian);
            arm.LimitAll(90f);
            arm.Initialize();
            arm.Solver.Mode = SolveMode.SolveOnly;

            Quaternion[] before = arm.LocalRotations();
            arm.Solve();

            Assert.That(arm.Solver.ApplicationStatus.Applied, Is.False);
            Assert.That(float.IsNaN(arm.Solver.ApplicationStatus.PositionError), Is.True);
            Quaternion[] after = arm.LocalRotations();
            for (int i = 0; i < before.Length; i++)
                Assert.That(after[i], Is.EqualTo(before[i]));
        }

        [Test]
        public void SolveAndApply_AppliesAndReportsStatusBeforeSolvedEvent()
        {
            using var arm = PlanarArm.Create(SolverKind.CCD);
            arm.LimitAll(90f);
            arm.Initialize();

            bool applied = false;
            arm.Solver.Solved += _ => applied = arm.Solver.ApplicationStatus.Applied;
            arm.Solve();

            Assert.That(applied, Is.True);
            Assert.That(arm.Solver.ApplicationStatus.UsedSpeedLimits, Is.True);
        }

        [Test]
        public void ApplyLastOutput_IsIgnoredInSolveAndApplyMode()
        {
            using var arm = PlanarArm.Create(SolverKind.CCD);
            arm.Initialize();

            LogAssert.Expect(LogType.Warning, new Regex("ApplyLastOutput is for SolveOnly mode"));
            IKApplicationStatus status = arm.Solver.ApplyLastOutput(0.1f);

            Assert.That(status.Applied, Is.False);
        }

        [Test]
        public void SnapToSolution_RequiresOutputAndAppliesItImmediately()
        {
            using var arm = PlanarArm.Create(SolverKind.Jacobian);
            arm.LimitAll(1f);
            arm.Initialize();
            arm.Solver.Mode = SolveMode.SolveOnly;

            Assert.That(arm.Solver.SnapToSolution(), Is.False, "No output exists before the first solve.");

            arm.Solve();
            Assert.That(arm.Solver.SnapToSolution(), Is.True);

            IKSolverOutput output = arm.Solver.LastOutput;
            for (int i = 0; i < output.JointCount; i++)
                Assert.That(Quaternion.Angle(arm.Joints[i].rotation, Quaternion.Normalize(output.WorldRotations[i])), Is.LessThan(AngleEpsilon));
            Assert.That(arm.Solver.ApplicationStatus.ReachedSolution, Is.True);
        }

        [Test]
        public void DisabledJoint_ContributesNoSpeedLimit()
        {
            using var arm = PlanarArm.Create(SolverKind.CCD);
            arm.LimitAll(1f);
            arm.Hinges[0].jointIsEnabled = false;
            arm.Hinges[1].limitSpeed = false;
            arm.Initialize();
            arm.Solver.Mode = SolveMode.SolveOnly;

            arm.Solve();
            IKApplicationStatus status = arm.Solver.ApplyLastOutput(0.1f);

            Assert.That(status.UsedSpeedLimits, Is.False);
        }

        [Test]
        public void StaticConfiguration_FreezesSpeedLimitSettings()
        {
            using var arm = PlanarArm.Create(SolverKind.CCD);
            SetPrivateField(arm.Solver, "staticSolverConfiguration", true);
            arm.Initialize();
            arm.Solver.Mode = SolveMode.SolveOnly;

            arm.LimitAll(1f);
            arm.Solve();
            IKApplicationStatus status = arm.Solver.ApplyLastOutput(0.1f);

            Assert.That(status.UsedSpeedLimits, Is.False);
        }

        [Test]
        public void UnsupportedJointType_WarnsOnceAndMovesWithoutLimit()
        {
            using var arm = PlanarArm.Create(SolverKind.CCD);
            arm.Hinges[1].limitSpeed = true;
            arm.Initialize();
            arm.Solver.Mode = SolveMode.SolveOnly;
            // Stand in for a custom joint type whose runtime constraint has no motion support. The
            // binding stays in place because the hinge only reconfigures HingeAngularConstraint instances.
            var chain = (SolverChain)typeof(CCDIKSolver)
                .GetField("_chain", BindingFlags.Instance | BindingFlags.NonPublic)
                .GetValue(arm.Solver);
            chain.Joints[1].Angular = new FreeAngularConstraint();

            LogAssert.Expect(LogType.Warning, new Regex("does not support speed limits"));
            arm.Solve();
            IKApplicationStatus status = arm.Solver.ApplyLastOutput(0.1f);
            arm.Solve();
            arm.Solver.ApplyLastOutput(0.1f);

            Assert.That(status.UsedSpeedLimits, Is.False);
            LogAssert.NoUnexpectedReceived();
        }

        [Test]
        public void LimitedApplication_DoesNotAllocateInSteadyState()
        {
            using var arm = PlanarArm.Create(SolverKind.Jacobian);
            arm.LimitAll(30f);
            arm.Initialize();
            arm.Solver.Mode = SolveMode.SolveOnly;
            arm.Solve();
            arm.Solver.ApplyLastOutput(0.01f);

            Assert.That(() => { arm.Solver.ApplyLastOutput(0.01f); },
                UnityEngine.TestTools.Constraints.ConstraintExtensions.AllocatingGCMemory(Is.Not));
        }

        [Test]
        public void ApplyLastOutput_FailsAfterChainTransformIsDestroyed()
        {
            using var arm = PlanarArm.Create(SolverKind.CCD);
            arm.Initialize();
            arm.Solver.Mode = SolveMode.SolveOnly;
            arm.Solve();

            Object.DestroyImmediate(arm.Joints[2].gameObject);

            Assert.That(arm.Solver.ApplyLastOutput(0.1f).Applied, Is.False);
            Assert.That(arm.Solver.SnapToSolution(), Is.False);
        }

        // ---------------------------------------------------------------------------------------
        // Fixture
        // ---------------------------------------------------------------------------------------

        public enum SolverKind
        {
            CCD,
            FABRIK,
            Jacobian
        }

        /// Two full-turn yaw hinges with 1 m segments along +Z; the target sits at 90 degrees to the side.
        private sealed class PlanarArm : IDisposable
        {
            private readonly GameObject _solverRoot;
            private readonly GameObject _target;

            public OpenIKSolverBase Solver { get; private set; }
            public List<Transform> Joints { get; } = new();
            public List<HingeIKJoint> Hinges { get; } = new();
            private readonly List<Transform> _parents = new();

            private PlanarArm(SolverKind kind, bool withIntermediateTransform)
            {
                _solverRoot = new GameObject("SolverRoot");
                _target = new GameObject("Target");
                _target.transform.position = new Vector3(1.6f, 0f, 0.4f);

                Transform root = new GameObject("Root").transform;
                root.SetParent(_solverRoot.transform, false);

                Transform middleParent = root;
                if (withIntermediateTransform)
                {
                    middleParent = new GameObject("Intermediate").transform;
                    middleParent.SetParent(root, false);
                    middleParent.localRotation = Quaternion.Euler(0f, 25f, 0f);
                }

                Transform middle = new GameObject("Middle").transform;
                middle.SetParent(middleParent, false);
                middle.position = new Vector3(0f, 0f, 1f);
                middle.rotation = Quaternion.identity;

                Transform end = new GameObject("End").transform;
                end.SetParent(middle, false);
                end.position = new Vector3(0f, 0f, 2f);

                Joints.Add(root);
                Joints.Add(middle);
                Joints.Add(end);
                _parents.Add(_solverRoot.transform);
                _parents.Add(root);
                _parents.Add(middle);

                foreach (Transform joint in new[] { root, middle })
                {
                    var hinge = joint.gameObject.AddComponent<HingeIKJoint>();
                    hinge.hingeAxis = Vector3.up;
                    hinge.hingeMinAngle = -180f;
                    hinge.hingeMaxAngle = 180f;
                    Hinges.Add(hinge);
                }

                Solver = kind switch
                {
                    SolverKind.CCD => _solverRoot.AddComponent<CCDIKSolver>(),
                    SolverKind.FABRIK => _solverRoot.AddComponent<FABRIKSolver>(),
                    _ => _solverRoot.AddComponent<JacobianIKSolver>()
                };
                SetPrivateField(Solver, "target", _target.transform);
                SetPrivateField(Solver, "chainJoints", new List<Transform>(Joints));

                if (kind == SolverKind.Jacobian)
                {
                    FieldInfo orientationMode = typeof(JacobianIKSolver).GetField("orientationMode", BindingFlags.Instance | BindingFlags.NonPublic);
                    orientationMode.SetValue(Solver, Enum.ToObject(orientationMode.FieldType, 0)); // None
                    SetPrivateField(Solver, "damping", 0.05f);
                }
            }

            public static PlanarArm Create(SolverKind kind, bool withIntermediateTransform = false)
            {
                return new PlanarArm(kind, withIntermediateTransform);
            }

            public void SetMaxIterations(int iterations) => SetPrivateField(Solver, "maxIterations", iterations);

            public void LimitAll(float maxAngularSpeed)
            {
                foreach (HingeIKJoint hinge in Hinges)
                {
                    hinge.limitSpeed = true;
                    hinge.maxAngularSpeed = maxAngularSpeed;
                }
            }

            public void Initialize() => InvokePrivate(Solver, "Awake");

            public void Solve() => InvokePrivate(Solver, "LateUpdate");

            /// Rotation of each chain joint relative to its IK parent.
            public Quaternion[] LocalRotations()
            {
                var result = new Quaternion[Joints.Count];
                for (int i = 0; i < Joints.Count; i++)
                    result[i] = Quaternion.Inverse(_parents[i].rotation) * Joints[i].rotation;
                return result;
            }

            public void Dispose()
            {
                Object.DestroyImmediate(_target);
                Object.DestroyImmediate(_solverRoot);
            }
        }

        private static Quaternion RandomRotation(System.Random random)
        {
            return Quaternion.Euler(
                (float)random.NextDouble() * 360f - 180f,
                (float)random.NextDouble() * 360f - 180f,
                (float)random.NextDouble() * 360f - 180f);
        }

        /// Ball socket with a 120-degree swing cone and the given twist half-angle.
        private static BallSocketAngularConstraint WideTwistBallSocket(float twistHalfAngle)
        {
            float halfSin = Mathf.Sin(Mathf.Deg2Rad * 120f * 0.5f);
            return new BallSocketAngularConstraint(new BallSocketAngularConstraint.Config(Quaternion.identity, halfSin, halfSin, twistHalfAngle));
        }

        /// A swing well inside a 120-degree cone, so the slerp between two of them never needs clamping, and a legal twist.
        private static void RandomLegalPose(System.Random random, float twistHalfAngle, out Quaternion swing, out float twist)
        {
            float radius = 0.45f * Mathf.Sqrt((float)random.NextDouble());
            float angle = (float)random.NextDouble() * 2f * Mathf.PI;
            float x = radius * Mathf.Cos(angle), y = radius * Mathf.Sin(angle);
            swing = new Quaternion(x, y, 0f, Mathf.Sqrt(1f - x * x - y * y));
            twist = ((float)random.NextDouble() * 2f - 1f) * twistHalfAngle;
        }

        private static float TwistAngle(Quaternion deviation)
        {
            return Mathf.DeltaAngle(0f, 2f * Mathf.Atan2(deviation.z, deviation.w) * Mathf.Rad2Deg);
        }

        /// Rotation angle between two orientations, accurate for small angles unlike Quaternion.Angle.
        private static float PreciseAngle(Quaternion a, Quaternion b)
        {
            Quaternion relative = Quaternion.Inverse(a) * b;
            float sinHalf = new Vector3(relative.x, relative.y, relative.z).magnitude;
            return 2f * Mathf.Atan2(sinHalf, Mathf.Abs(relative.w)) * Mathf.Rad2Deg;
        }

        private static void SetPrivateField(object instance, string fieldName, object value)
        {
            FieldInfo field = instance.GetType().GetField(fieldName, BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.That(field, Is.Not.Null, $"Expected private field '{fieldName}' on {instance.GetType().Name}.");
            field.SetValue(instance, value);
        }

        private static void InvokePrivate(object instance, string methodName)
        {
            MethodInfo method = instance.GetType().GetMethod(methodName, BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.That(method, Is.Not.Null, $"Expected private method '{methodName}' on {instance.GetType().Name}.");
            method.Invoke(instance, null);
        }

        private static void AssertVector3(Vector3 expected, Vector3 actual)
        {
            Assert.That(actual.x, Is.EqualTo(expected.x).Within(Epsilon));
            Assert.That(actual.y, Is.EqualTo(expected.y).Within(Epsilon));
            Assert.That(actual.z, Is.EqualTo(expected.z).Within(Epsilon));
        }
    }
}
