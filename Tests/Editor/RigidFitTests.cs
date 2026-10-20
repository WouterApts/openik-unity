using System;
using NUnit.Framework;
using UnityEngine;

namespace OpenIK.Editor.Tests
{
    public class RigidFitTests
    {
        private static readonly Vector3[] Points =
        {
            new(-0.4f, 0f, 0f), new(0.4f, 0f, 0f), new(0f, 0.12f, 0f), new(0f, 0f, 0.2f)
        };

        [TestCase(10f)]
        [TestCase(90f)]
        [TestCase(180f)]
        public void ExactPoints_RecoverPose(float angle)
        {
            Quaternion expected = Quaternion.AngleAxis(angle, Vector3.right);
            Vector3 position = new(1f, -2f, 3f);
            var goals = new Vector3[4];
            for (int i = 0; i < 4; i++) goals[i] = position + expected * Points[i];
            RigidFit.FitPose(Points, goals, new[] { 1f, 1f, 1f, 1f }, 4,
                Vector3.zero, Quaternion.identity, out var p, out var r);
            Assert.That(Vector3.Distance(p, position), Is.LessThan(1e-5f));
            AssertRotation(r, expected, 1e-4f);
        }

        [Test]
        public void RandomExactFits_ConvergeFromNinetyDegreesAway()
        {
            var random = new System.Random(417);
            var rest = new Vector3[5];
            var goals = new Vector3[5];
            var weights = new[] { 1f, 0.2f, 0.7f, 1f, 0.5f };
            for (int sample = 0; sample < 100; sample++)
            {
                Vector3 axis = RandomVector(random).normalized;
                Quaternion expected = Quaternion.AngleAxis(90f, axis);
                for (int i = 0; i < 5; i++)
                {
                    rest[i] = RandomVector(random);
                    goals[i] = expected * rest[i] + Vector3.one;
                }
                RigidFit.FitPose(rest, goals, weights, 5, Vector3.zero, Quaternion.identity,
                    out var p, out var r);
                Assert.That(Vector3.Distance(p, Vector3.one), Is.LessThan(2e-4f), $"sample {sample}");
                AssertRotation(r, expected, 2e-4f);
            }
        }

        [Test]
        public void NoisyWeightedFit_IsALocalMinimum()
        {
            var goals = new Vector3[4];
            var weights = new[] { 1f, 0.1f, 0.7f, 0.4f };
            Quaternion expected = Quaternion.Euler(20f, 40f, -10f);
            for (int i = 0; i < 4; i++) goals[i] = expected * Points[i] + new Vector3(i * 0.006f, -i * 0.01f, 0.002f);
            RigidFit.FitPose(Points, goals, weights, 4, Vector3.zero, Quaternion.identity, out var p, out var r);
            float error = Error(p, r, goals, weights);
            foreach (var axis in new[] { Vector3.right, Vector3.up, Vector3.forward })
            foreach (float sign in new[] { -1f, 1f })
            {
                Assert.That(Error(p, Quaternion.AngleAxis(sign * 0.1f, axis) * r, goals, weights), Is.GreaterThanOrEqualTo(error - 1e-8f));
                Assert.That(Error(p + axis * sign * 0.001f, r, goals, weights), Is.GreaterThanOrEqualTo(error));
            }
        }

        [Test]
        public void TwoPoints_PreserveIncomingTwist()
        {
            var rest = new[] { Vector3.left, Vector3.right };
            Quaternion start = Quaternion.AngleAxis(57f, Vector3.right);
            Quaternion swing = Quaternion.AngleAxis(35f, Vector3.forward);
            var goals = new[] { swing * rest[0] + Vector3.up, swing * rest[1] + Vector3.up };
            RigidFit.FitPose(rest, goals, new[] { 1f, 1f }, 2, Vector3.zero, start, out var p, out var r);
            AssertRotation(r, swing * start, 1e-4f);
            Assert.That(Vector3.Distance(p, Vector3.up), Is.LessThan(1e-5f));
        }

        [Test]
        public void Weights_MoveTheTranslationTowardTheHeavierPoint()
        {
            var rest = new[] { Vector3.zero, Vector3.zero };
            var goals = new[] { Vector3.zero, Vector3.right };
            RigidFit.FitPose(rest, goals, new[] { 1f, 3f }, 2, Vector3.zero, Quaternion.identity, out var p, out var r);
            Assert.That(p.x, Is.EqualTo(0.75f).Within(1e-6f));
            AssertRotation(r, Quaternion.identity, 1e-6f);
        }

        [Test]
        public void DegenerateFits_ReturnFiniteStartRotationAndZeroWeightsKeepPosition()
        {
            var rest = new[] { Vector3.zero, Vector3.zero, Vector3.zero };
            var goals = new[] { Vector3.one, Vector3.one, Vector3.one };
            Quaternion start = Quaternion.Euler(10f, 20f, 30f);
            foreach (var weights in new[] { new[] { 0f, 0f, 0f }, new[] { 1f, 1f, 1f }, new[] { 1f, 0f, 0f } })
            {
                RigidFit.FitPose(rest, goals, weights, 3, Vector3.left, start, out var p, out var r);
                AssertRotation(r, start, 1e-6f);
                Assert.That(p, Is.EqualTo(weights[0] == 0f ? Vector3.left : Vector3.one));
            }
        }

        [Test]
        public void Fits_AllocateNothingAfterWarmup()
        {
            var weights = new[] { 1f, 1f, 1f, 1f };
            for (int i = 0; i < 10; i++) Fit();
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < 100; i++) Fit();
            Assert.That(GC.GetAllocatedBytesForCurrentThread() - before, Is.Zero);
            void Fit() => RigidFit.FitPose(Points, Points, weights, 4, Vector3.zero, Quaternion.identity, out _, out _);
        }

        private static float Error(Vector3 p, Quaternion r, Vector3[] goals, float[] weights)
        {
            float error = 0f;
            for (int i = 0; i < 4; i++) error += weights[i] * (p + r * Points[i] - goals[i]).sqrMagnitude;
            return error;
        }

        private static Vector3 RandomVector(System.Random random) => new(
            (float)random.NextDouble() * 2f - 1f, (float)random.NextDouble() * 2f - 1f, (float)random.NextDouble() * 2f - 1f);

        private static void AssertRotation(Quaternion actual, Quaternion expected, float tolerance)
        {
            Assert.That(Vector3.Distance(actual * Vector3.right, expected * Vector3.right), Is.LessThan(tolerance));
            Assert.That(Vector3.Distance(actual * Vector3.up, expected * Vector3.up), Is.LessThan(tolerance));
        }
    }
}
