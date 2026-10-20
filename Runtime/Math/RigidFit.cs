using Unity.Mathematics;
using UnityEngine;

namespace OpenIK
{
    internal static class RigidFit
    {
        private const int MaxIterations = 8;

        internal static Quaternion FitRotation(Vector3[] rest, Vector3[] goals, float[] weights, int count, Quaternion start)
        {
            double3x3 covariance = double3x3.zero;
            for (int i = 0; i < count; i++)
                covariance += math.max(0f, weights[i]) * Outer((float3)rest[i], (float3)goals[i]);

            return FitRotation(covariance, start);
        }

        internal static void FitPose(Vector3[] localOffsets, Vector3[] goals, float[] weights, int count,
            Vector3 startPosition, Quaternion startRotation, out Vector3 position, out Quaternion rotation)
        {
            double total = 0;
            double3 localSum = double3.zero, goalSum = double3.zero;
            double3x3 covariance = double3x3.zero;
            for (int i = 0; i < count; i++)
            {
                double w = math.max(0f, weights[i]);
                double3 local = (float3)localOffsets[i], goal = (float3)goals[i];
                total += w;
                localSum += w * local;
                goalSum += w * goal;
                covariance += w * Outer(local, goal);
            }

            if (total <= 1e-12)
            {
                position = startPosition;
                rotation = startRotation.normalized;
                return;
            }

            double3 localCenter = localSum / total, goalCenter = goalSum / total;
            covariance -= total * Outer(localCenter, goalCenter);
            rotation = FitRotation(covariance, startRotation);
            position = (float3)(goalCenter - math.mul(new double3x3(new float3x3(rotation)), localCenter));
        }

        // Newton's method from start, so a free twist keeps its incoming value. covariance is the
        // weighted sum of rest * goal^T.
        private static Quaternion FitRotation(double3x3 covariance, Quaternion start)
        {
            quaternion rotation = math.normalize((quaternion)start);
            for (int iteration = 0; iteration < MaxIterations; iteration++)
            {
                // The weighted sum of u * goal^T, where u is a rest offset turned by rotation.
                double3x3 m = math.mul(new double3x3(new float3x3(rotation)), covariance);
                double alignment = m.c0.x + m.c1.y + m.c2.z;
                double3 torque = new(m.c2.y - m.c1.z, m.c0.z - m.c2.x, m.c1.x - m.c0.y);
                double3x3 curvature = alignment * double3x3.identity - 0.5 * (m + math.transpose(m));

                // Damping keeps the step's free twist at zero, for example with two points.
                double damping = math.max(1e-30, math.abs(alignment) * 2e-6);
                double3x3 damped = curvature + damping * double3x3.identity;

                double3 step;
                if (IsPositiveDefinite(damped))
                {
                    step = math.mul(math.inverse(damped), torque);
                }
                else
                {
                    double torqueLength = math.length(torque);
                    double scale = math.abs(curvature.c0.x) + math.abs(curvature.c1.y) + math.abs(curvature.c2.z);
                    if (torqueLength <= 1e-6 * math.max(scale, 1e-30))
                    {
                        // A half turn away from the optimum has no torque, so Newton cannot leave it.
                        // Jump to the closed-form fit and refine from there.
                        rotation = svd.svdRotation(new float3x3(math.transpose(covariance)));
                        continue;
                    }

                    step = torque / torqueLength * math.atan2(torqueLength, alignment);
                }

                double length = math.length(step);
                if (length < 1e-7)
                    break;

                float angle = (float)math.min(length, math.PI * 0.5);
                rotation = math.normalize(math.mul(quaternion.AxisAngle((float3)(step / length), angle), rotation));
            }

            return rotation;
        }
        
        private static bool IsPositiveDefinite(double3x3 m)
        {
            return m.c0.x > 0
                && m.c0.x * m.c1.y - m.c1.x * m.c0.y > 0
                && math.determinant(m) > 0;
        }

        private static double3x3 Outer(double3 a, double3 b) => new(a * b.x, a * b.y, a * b.z);
    }
}
