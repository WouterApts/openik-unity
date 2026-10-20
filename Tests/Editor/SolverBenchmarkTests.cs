using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using Debug = UnityEngine.Debug;
using Object = UnityEngine.Object;

namespace OpenIK.Editor.Tests
{
    // Times Step on 100 solvers with joint limits and a tight tolerance, after a warm-up. Explicit, so
    // normal runs skip it; run it by name from the Test Runner. The numbers are editor timings, so
    // compare them only with other editor runs of this test on the same machine.
    [Explicit("Benchmark. Run it by name.")]
    public class SolverBenchmarkTests
    {
        private const int SolverCount = 100;
        private const int WarmupFrames = 60;
        private const int MeasuredFrames = 300;
        private const int Repeats = 5;
        private const float DeltaTime = 1f / 60f;
        private const float Tolerance = 1e-5f;

        private static readonly Vector3[] ThreeJoints = { Vector3.zero, new(0f, 1f, 0.1f), new(0f, 1f, -0.1f) };

        private static readonly Vector3[] SixJoints =
        {
            Vector3.zero, new(0f, 0.5f, 0.05f), new(0f, 0.5f, -0.05f), new(0f, 0.5f, 0.05f), new(0f, 0.5f, -0.05f), new(0f, 0.4f, 0f)
        };

        [Test]
        public void TwoBone() => Measure("Two Bone, 3 joints", bench =>
        {
            List<Transform> joints = bench.AddChain(ThreeJoints);
            AddBallSocket(joints[0], 80f, 80f, 60f);
            AddHinge(joints[1], Vector3.right, -150f, 150f);
            var solver = bench.AddSolver<TwoBoneIKSolver>(joints);
            solver.Target = bench.AddTarget(new Vector3(0.3f, 1.4f, 0.3f), new Vector3(0.6f, 0.5f, 0.5f));
        });

        [Test]
        public void CCD() => Measure("CCD, 6 joints, 10 iterations", bench =>
        {
            var solver = bench.AddSolver<CCDIKSolver>(ConstrainedSixJoints(bench));
            solver.Target = bench.AddTarget(new Vector3(0.4f, 1.6f, 0.3f), new Vector3(0.8f, 0.6f, 0.6f));
        });

        [Test]
        public void FABRIK() => Measure("FABRIK, 6 joints, 10 iterations", bench =>
        {
            var solver = bench.AddSolver<FABRIKSolver>(ConstrainedSixJoints(bench));
            solver.Target = bench.AddTarget(new Vector3(0.4f, 1.6f, 0.3f), new Vector3(0.8f, 0.6f, 0.6f));
        });

        [Test]
        public void Jacobian() => Measure("Jacobian, 6 joints, 10 iterations", bench =>
        {
            var solver = bench.AddSolver<JacobianIKSolver>(ConstrainedSixJoints(bench));
            solver.Target = bench.AddTarget(new Vector3(0.4f, 1.6f, 0.3f), new Vector3(0.8f, 0.6f, 0.6f));
        });

        [Test]
        public void MultiFABRIKTorso() => Measure("Multi FABRIK, two-arm torso, 10 iterations", bench =>
        {
            List<Transform> trunk = bench.AddChain(new[] { Vector3.zero, Vector3.up * 0.5f, Vector3.up * 0.5f });
            var solver = bench.AddMultiSolver();
            solver.AddChain("Trunk", trunk).Tolerance = Tolerance;
            AddBallSocket(trunk[2], 50f, 50f, 30f);
            foreach (float side in new[] { -1f, 1f })
            {
                var arm = bench.AddChain(new[] { new Vector3(side * 0.25f, 0f, 0f),
                    new Vector3(side * 0.4f, 0f, 0.1f), new Vector3(side * 0.4f, 0f, -0.1f) }, trunk[2]);
                AddBallSocket(arm[0], 80f, 80f, 60f);
                arm.Insert(0, trunk[2]);
                solver.AddChain("Arm", arm, bench.AddTarget(new Vector3(side * 0.7f, 1f, 0.2f),
                    new Vector3(0.15f, 0.2f, 0.2f))).Tolerance = Tolerance;
            }
        });

        [Test]
        public void MultiFABRIKHand() => Measure("Multi FABRIK, five-finger hand, 10 iterations", bench =>
        {
            List<Transform> arm = bench.AddChain(new[] { Vector3.zero, new Vector3(0f, 0.4f, 0.05f), new Vector3(0f, 0.4f, -0.05f) });
            var solver = bench.AddMultiSolver();
            solver.AddChain("Arm", arm).Tolerance = Tolerance;
            AddBallSocket(arm[2], 50f, 50f, 30f);
            for (int i = 0; i < 5; i++)
            {
                float x = (i - 2) * 0.12f;
                var finger = bench.AddChain(new[] { new Vector3(x, 0.12f, 0f),
                    new Vector3(0f, 0.2f, 0.04f), new Vector3(0f, 0.2f, -0.04f) }, arm[2]);
                AddBallSocket(finger[0], 80f, 80f, 60f);
                finger.Insert(0, arm[2]);
                solver.AddChain("Finger", finger, bench.AddTarget(new Vector3(x, 1.25f, 0.08f),
                    new Vector3(0.04f, 0.06f, 0.08f))).Tolerance = Tolerance;
            }
        });

        private static List<Transform> ConstrainedSixJoints(Bench bench)
        {
            List<Transform> joints = bench.AddChain(SixJoints);
            AddBallSocket(joints[0], 60f, 45f, 30f);
            AddHinge(joints[1], Vector3.right, -100f, 100f);
            AddBallSocket(joints[2], 50f, 70f, 20f);
            AddHinge(joints[3], new Vector3(1f, 0f, 1f), -80f, 120f);
            AddBallSocket(joints[4], 50f, 50f, 20f);
            return joints;
        }

        // Builds SolverCount rigs, steps them through a warm-up, then times several runs of
        // MeasuredFrames frames and logs the fastest, which is the least disturbed by the editor.
        private static void Measure(string name, Action<Bench> buildRig)
        {
            using var bench = new Bench();
            for (int i = 0; i < SolverCount; i++)
            {
                bench.BeginRig(new Vector3(i % 10 * 4f, 0f, i / 10 * 4f));
                buildRig(bench);
            }

            int frame = 0;
            for (int i = 0; i < WarmupFrames; i++)
                bench.StepAll(frame++);

            double bestMilliseconds = double.MaxValue;
            var stopwatch = new Stopwatch();
            for (int repeat = 0; repeat < Repeats; repeat++)
            {
                stopwatch.Restart();
                for (int i = 0; i < MeasuredFrames; i++)
                    bench.StepAll(frame++);
                stopwatch.Stop();
                bestMilliseconds = Math.Min(bestMilliseconds, stopwatch.Elapsed.TotalMilliseconds);
            }

            double perFrame = bestMilliseconds / MeasuredFrames;
            double perStep = perFrame * 1000.0 / bench.SolverCount;
            Debug.Log($"[OpenIK] Benchmark {name}: {perFrame:F3} ms per frame for {bench.SolverCount} solvers, {perStep:F2} us per Step.");
            Assert.Pass($"{name}: {perStep:F2} us per Step");
        }

        private static void AddHinge(Transform joint, Vector3 axis, float min, float max)
        {
            var hinge = joint.gameObject.AddComponent<HingeIKJoint>();
            hinge.hingeAxis = axis;
            hinge.hingeMinAngle = min;
            hinge.hingeMaxAngle = max;
        }

        private static void AddBallSocket(Transform joint, float pitch, float yaw, float twist)
        {
            var ball = joint.gameObject.AddComponent<BallSocketIKJoint>();
            ball.constraintAxis = Vector3.up;
            ball.swingPitchHalfAngle = pitch;
            ball.swingYawHalfAngle = yaw;
            ball.twistHalfAngle = twist;
        }

        // Owns every object of a benchmark. Each rig gets its own root, so the rigs do not overlap.
        private sealed class Bench : IDisposable
        {
            private readonly GameObject _root = new("Benchmark");
            private readonly List<OpenIKSolverBase> _solvers = new();
            private readonly List<MovingTarget> _targets = new();
            private Transform _rig;

            public int SolverCount => _solvers.Count;

            public void BeginRig(Vector3 position)
            {
                _rig = new GameObject($"Rig {_solvers.Count}").transform;
                _rig.SetParent(_root.transform, false);
                _rig.localPosition = position;
                _rig.localRotation = Quaternion.Euler(10f, 25f, -5f);
            }

            // A chain of joints parented to each other, with the given local offsets, below parent or the rig.
            public List<Transform> AddChain(Vector3[] localOffsets, Transform parent = null)
            {
                var joints = new List<Transform>();
                parent ??= _rig;
                for (int i = 0; i < localOffsets.Length; i++)
                {
                    Transform joint = new GameObject($"Joint{i}").transform;
                    joint.SetParent(parent, false);
                    joint.localPosition = localOffsets[i];
                    joints.Add(joint);
                    parent = joint;
                }

                return joints;
            }

            public T AddSolver<T>(List<Transform> joints) where T : SingleChainSolverBase
            {
                var solver = _rig.gameObject.AddComponent<T>();
                SetField(solver, "chainJoints", new List<Transform>(joints));
                solver.Tolerance = Tolerance;
                Configure(solver);
                return solver;
            }

            public MultiFABRIKSolver AddMultiSolver()
            {
                var solver = _rig.gameObject.AddComponent<MultiFABRIKSolver>();
                Configure(solver);
                return solver;
            }

            public void Configure(OpenIKSolverBase solver)
            {
                solver.UpdateMode = UpdateMode.Manual;
                solver.MaxIterations = 10;
                _solvers.Add(solver);
            }

            // A target that moves around center, in the rig's frame, within amplitude on each axis.
            public Transform AddTarget(Vector3 center, Vector3 amplitude)
            {
                Transform target = new GameObject("Target").transform;
                target.SetParent(_rig, false);
                _targets.Add(new MovingTarget(target, center, amplitude, _targets.Count * 0.37f));
                return target;
            }

            public void StepAll(int frame)
            {
                float t = frame * DeltaTime;
                foreach (MovingTarget target in _targets)
                    target.Move(t);
                foreach (OpenIKSolverBase solver in _solvers)
                    solver.Step(DeltaTime);
            }

            public void Dispose()
            {
                Object.DestroyImmediate(_root);
            }
        }

        private readonly struct MovingTarget
        {
            private readonly Transform _transform;
            private readonly Vector3 _center;
            private readonly Vector3 _amplitude;
            private readonly float _phase;

            public MovingTarget(Transform transform, Vector3 center, Vector3 amplitude, float phase)
            {
                _transform = transform;
                _center = center;
                _amplitude = amplitude;
                _phase = phase;
            }

            public void Move(float t)
            {
                t += _phase;
                _transform.localPosition = _center + Vector3.Scale(_amplitude,
                    new Vector3(Mathf.Sin(2.6f * t), Mathf.Sin(1.8f * t + 0.7f), Mathf.Cos(1.4f * t)));
            }
        }

        private static void SetField(object instance, string fieldName, object value)
        {
            for (Type type = instance.GetType(); type != null; type = type.BaseType)
            {
                FieldInfo field = type.GetField(fieldName, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.DeclaredOnly);
                if (field == null)
                    continue;

                field.SetValue(instance, value);
                return;
            }

            Assert.Fail($"Expected field '{fieldName}' on {instance.GetType().Name}.");
        }
    }
}
