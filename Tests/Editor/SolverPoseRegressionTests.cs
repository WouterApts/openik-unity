using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;
using System.Text;
using NUnit.Framework;
using UnityEngine;
using Object = UnityEngine.Object;

namespace OpenIK.Editor.Tests
{
    // Characterization tests: each scenario drives a solver along a fixed target path and compares
    // the resulting joint poses against a recorded baseline. They catch unintended changes to solver
    // output during refactors. When a solver's behavior changes on purpose, delete the baseline file
    // and run the tests once to record a new one.
    public class SolverPoseRegressionTests
    {
        private const string BaselineFileName = "SolverPoseBaseline.txt";
        private const int FrameCount = 120;
        private const int SampleInterval = 10;
        private const float DeltaTime = 1f / 60f;
        private const float PositionTolerance = 1e-5f;
        private const float RotationTolerance = 1e-5f;

        private enum Kind { FABRIK, CCD, Jacobian }

        private sealed class Scenario
        {
            public string Name;
            public Kind Kind;
            public Vector3[] LocalOffsets;
            public Action<List<Transform>> AddJoints;
            public Action<OpenIKSolverBase> Configure;
            public bool SpeedLimited;
            public bool Synchronize;
            public Vector3 PathCenter;
            public Vector3 PathAmplitude;
        }

        private static readonly Vector3[] FourJoints = { Vector3.zero, new(0f, 1f, 0f), new(0f, 0.8f, 0f), new(0f, 0.6f, 0f) };
        private static readonly Vector3[] FiveJoints = { Vector3.zero, new(0f, 0.7f, 0f), new(0f, 0.7f, 0.2f), new(0f, 0.6f, 0f), new(0f, 0.4f, 0f) };

        private static IEnumerable<Scenario> Scenarios()
        {
            // FABRIK
            yield return Free("FABRIK_Free", Kind.FABRIK);
            yield return Constrained("FABRIK_HingeBall", Kind.FABRIK);
            yield return Slider("FABRIK_Slider", Kind.FABRIK);
            yield return new Scenario
            {
                Name = "FABRIK_Singularity",
                Kind = Kind.FABRIK,
                LocalOffsets = FourJoints,
                AddJoints = joints => AddHinge(joints[2], Vector3.right, -120f, 120f),
                // Target sweeps along the straight chain's axis, crossing the fully extended pose.
                PathCenter = new Vector3(0f, 2.1f, 0f),
                PathAmplitude = new Vector3(0.02f, 0.6f, 0.02f)
            };
            yield return RestPose(Constrained("FABRIK_RestPose", Kind.FABRIK));
            yield return Limited(Constrained("FABRIK_SpeedLimited", Kind.FABRIK), synchronize: false);
            yield return Limited(Constrained("FABRIK_SpeedLimitedSync", Kind.FABRIK), synchronize: true);
            yield return Limited(Slider("FABRIK_SliderSpeedLimited", Kind.FABRIK), synchronize: false);

            // CCD
            yield return Free("CCD_Free", Kind.CCD);
            yield return Constrained("CCD_HingeBall", Kind.CCD);
            Scenario halfStep = Constrained("CCD_HalfStep", Kind.CCD);
            halfStep.Configure = solver => SetField(solver, "rotationStep", 0.5f);
            yield return halfStep;
            yield return RestPose(Constrained("CCD_RestPose", Kind.CCD));
            yield return Limited(Constrained("CCD_SpeedLimited", Kind.CCD), synchronize: false);

            // Jacobian
            yield return Constrained("Jacobian_FullRotation", Kind.Jacobian);
            Scenario boneDirection = Constrained("Jacobian_BoneDirection", Kind.Jacobian);
            boneDirection.Configure = solver => SetEnumField(solver, "orientationMode", 2);
            yield return boneDirection;
            Scenario positionOnly = Free("Jacobian_PositionOnly", Kind.Jacobian);
            positionOnly.Configure = solver => SetEnumField(solver, "orientationMode", 0);
            yield return positionOnly;
            yield return Limited(Slider("Jacobian_SliderSpeedLimitedSync", Kind.Jacobian), synchronize: true);
        }

        private static Scenario Free(string name, Kind kind) => new()
        {
            Name = name,
            Kind = kind,
            LocalOffsets = FourJoints,
            PathCenter = new Vector3(0.6f, 1.4f, 0.3f),
            // Large enough that the target leaves the chain's reach for part of the path.
            PathAmplitude = new Vector3(1.6f, 1.4f, 1.2f)
        };

        private static Scenario Constrained(string name, Kind kind) => new()
        {
            Name = name,
            Kind = kind,
            LocalOffsets = FiveJoints,
            AddJoints = joints =>
            {
                AddBallSocket(joints[0], 60f, 45f, 30f);
                AddHinge(joints[1], Vector3.right, -100f, 100f);
                AddHinge(joints[2], new Vector3(1f, 0f, 1f), -80f, 120f);
                AddBallSocket(joints[3], 50f, 70f, 20f);
            },
            PathCenter = new Vector3(0.5f, 1.5f, 0.4f),
            PathAmplitude = new Vector3(1.2f, 1.1f, 1.0f)
        };

        private static Scenario Slider(string name, Kind kind) => new()
        {
            Name = name,
            Kind = kind,
            LocalOffsets = FourJoints,
            AddJoints = joints =>
            {
                AddHinge(joints[0], Vector3.forward, -150f, 150f);
                var slider = joints[2].gameObject.AddComponent<SliderIKJoint>();
                slider.slideAxis = Vector3.up;
                slider.minLength = 0.3f;
                slider.maxLength = 1.6f;
                slider.maxLinearSpeed = 0.8f;
                AddHinge(joints[1], Vector3.forward, -120f, 120f);
            },
            PathCenter = new Vector3(0.4f, 1.6f, 0f),
            PathAmplitude = new Vector3(1.3f, 1.2f, 0.3f)
        };

        private static Scenario RestPose(Scenario scenario)
        {
            Action<OpenIKSolverBase> configure = scenario.Configure;
            scenario.Configure = solver =>
            {
                configure?.Invoke(solver);
                SetField(solver, "solveFromRestPose", true);
            };
            return scenario;
        }

        private static Scenario Limited(Scenario scenario, bool synchronize)
        {
            Action<List<Transform>> addJoints = scenario.AddJoints;
            scenario.AddJoints = joints =>
            {
                addJoints?.Invoke(joints);
                foreach (Transform joint in joints)
                {
                    if (joint.TryGetComponent(out ConstrainedJoint constrained))
                    {
                        constrained.limitSpeed = true;
                        constrained.maxAngularSpeed = 120f;
                    }
                }
            };
            scenario.SpeedLimited = true;
            scenario.Synchronize = synchronize;
            return scenario;
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

        [Test]
        public void SolverPoses_MatchRecordedBaseline()
        {
            string recorded = RecordAll();
            string baselinePath = GetBaselinePath();

            if (!File.Exists(baselinePath))
            {
                File.WriteAllText(baselinePath, recorded);
                Assert.Inconclusive($"No baseline existed. Recorded a new one at {baselinePath}.");
            }

            Compare(File.ReadAllText(baselinePath), recorded);
        }

        private static string RecordAll()
        {
            var builder = new StringBuilder();
            foreach (Scenario scenario in Scenarios())
                Record(scenario, builder);
            return builder.ToString();
        }

        private static void Record(Scenario scenario, StringBuilder builder)
        {
            var solverRoot = new GameObject(scenario.Name);
            var target = new GameObject("Target");
            try
            {
                // A rotated root parent exercises the root joint's constraint frame.
                solverRoot.transform.SetPositionAndRotation(new Vector3(0.1f, -0.2f, 0.05f), Quaternion.Euler(10f, 25f, -5f));

                var joints = new List<Transform>();
                Transform parent = solverRoot.transform;
                for (int i = 0; i < scenario.LocalOffsets.Length; i++)
                {
                    Transform joint = new GameObject($"Joint{i}").transform;
                    joint.SetParent(parent, false);
                    joint.localPosition = scenario.LocalOffsets[i];
                    joints.Add(joint);
                    parent = joint;
                }

                scenario.AddJoints?.Invoke(joints);

                OpenIKSolverBase solver = scenario.Kind switch
                {
                    Kind.FABRIK => solverRoot.AddComponent<FABRIKSolver>(),
                    Kind.CCD => solverRoot.AddComponent<CCDIKSolver>(),
                    _ => solverRoot.AddComponent<JacobianIKSolver>()
                };
                SetField(solver, "target", target.transform);
                SetField(solver, "chainJoints", new List<Transform>(joints));
                SetField(solver, "maxIterations", 10);
                SetField(solver, "synchronizeLimitedJoints", scenario.Synchronize);
                scenario.Configure?.Invoke(solver);
                SolverDriver.Initialize(solver);

                builder.Append("# ").Append(scenario.Name).Append('\n');
                for (int frame = 0; frame < FrameCount; frame++)
                {
                    float t = frame * DeltaTime;
                    target.transform.SetPositionAndRotation(
                        solverRoot.transform.TransformPoint(scenario.PathCenter + Vector3.Scale(scenario.PathAmplitude,
                            new Vector3(Mathf.Sin(1.3f * t * 2f), Mathf.Sin(0.9f * t * 2f + 0.7f), Mathf.Cos(0.7f * t * 2f)))),
                        Quaternion.Euler(40f * Mathf.Sin(1.1f * t), 70f * Mathf.Cos(0.8f * t), 25f * Mathf.Sin(0.5f * t)));

                    SolverDriver.Step(solver, scenario.SpeedLimited, DeltaTime);

                    if (frame % SampleInterval != SampleInterval - 1)
                        continue;

                    IKSolverOutput output = solver.LastOutput;
                    builder.Append("f ").Append(frame).Append(' ')
                        .Append(output.IterationsUsed).Append(' ')
                        .Append(Format(output.FinalError)).Append(' ')
                        .Append(output.Converged ? 1 : 0).Append('\n');
                    foreach (Transform joint in joints)
                    {
                        joint.GetPositionAndRotation(out Vector3 p, out Quaternion q);
                        builder.Append("j ")
                            .Append(Format(p.x)).Append(' ').Append(Format(p.y)).Append(' ').Append(Format(p.z)).Append(' ')
                            .Append(Format(q.x)).Append(' ').Append(Format(q.y)).Append(' ').Append(Format(q.z)).Append(' ')
                            .Append(Format(q.w)).Append('\n');
                    }
                }
            }
            finally
            {
                Object.DestroyImmediate(target);
                Object.DestroyImmediate(solverRoot);
            }
        }

        private static void Compare(string expected, string actual)
        {
            string[] expectedLines = expected.Replace("\r\n", "\n").Split('\n');
            string[] actualLines = actual.Split('\n');
            Assert.That(actualLines.Length, Is.EqualTo(expectedLines.Length), "Recorded line count differs from the baseline.");

            string scenario = "";
            float maxPositionDelta = 0f, maxRotationDelta = 0f;
            var failures = new List<string>();
            for (int i = 0; i < expectedLines.Length; i++)
            {
                string e = expectedLines[i], a = actualLines[i];
                if (e.StartsWith("# "))
                {
                    Assert.That(a, Is.EqualTo(e), "Scenario order differs from the baseline.");
                    scenario = e.Substring(2);
                    continue;
                }

                if (e.StartsWith("f "))
                {
                    // Iteration counts and convergence must match exactly; the error within tolerance.
                    string[] ef = e.Split(' '), af = a.Split(' ');
                    if (ef[1] != af[1] || ef[2] != af[2] || ef[4] != af[4] ||
                        Mathf.Abs(Parse(ef[3]) - Parse(af[3])) > PositionTolerance)
                        failures.Add($"{scenario} frame {ef[1]}: stats '{e}' became '{a}'");
                    continue;
                }

                if (!e.StartsWith("j "))
                    continue;

                float[] ev = ParseValues(e), av = ParseValues(a);
                float positionDelta = Mathf.Max(Mathf.Abs(ev[0] - av[0]), Mathf.Abs(ev[1] - av[1]), Mathf.Abs(ev[2] - av[2]));
                // q and -q are the same rotation.
                float sign = ev[3] * av[3] + ev[4] * av[4] + ev[5] * av[5] + ev[6] * av[6] < 0f ? -1f : 1f;
                float rotationDelta = 0f;
                for (int c = 3; c < 7; c++)
                    rotationDelta = Mathf.Max(rotationDelta, Mathf.Abs(ev[c] - sign * av[c]));

                maxPositionDelta = Mathf.Max(maxPositionDelta, positionDelta);
                maxRotationDelta = Mathf.Max(maxRotationDelta, rotationDelta);
                if (positionDelta > PositionTolerance || rotationDelta > RotationTolerance)
                    failures.Add($"{scenario} line {i + 1}: '{e}' became '{a}'");
            }

            Debug.Log($"[OpenIK] Pose regression: max position delta {maxPositionDelta:G3}, max rotation delta {maxRotationDelta:G3}.");
            Assert.That(failures, Is.Empty, string.Join("\n", failures.GetRange(0, Mathf.Min(failures.Count, 20))));
        }

        private static float[] ParseValues(string line)
        {
            string[] parts = line.Split(' ');
            var values = new float[parts.Length - 1];
            for (int i = 1; i < parts.Length; i++)
                values[i - 1] = Parse(parts[i]);
            return values;
        }

        private static string Format(float value) => value.ToString("G9", CultureInfo.InvariantCulture);

        private static float Parse(string value) => float.Parse(value, CultureInfo.InvariantCulture);

        private static string GetBaselinePath()
        {
            var package = UnityEditor.PackageManager.PackageInfo.FindForAssembly(typeof(SolverPoseRegressionTests).Assembly);
            Assert.That(package, Is.Not.Null, "Could not resolve the OpenIK package path.");
            return Path.Combine(package.resolvedPath, "Tests", "Editor", BaselineFileName);
        }

        private static void SetField(object instance, string fieldName, object value)
        {
            FieldInfo field = FindField(instance.GetType(), fieldName);
            Assert.That(field, Is.Not.Null, $"Expected field '{fieldName}' on {instance.GetType().Name}.");
            field.SetValue(instance, value);
        }

        private static void SetEnumField(object instance, string fieldName, int value)
        {
            FieldInfo field = FindField(instance.GetType(), fieldName);
            Assert.That(field, Is.Not.Null, $"Expected field '{fieldName}' on {instance.GetType().Name}.");
            field.SetValue(instance, Enum.ToObject(field.FieldType, value));
        }

        private static FieldInfo FindField(Type type, string fieldName)
        {
            for (; type != null; type = type.BaseType)
            {
                FieldInfo field = type.GetField(fieldName, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public);
                if (field != null)
                    return field;
            }

            return null;
        }

        // Drives a solver the way the runtime does: automatic application in LateUpdate when no
        // speed limits are involved, and manual application with a fixed delta time otherwise.
        private static class SolverDriver
        {
            public static void Initialize(OpenIKSolverBase solver) => Invoke(solver, "Awake");

            public static void Step(OpenIKSolverBase solver, bool manualApply, float deltaTime)
            {
                if (!manualApply)
                {
                    solver.ApplyMode = ApplyMode.Automatic;
                    Invoke(solver, "LateUpdate");
                    return;
                }

                solver.ApplyMode = ApplyMode.Manual;
                solver.Solve();
                solver.Apply(deltaTime);
            }

            private static void Invoke(object instance, string methodName)
            {
                for (Type type = instance.GetType(); type != null; type = type.BaseType)
                {
                    MethodInfo method = type.GetMethod(methodName, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.DeclaredOnly);
                    if (method == null)
                        continue;

                    method.Invoke(instance, null);
                    return;
                }

                Assert.Fail($"Expected method '{methodName}' on {instance.GetType().Name}.");
            }
        }
    }
}
