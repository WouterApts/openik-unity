using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using Object = UnityEngine.Object;

namespace OpenIK.Editor.Tests
{
    public class JacobianSolverTests
    {
        // Checks that rotations stay unit length on a long chain of ball sockets or free joints,
        // which, unlike hinges, do not rebuild the rotation when clamping.
        [TestCase(false)]
        [TestCase(true)]
        public void LongChainWithThreeAxisJoints_StaysFiniteAndReachesTarget(bool ballSockets)
        {
            var root = new GameObject("Root");
            var target = new GameObject("Target");
            try
            {
                var joints = new List<Transform>();
                Transform parent = root.transform;
                for (int i = 0; i < 8; i++)
                {
                    Transform joint = new GameObject($"Joint{i}").transform;
                    joint.SetParent(parent, false);
                    joint.localPosition = i == 0 ? Vector3.zero : Vector3.up;
                    if (ballSockets && i < 7)
                        joint.gameObject.AddComponent<BallSocketIKJoint>().constraintAxis = Vector3.up;
                    joints.Add(joint);
                    parent = joint;
                }

                target.transform.position = new Vector3(3f, 4f, 2f);
                var solver = root.AddComponent<JacobianIKSolver>();
                solver.Target = target.transform;
                solver.UpdateMode = UpdateMode.Manual;
                typeof(OpenIKSolverBase)
                    .GetField("chainJoints", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic)
                    .SetValue(solver, joints);

                for (int frame = 0; frame < 120; frame++)
                    solver.Step(1f / 60f);

                IKSolverOutput output = solver.LastOutput;
                Assert.That(float.IsNaN(output.FinalError), Is.False, "The solve returned NaN.");
                for (int i = 0; i < output.JointCount; i++)
                {
                    Quaternion q = output.WorldRotations[i];
                    float length = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
                    Assert.That(length, Is.EqualTo(1f).Within(1e-3f), $"Joint {i} rotation is not unit length.");
                }
                Assert.That(solver.ApplicationStatus.PositionError, Is.LessThan(0.01f));
            }
            finally
            {
                Object.DestroyImmediate(target);
                Object.DestroyImmediate(root);
            }
        }
    }
}
