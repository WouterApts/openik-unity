using System;
using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using Object = UnityEngine.Object;

namespace OpenIK.Editor.Tests
{
    // A root joint without a parent transform is constrained relative to world space. Each root here has
    // a hinge narrow enough that it cannot turn far enough to reach the target.
    public class ParentlessRootTests
    {
        private const float NarrowLimit = 1f;
        private const float AngleEpsilon = 0.01f;

        [TestCase(typeof(FABRIKSolver))]
        [TestCase(typeof(CCDIKSolver))]
        [TestCase(typeof(JacobianIKSolver))]
        [TestCase(typeof(TwoBoneIKSolver))]
        public void ParentlessRoot_StaysWithinItsLimitsInWorldSpace(Type solverType)
        {
            var created = new List<GameObject>();
            try
            {
                // Reaching this target takes a turn around the hinge axis far past its limit. The middle
                // joint stays near (0, 1, 0) while the root cannot turn, so the end joint cannot get
                // closer than about 0.58.
                Transform root = CreateJoint("Root", null, Vector3.zero, created);
                Transform middle = CreateJoint("Middle", root, Vector3.up, created);
                Transform end = CreateJoint("End", middle, Vector3.up, created);
                AddNarrowHinge(root);

                Transform target = new GameObject("Target").transform;
                created.Add(target.gameObject);
                target.position = new Vector3(0f, 0.5f, 1.5f);

                var solverObject = new GameObject("Solver");
                created.Add(solverObject);
                var solver = (SingleChainSolverBase)solverObject.AddComponent(solverType);
                solver.Target = target;
                solver.MaxIterations = 50;
                SetField(solver, "chainJoints", new List<Transform> { root, middle, end });

                Assert.That(solver.Solve(), Is.True);
                Quaternion rootRotation = solver.LastSolution.WorldRotations[0];
                Assert.That(Quaternion.Angle(rootRotation, Quaternion.identity), Is.LessThanOrEqualTo(NarrowLimit + AngleEpsilon));
                Assert.That(solver.LastSolution.FinalError, Is.GreaterThan(0.5f));
            }
            finally
            {
                foreach (GameObject gameObject in created)
                {
                    Object.DestroyImmediate(gameObject);
                }
            }
        }

        private static Transform CreateJoint(string name, Transform parent, Vector3 localPosition, List<GameObject> created)
        {
            Transform joint = new GameObject(name).transform;
            created.Add(joint.gameObject);
            joint.SetParent(parent, false);
            joint.localPosition = localPosition;
            return joint;
        }

        private static void AddNarrowHinge(Transform joint)
        {
            var hinge = joint.gameObject.AddComponent<HingeIKJoint>();
            hinge.hingeAxis = Vector3.right;
            hinge.hingeMinAngle = -NarrowLimit;
            hinge.hingeMaxAngle = NarrowLimit;
        }

        private static void SetField(object instance, string fieldName, object value)
        {
            for (Type type = instance.GetType(); type != null; type = type.BaseType)
            {
                FieldInfo field = type.GetField(fieldName, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.DeclaredOnly);
                if (field != null)
                {
                    field.SetValue(instance, value);
                    return;
                }
            }

            throw new MissingFieldException(instance.GetType().Name, fieldName);
        }
    }
}
