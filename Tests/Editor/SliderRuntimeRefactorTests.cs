using System.Collections.Generic;
using System.Reflection;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

namespace OpenIK.Editor.Tests
{
    public class SliderRuntimeRefactorTests
    {
        private const float Epsilon = 1e-4f;

        [Test]
        public void SliderSegmentConstraint_ComputesExpectedSliderDecomposition()
        {
            var parent = new GameObject("Parent");
            var child = new GameObject("Child");

            try
            {
                Vector3 localOffset = new(0f, 1f, 0f);
                var context = new ISegmentConstraint.SetupData(
                    localOffset,
                    localOffset.magnitude,
                    parent.transform.rotation,
                    child.transform.rotation,
                    true);
                var config = new SliderSegmentConstraint.Config(Vector3.up, 0f, 2f);
                var segment = new SliderSegmentConstraint(context, config);

                AssertVector3(new Vector3(0f, 0f, 0f), segment.FixedOffsetLocal);
                AssertVector3(Vector3.up, segment.SlideAxisParentLocal);
                Assert.That(segment.RestSlideOffset, Is.EqualTo(1f).Within(Epsilon));
                Assert.That(segment.RestSlide, Is.EqualTo(0f).Within(Epsilon));
                Assert.That(segment.MaxReach, Is.EqualTo(3f).Within(Epsilon));
            }
            finally
            {
                Object.DestroyImmediate(parent);
                Object.DestroyImmediate(child);
            }
        }

        [Test]
        public void LocalOffsetComposition_RoundTripsCurrentSlide()
        {
            var parent = new GameObject("Parent");
            var child = new GameObject("Child");

            try
            {
                var segment = new SliderSegmentConstraint(
                    new ISegmentConstraint.SetupData(
                        new Vector3(0f, 1f, 0f),
                        1f,
                        parent.transform.rotation,
                        child.transform.rotation,
                        true),
                    new SliderSegmentConstraint.Config(Vector3.up, 0f, 2f));

                segment.SetCurrentSlide(1.25f);
                Vector3 currentOffset = segment.GetCurrentLocalOffset();

                AssertVector3(new Vector3(0f, 2.25f, 0f), currentOffset);

                segment.SyncFromLocalOffset(currentOffset);

                Assert.That(segment.CurrentSlide, Is.EqualTo(1.25f).Within(Epsilon));
            }
            finally
            {
                Object.DestroyImmediate(parent);
                Object.DestroyImmediate(child);
            }
        }

        [Test]
        public void FabrikSolver_EnabledSliderUsesExpandedSliderReach()
        {
            using var chain = CreateSliderChain(jointIsEnabled: true);

            InvokePrivate(chain.FabrikSolver, "Awake");

            float chainLength = GetChainLength(chain.FabrikSolver);
            SliderSegmentConstraint segment = GetSegment<SliderSegmentConstraint>(chain.FabrikSolver, 1);

            Assert.That(chainLength, Is.EqualTo(4f).Within(Epsilon));
            Assert.That(segment.MaxReach, Is.EqualTo(3f).Within(Epsilon));
        }

        [Test]
        public void FabrikSolver_DisabledSliderFallsBackToRigidReach()
        {
            using var chain = CreateSliderChain(jointIsEnabled: false);

            InvokePrivate(chain.FabrikSolver, "Awake");

            float chainLength = GetChainLength(chain.FabrikSolver);
            SolverJoint[] joints = GetSolverJoints(chain.FabrikSolver);

            Assert.That(chainLength, Is.EqualTo(2f).Within(Epsilon));
            Assert.That(joints[1].Segment, Is.TypeOf<RigidSegmentConstraint>());
            Assert.That(joints[1].Angular, Is.TypeOf<FreeAngularConstraint>());
        }

        [Test]
        public void JacobianSolver_MatchesFabrikSliderStateForEnabledSlider()
        {
            using var chain = CreateSliderChain(jointIsEnabled: true);

            InvokePrivate(chain.FabrikSolver, "Awake");
            InvokePrivate(chain.JacobianSolver, "Awake");

            SliderSegmentConstraint fabrikSegment = GetSegment<SliderSegmentConstraint>(chain.FabrikSolver, 1);
            SliderSegmentConstraint jacobianSegment = GetSegment<SliderSegmentConstraint>(chain.JacobianSolver, 1);

            AssertVector3(fabrikSegment.FixedOffsetLocal, jacobianSegment.FixedOffsetLocal);
            AssertVector3(fabrikSegment.SlideAxisParentLocal, jacobianSegment.SlideAxisParentLocal);
            Assert.That(jacobianSegment.RestSlideOffset, Is.EqualTo(fabrikSegment.RestSlideOffset).Within(Epsilon));
            Assert.That(jacobianSegment.RestSlide, Is.EqualTo(fabrikSegment.RestSlide).Within(Epsilon));
            Assert.That(jacobianSegment.MaxReach, Is.EqualTo(fabrikSegment.MaxReach).Within(Epsilon));
        }

        [Test]
        public void SolverJoint_DefaultsToRigidSegmentAndFreeAngularWhenNoJointComponent()
        {
            using var chain = CreateUnconstrainedChain();

            InvokePrivate(chain.FabrikSolver, "Awake");

            SolverJoint[] joints = GetSolverJoints(chain.FabrikSolver);

            Assert.That(joints[1].Segment, Is.TypeOf<RigidSegmentConstraint>());
            Assert.That(joints[1].Angular, Is.TypeOf<FreeAngularConstraint>());
        }

        [Test]
        public void JacobianSolver_FreeAngularExposesThreeRotationDofsForNonEndJoint()
        {
            using var chain = CreateUnconstrainedChain();

            InvokePrivate(chain.JacobianSolver, "Awake");

            int dofCount = (int)GetPrivateField(chain.JacobianSolver, "_dofCount");

            Assert.That(dofCount, Is.EqualTo(6));
        }

        [Test]
        public void ConstraintDofCounts_MatchExpectedJointCapabilities()
        {
            var segmentContext = new ISegmentConstraint.SetupData(Vector3.forward, 1f, Quaternion.identity, Quaternion.identity, true);

            Assert.That(new RigidSegmentConstraint().DofCount, Is.EqualTo(0));
            Assert.That(new SliderSegmentConstraint(segmentContext, new SliderSegmentConstraint.Config(Vector3.forward, 0f, 1f)).DofCount, Is.EqualTo(1));
            Assert.That(new FixedAngularConstraint().DofCount, Is.EqualTo(0));
            Assert.That(new HingeAngularConstraint(new HingeAngularConstraint.Config(Vector3.forward, Quaternion.identity, -90f, 90f)).DofCount, Is.EqualTo(1));
            Assert.That(new BallSocketAngularConstraint(new BallSocketAngularConstraint.Config(Quaternion.identity, 1f, 1f, 180f)).DofCount, Is.EqualTo(3));
            Assert.That(new FreeAngularConstraint().DofCount, Is.EqualTo(3));
        }

        [Test]
        public void SliderSegmentConstraint_ConfigUpdateClampsCurrentSlideAndRecomputesReach()
        {
            var segment = new SliderSegmentConstraint(
                new ISegmentConstraint.SetupData(Vector3.up, 1f, Quaternion.identity, Quaternion.identity, true),
                new SliderSegmentConstraint.Config(Vector3.up, 0f, 2f));

            segment.SetCurrentSlide(1.5f);
            segment.ApplyConfig(new SliderSegmentConstraint.Config(Vector3.right, 0f, 1f));

            Assert.That(segment.CurrentSlide, Is.EqualTo(1f).Within(Epsilon));
            Assert.That(segment.MaxReach, Is.EqualTo(2f).Within(Epsilon));
            AssertVector3(Vector3.up, segment.SlideAxisParentLocal);
        }

        [Test]
        public void AngularConstraints_ConfigUpdatesChangeClampingWithoutOwnerReference()
        {
            var hinge = new HingeAngularConstraint(new HingeAngularConstraint.Config(Vector3.forward, Quaternion.identity, -90f, 90f));
            hinge.ApplyConfig(new HingeAngularConstraint.Config(Vector3.right, Quaternion.AngleAxis(90f, Vector3.up), 0f, 0f));

            Quaternion hingeDeviation = Quaternion.AngleAxis(45f, Vector3.forward);
            Quaternion clampedHinge = hinge.ClampDeviation(hingeDeviation);

            Assert.That(Quaternion.Angle(Quaternion.identity, clampedHinge), Is.EqualTo(0f).Within(Epsilon));
            AssertVector3(Vector3.forward, hinge.HingeAxis);

            var ball = new BallSocketAngularConstraint(new BallSocketAngularConstraint.Config(Quaternion.identity, 1f, 1f, 180f));
            ball.ApplyConfig(new BallSocketAngularConstraint.Config(Quaternion.AngleAxis(90f, Vector3.up), 0f, 0f, 0f));

            Quaternion ballDeviation = Quaternion.AngleAxis(45f, Vector3.right);
            Quaternion clampedBall = ball.ClampDeviation(ballDeviation);

            Assert.That(Quaternion.Angle(Quaternion.identity, clampedBall), Is.EqualTo(0f).Within(Epsilon));
            Assert.That(Quaternion.Angle(Quaternion.identity, ball.ConstraintAxisRotation), Is.EqualTo(0f).Within(Epsilon));
        }

        [Test]
        public void SolverJoint_RebindsWhenJointIsEnabledChanges()
        {
            using var chain = CreateSliderChain(jointIsEnabled: true);

            InvokePrivate(chain.FabrikSolver, "Awake");
            SolverJoint[] joints = GetSolverJoints(chain.FabrikSolver);

            Assert.That(joints[1].Segment, Is.TypeOf<SliderSegmentConstraint>());
            Assert.That(joints[1].Angular, Is.TypeOf<FixedAngularConstraint>());

            chain.SliderJoint.jointIsEnabled = false;
            Assert.That(chain.FabrikSolver.Chain.RefreshConstraintBindings(), Is.True);

            Assert.That(joints[1].Segment, Is.TypeOf<RigidSegmentConstraint>());
            Assert.That(joints[1].Angular, Is.TypeOf<FreeAngularConstraint>());

            chain.SliderJoint.jointIsEnabled = true;
            chain.SliderJoint.UpdateConstraints();
            Assert.That(chain.FabrikSolver.Chain.RefreshConstraintBindings(), Is.True);

            Assert.That(joints[1].Segment, Is.TypeOf<SliderSegmentConstraint>());
            Assert.That(joints[1].Angular, Is.TypeOf<FixedAngularConstraint>());
        }

        [Test]
        public void JacobianSolver_RangeConfigChangeDoesNotRebuildDofs()
        {
            using var chain = CreateSliderChain(jointIsEnabled: true);

            InvokePrivate(chain.JacobianSolver, "Awake");
            object dofsBefore = GetPrivateField(chain.JacobianSolver, "_dofs");

            chain.SliderJoint.maxLength = 3f;
            Assert.That(chain.JacobianSolver.Solve(), Is.True);

            object dofsAfter = GetPrivateField(chain.JacobianSolver, "_dofs");

            Assert.That(dofsAfter, Is.SameAs(dofsBefore));
            Assert.That(GetSegment<SliderSegmentConstraint>(chain.JacobianSolver, 1).MaxReach, Is.EqualTo(4f).Within(Epsilon));
        }

        [Test]
        public void Solvers_DisableWhenTargetIsMissing()
        {
            using var chain = CreateUnconstrainedChain(assignTarget: false);

            LogAssert.Expect(LogType.Error, "[OpenIK] FABRIK Solver disabled: target is not assigned.");
            InvokePrivate(chain.FabrikSolver, "Awake");
            LogAssert.Expect(LogType.Error, "[OpenIK] Jacobian Solver disabled: target is not assigned.");
            InvokePrivate(chain.JacobianSolver, "Awake");

            Assert.That(chain.FabrikSolver.enabled, Is.False);
            Assert.That(chain.JacobianSolver.enabled, Is.False);
        }

        [Test]
        public void Solvers_DisableWhenChainHasFewerThanTwoJoints()
        {
            using var chain = CreateUnconstrainedChain();
            var singleJointChain = new List<Transform> { chain.ChainJoints[0] };
            SetPrivateField(chain.FabrikSolver, "chainJoints", singleJointChain);
            SetPrivateField(chain.JacobianSolver, "chainJoints", singleJointChain);

            LogAssert.Expect(LogType.Error, "[OpenIK] FABRIK Solver disabled: at least two chain joints are required.");
            InvokePrivate(chain.FabrikSolver, "Awake");
            LogAssert.Expect(LogType.Error, "[OpenIK] Jacobian Solver disabled: at least two chain joints are required.");
            InvokePrivate(chain.JacobianSolver, "Awake");

            Assert.That(chain.FabrikSolver.enabled, Is.False);
            Assert.That(chain.JacobianSolver.enabled, Is.False);
        }

        [Test]
        public void Solvers_DisableWhenChainContainsNullJoint()
        {
            using var chain = CreateUnconstrainedChain();
            var chainWithNull = new List<Transform> { chain.ChainJoints[0], null, chain.ChainJoints[2] };
            SetPrivateField(chain.FabrikSolver, "chainJoints", chainWithNull);
            SetPrivateField(chain.JacobianSolver, "chainJoints", chainWithNull);

            LogAssert.Expect(LogType.Error, "[OpenIK] FABRIK Solver disabled: chainJoints[1] is not assigned.");
            InvokePrivate(chain.FabrikSolver, "Awake");
            LogAssert.Expect(LogType.Error, "[OpenIK] Jacobian Solver disabled: chainJoints[1] is not assigned.");
            InvokePrivate(chain.JacobianSolver, "Awake");

            Assert.That(chain.FabrikSolver.enabled, Is.False);
            Assert.That(chain.JacobianSolver.enabled, Is.False);
        }

        [Test]
        public void SolverChain_InitializeCreatesRigidBindingsAndComputesLength()
        {
            using var chain = CreateUnconstrainedChain();
            var solverChain = new SolverChain();

            solverChain.Initialize(chain.ChainJoints);

            Assert.That(solverChain.Count, Is.EqualTo(3));
            Assert.That(solverChain.ChainLength, Is.EqualTo(2f).Within(Epsilon));
            Assert.That(solverChain.Joints[1].Segment, Is.TypeOf<RigidSegmentConstraint>());
            Assert.That(solverChain.Joints[1].Angular, Is.TypeOf<FreeAngularConstraint>());
        }

        [Test]
        public void SolverChain_InitializeCreatesSliderBindingsAppliesConfigAndComputesLength()
        {
            using var chain = CreateSliderChain(jointIsEnabled: true);
            var solverChain = new SolverChain();

            solverChain.Initialize(chain.ChainJoints);

            Assert.That(solverChain.Count, Is.EqualTo(3));
            Assert.That(solverChain.ChainLength, Is.EqualTo(4f).Within(Epsilon));
            Assert.That(solverChain.Joints[1].Segment, Is.TypeOf<SliderSegmentConstraint>());
            Assert.That(solverChain.Joints[1].Angular, Is.TypeOf<FixedAngularConstraint>());
            Assert.That(((SliderSegmentConstraint)solverChain.Joints[1].Segment).MaxReach, Is.EqualTo(3f).Within(Epsilon));
        }

        [Test]
        public void RestPoseOffset_KeepsRestPositionAnchoredToIKParent()
        {
            using var chain = CreateSliderChain(jointIsEnabled: true);
            Transform parent = chain.ChainJoints[0];
            Transform child = chain.ChainJoints[1];
            // Deliberately skip the Unity parent in the IK chain, with a scaled hierarchy.
            child.SetParent(parent.parent, true);
            parent.parent.localScale = Vector3.one * 2f;
            parent.rotation = Quaternion.Euler(15f, 35f, 20f);
            child.rotation = Quaternion.Euler(40f, 10f, 60f);
            Vector3 restPosition = child.position;

            var solverChain = new SolverChain();
            solverChain.Initialize(chain.ChainJoints);
            Assert.That(chain.SliderJoint.IKParentTransform, Is.SameAs(parent));

            // Sliding the joint must not move its rest position.
            child.position += child.rotation * Vector3.up * 0.75f;
            AssertVector3(restPosition, PlayModeBasePosition(chain.SliderJoint));

            // Moving the IK parent carries the rest position with it.
            Quaternion delta = Quaternion.Euler(20f, -50f, 30f);
            Vector3 previousParentPosition = parent.position;
            parent.position += new Vector3(3f, -2f, 5f);
            parent.rotation = delta * parent.rotation;
            AssertVector3(parent.position + delta * (restPosition - previousParentPosition), PlayModeBasePosition(chain.SliderJoint));
        }

        /// What GetConstraintBasePosition returns in Play mode. EditMode tests always take its edit-time branch.
        private static Vector3 PlayModeBasePosition(ConstrainedJoint joint)
        {
            return joint.IKParentTransform.position + joint.IKParentTransform.rotation * joint.RestPoseOffset;
        }

        private static SliderChainFixture CreateSliderChain(bool jointIsEnabled)
        {
            var solverRoot = new GameObject("SolverRoot");
            var target = new GameObject("Target");
            var root = new GameObject("Root");
            var slider = new GameObject("Slider");
            var end = new GameObject("End");

            root.transform.SetParent(solverRoot.transform);
            slider.transform.SetParent(root.transform);
            end.transform.SetParent(slider.transform);

            root.transform.position = Vector3.zero;
            slider.transform.position = Vector3.up;
            end.transform.position = Vector3.up * 2f;

            var sliderJoint = slider.AddComponent<SliderIKJoint>();
            sliderJoint.slideAxis = Vector3.up;
            sliderJoint.minLength = 0f;
            sliderJoint.maxLength = 2f;
            sliderJoint.jointIsEnabled = jointIsEnabled;

            var fabrikSolver = solverRoot.AddComponent<FABRIKSolver>();
            var jacobianSolver = solverRoot.AddComponent<JacobianIKSolver>();
            var chain = new List<Transform> { root.transform, slider.transform, end.transform };

            SetPrivateField(fabrikSolver, "target", target.transform);
            SetPrivateField(fabrikSolver, "chainJoints", chain);

            SetPrivateField(jacobianSolver, "target", target.transform);
            SetPrivateField(jacobianSolver, "chainJoints", chain);

            return new SliderChainFixture(solverRoot, target, chain, fabrikSolver, jacobianSolver, sliderJoint);
        }

        private static SliderChainFixture CreateUnconstrainedChain(bool assignTarget = true)
        {
            var solverRoot = new GameObject("SolverRoot");
            var target = new GameObject("Target");
            var root = new GameObject("Root");
            var middle = new GameObject("Middle");
            var end = new GameObject("End");

            root.transform.SetParent(solverRoot.transform);
            middle.transform.SetParent(root.transform);
            end.transform.SetParent(middle.transform);

            root.transform.position = Vector3.zero;
            middle.transform.position = Vector3.up;
            end.transform.position = Vector3.up * 2f;

            var fabrikSolver = solverRoot.AddComponent<FABRIKSolver>();
            var jacobianSolver = solverRoot.AddComponent<JacobianIKSolver>();
            var chain = new List<Transform> { root.transform, middle.transform, end.transform };

            if (assignTarget)
                SetPrivateField(fabrikSolver, "target", target.transform);
            SetPrivateField(fabrikSolver, "chainJoints", chain);

            if (assignTarget)
                SetPrivateField(jacobianSolver, "target", target.transform);
            SetPrivateField(jacobianSolver, "chainJoints", chain);

            return new SliderChainFixture(solverRoot, target, chain, fabrikSolver, jacobianSolver, null);
        }

        private static SolverJoint[] GetSolverJoints(OpenIKSolverBase solver) => solver.Chain.Joints;

        private static float GetChainLength(OpenIKSolverBase solver) => solver.Chain.ChainLength;

        private static T GetSegment<T>(OpenIKSolverBase solver, int jointIndex) where T : class, ISegmentConstraint
        {
            SolverJoint[] joints = GetSolverJoints(solver);
            Assert.That(joints[jointIndex].Segment, Is.TypeOf<T>());
            return joints[jointIndex].Segment as T;
        }

        private static object GetPrivateField(object instance, string fieldName)
        {
            FieldInfo field = instance.GetType().GetField(fieldName, BindingFlags.Instance | BindingFlags.NonPublic);
            Assert.That(field, Is.Not.Null, $"Expected private field '{fieldName}' on {instance.GetType().Name}.");
            return field.GetValue(instance);
        }

        // Private members can be declared on a base class, so search the whole hierarchy.
        private static void SetPrivateField(object instance, string fieldName, object value)
        {
            FieldInfo field = null;
            for (System.Type type = instance.GetType(); type != null && field == null; type = type.BaseType)
                field = type.GetField(fieldName, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.DeclaredOnly);
            Assert.That(field, Is.Not.Null, $"Expected private field '{fieldName}' on {instance.GetType().Name}.");
            field.SetValue(instance, value);
        }

        private static void InvokePrivate(object instance, string methodName)
        {
            InvokePrivateResult(instance, methodName);
        }

        private static object InvokePrivateResult(object instance, string methodName)
        {
            MethodInfo method = null;
            for (System.Type type = instance.GetType(); type != null && method == null; type = type.BaseType)
                method = type.GetMethod(methodName, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.DeclaredOnly);
            Assert.That(method, Is.Not.Null, $"Expected private method '{methodName}' on {instance.GetType().Name}.");
            return method.Invoke(instance, null);
        }

        private static void AssertVector3(Vector3 expected, Vector3 actual)
        {
            Assert.That(actual.x, Is.EqualTo(expected.x).Within(Epsilon));
            Assert.That(actual.y, Is.EqualTo(expected.y).Within(Epsilon));
            Assert.That(actual.z, Is.EqualTo(expected.z).Within(Epsilon));
        }

        private readonly struct SliderChainFixture : System.IDisposable
        {
            private readonly GameObject _solverRoot;
            private readonly GameObject _target;

            public SliderChainFixture(
                GameObject solverRoot,
                GameObject target,
                IReadOnlyList<Transform> chainJoints,
                FABRIKSolver fabrikSolver,
                JacobianIKSolver jacobianSolver,
                SliderIKJoint sliderJoint)
            {
                _solverRoot = solverRoot;
                _target = target;
                ChainJoints = chainJoints;
                FabrikSolver = fabrikSolver;
                JacobianSolver = jacobianSolver;
                SliderJoint = sliderJoint;
            }

            public IReadOnlyList<Transform> ChainJoints { get; }
            public FABRIKSolver FabrikSolver { get; }
            public JacobianIKSolver JacobianSolver { get; }
            public SliderIKJoint SliderJoint { get; }

            public void Dispose()
            {
                Object.DestroyImmediate(_target);
                Object.DestroyImmediate(_solverRoot);
            }
        }
    }
}
