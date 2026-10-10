using UnityEngine;

namespace OpenIK
{
    // -- Interfaces --
    /// <summary>Limits how a joint rotates away from its rest pose. A ConstrainedJoint creates one for the solver.</summary>
    public interface IAngularConstraint : IJacobianDofProvider
    {
        /// <summary>Rest pose data for creating or rebinding an angular constraint.</summary>
        public readonly struct SetupData
        {
            public readonly Quaternion RestLocalRotation;

            public SetupData(Quaternion restLocalRotation)
            {
                RestLocalRotation = restLocalRotation;
            }
        }

        bool NeedsCurrentDeviation { get; }
        Quaternion ConstraintAxisRotation { get; }

        Quaternion ClampDeviation(Quaternion deviation, Quaternion? twistSource = null);
        Quaternion ProjectDeviation(Quaternion deviation);
    }

    /// <summary>
    /// Limits a joint's offset from its IK parent, for example a fixed bone length or a slider.
    /// A ConstrainedJoint creates one for the solver.
    /// </summary>
    public interface ISegmentConstraint : IJacobianDofProvider
    {
        /// <summary>Rest pose data for creating or rebinding a segment constraint.</summary>
        public readonly struct SetupData
        {
            public readonly Vector3 RestLocalOffset;
            public readonly float RestLocalOffsetLength;
            public readonly Quaternion ParentRotation;
            public readonly Quaternion ChildRotation;
            public readonly bool HasParent;

            public SetupData(
                Vector3 restLocalOffset,
                float restLocalOffsetLength,
                Quaternion parentRotation,
                Quaternion childRotation,
                bool hasParent)
            {
                RestLocalOffset = restLocalOffset;
                RestLocalOffsetLength = restLocalOffsetLength;
                ParentRotation = parentRotation;
                ChildRotation = childRotation;
                HasParent = hasParent;
            }
        }

        public readonly struct SolveContext
        {
            public readonly Vector3 ParentPosition;
            public readonly Quaternion ParentRotation;
            public readonly Vector3 ChildPosition;

            public SolveContext(Vector3 parentPosition, Quaternion parentRotation,
                Vector3 childPosition)
            {
                ParentPosition = parentPosition;
                ParentRotation = parentRotation;
                ChildPosition = childPosition;
            }
        }

        float MaxReach { get; }
        bool RequiresExplicitTransformPosition { get; }

        void ResetToRest();
        Vector3 GetRestLocalOffset();
        Vector3 GetCurrentLocalOffset();
        Vector3 ConstrainParentPosition(in SolveContext context);
        Vector3 ConstrainChildPosition(in SolveContext context);
        void SyncFromLocalOffset(Vector3 localOffset);
    }


    // -- Implementations --
    /// <summary>Keeps a joint at its rest offset from its IK parent. Used for joints without a slider.</summary>
    public sealed class RigidSegmentConstraint : ISegmentConstraint
    {
        private Vector3 _restLocalOffset;
        private float _restLocalOffsetLength;

        public RigidSegmentConstraint() { }

        public RigidSegmentConstraint(in ISegmentConstraint.SetupData setupData)
        {
            _restLocalOffset = setupData.RestLocalOffset;
            _restLocalOffsetLength = setupData.RestLocalOffsetLength;
        }

        public float MaxReach => _restLocalOffsetLength;
        public bool RequiresExplicitTransformPosition => false;
        public int DofCount => 0;

        public void ResetToRest() { }

        public Vector3 GetRestLocalOffset() => _restLocalOffset;

        public Vector3 GetCurrentLocalOffset() => _restLocalOffset;

        public Vector3 ConstrainParentPosition(in ISegmentConstraint.SolveContext context)
        {
            float currentDist = Vector3.Distance(context.ParentPosition, context.ChildPosition);
            if (currentDist > 0f)
            {
                float lambda = _restLocalOffsetLength / currentDist;
                return (1f - lambda) * context.ChildPosition + lambda * context.ParentPosition;
            }

            return context.ParentPosition;
        }

        public Vector3 ConstrainChildPosition(in ISegmentConstraint.SolveContext context)
        {
            return context.ParentPosition + context.ParentRotation * _restLocalOffset;
        }

        public void SyncFromLocalOffset(Vector3 localOffset) { }

        public SolverDofType GetDofType(int dofIndex) => SolverDofType.Translation;

        public Vector3 GetDofAxisWorld(int dofIndex, in IJacobianDofProvider.Context context) => Vector3.zero;

        public void ApplyDofDelta(int dofIndex, float delta, in IJacobianDofProvider.Context context) { }
    }

    /// <summary>Lets a joint slide along an axis within its travel limits. SliderIKJoint creates it.</summary>
    public sealed class SliderSegmentConstraint : ISegmentConstraint, ISegmentMotionProvider
    {
        // How far, in metres, a starting offset may lie off the slide axis or outside the travel range
        // before it counts as outside the limits.
        private const float OutsideLimitsTolerance = 1e-3f;

        /// <summary>Settings that a SliderIKJoint copies into its segment constraint.</summary>
        public readonly struct Config
        {
            public readonly Vector3 SlideAxisNormalized;
            public readonly float MinLength;
            public readonly float MaxLength;

            public Config(Vector3 slideAxisNormalized, float minLength, float maxLength)
            {
                SlideAxisNormalized = slideAxisNormalized.sqrMagnitude > 1e-8f
                    ? slideAxisNormalized.normalized
                    : Vector3.forward;
                MinLength = Mathf.Min(minLength, maxLength);
                MaxLength = Mathf.Max(minLength, maxLength);
            }
        }

        private Config _config;
        private Vector3 _fixedOffsetLocal;
        private Vector3 _slideAxisParentLocal = Vector3.forward;
        private float _restSlideOffset;
        private float _restSlide;
        private float _currentSlide;
        private float _maxReach;

        public SliderSegmentConstraint(in ISegmentConstraint.SetupData setupData, Config config)
        {
            _config = config;
            InitializeFromSetupData(setupData);
        }

        public float MaxReach => _maxReach;
        public bool RequiresExplicitTransformPosition => true;
        public int DofCount => 1;

        public Vector3 FixedOffsetLocal => _fixedOffsetLocal;
        public Vector3 SlideAxisParentLocal => _slideAxisParentLocal;
        public float RestSlideOffset => _restSlideOffset;
        public float RestSlide => _restSlide;
        public float CurrentSlide => _currentSlide;

        /// <summary>Updates the travel limits. The slide axis and rest offset keep their values from setup.</summary>
        public void ApplyConfig(Config config)
        {
            _config = new Config(_config.SlideAxisNormalized, config.MinLength, config.MaxLength);
            _restSlide = ClampRelativeSlide(0f);
            _currentSlide = ClampRelativeSlide(_currentSlide);
            _maxReach = ComputeMaxReach();
        }

        public void ResetToRest()
        {
            _currentSlide = _restSlide;
        }

        public void SetCurrentSlide(float relativeSlide)
        {
            _currentSlide = ClampRelativeSlide(relativeSlide);
        }

        public Vector3 GetRestLocalOffset() => GetLocalOffset(_restSlide);

        public Vector3 GetCurrentLocalOffset() => GetLocalOffset(_currentSlide);

        public Vector3 ConstrainParentPosition(in ISegmentConstraint.SolveContext context)
        {
            Vector3 axisWorld = GetAxisWorld(context.ParentRotation);
            Vector3 anchorOffsetWorld = context.ParentRotation * _fixedOffsetLocal;
            float absoluteSlide = Vector3.Dot(
                context.ChildPosition - anchorOffsetWorld - context.ParentPosition,
                axisWorld);

            _currentSlide = ClampRelativeSlide(absoluteSlide - _restSlideOffset);
            return context.ChildPosition
                - anchorOffsetWorld
                - axisWorld * GetAbsoluteSlide(_currentSlide);
        }

        public Vector3 ConstrainChildPosition(in ISegmentConstraint.SolveContext context)
        {
            Vector3 axisWorld = GetAxisWorld(context.ParentRotation);
            Vector3 anchorWorld = context.ParentPosition + context.ParentRotation * _fixedOffsetLocal;
            float absoluteSlide = Vector3.Dot(context.ChildPosition - anchorWorld, axisWorld);
            _currentSlide = ClampRelativeSlide(absoluteSlide - _restSlideOffset);
            return anchorWorld + axisWorld * GetAbsoluteSlide(_currentSlide);
        }

        public void SyncFromLocalOffset(Vector3 localOffset)
        {
            float absoluteSlide = Vector3.Dot(localOffset, _slideAxisParentLocal);
            _currentSlide = ClampRelativeSlide(absoluteSlide - _restSlideOffset);
        }

        public float GetMotionDistance(Vector3 currentLocalOffset, Vector3 desiredLocalOffset)
        {
            return Mathf.Abs(Vector3.Dot(desiredLocalOffset - currentLocalOffset, _slideAxisParentLocal));
        }

        // Only travel along the slide axis uses the budget. A sideways offset at the start is removed
        // at once. Does not change the segment's solver state.
        public JointMotionStep StepLocalOffset(
            Vector3 currentLocalOffset,
            Vector3 desiredLocalOffset,
            float maxDistance,
            out Vector3 appliedLocalOffset)
        {
            float currentSlide = Vector3.Dot(currentLocalOffset, _slideAxisParentLocal) - _restSlideOffset;
            float desiredSlide = Vector3.Dot(desiredLocalOffset, _slideAxisParentLocal) - _restSlideOffset;

            bool startedOutsideLimits =
                currentSlide < _config.MinLength - OutsideLimitsTolerance ||
                currentSlide > _config.MaxLength + OutsideLimitsTolerance ||
                (currentLocalOffset - GetLocalOffset(currentSlide)).sqrMagnitude
                    > OutsideLimitsTolerance * OutsideLimitsTolerance;

            float delta = desiredSlide - currentSlide;
            if (Mathf.Abs(delta) <= maxDistance)
            {
                appliedLocalOffset = desiredLocalOffset;
                return new JointMotionStep(JointMotionStatus.Reached, startedOutsideLimits);
            }

            appliedLocalOffset = GetLocalOffset(currentSlide + Mathf.Sign(delta) * maxDistance);
            return new JointMotionStep(JointMotionStatus.Limited, startedOutsideLimits);
        }

        public SolverDofType GetDofType(int dofIndex) => SolverDofType.Translation;

        public Vector3 GetDofAxisWorld(int dofIndex, in IJacobianDofProvider.Context context)
        {
            if (context.Parent != null)
                return GetAxisWorld(context.Parent.SolverRotation);

            return context.Joint.SolverRotation * _config.SlideAxisNormalized;
        }

        public void ApplyDofDelta(int dofIndex, float delta, in IJacobianDofProvider.Context context)
        {
            if (context.Parent == null)
                return;

            _currentSlide = ClampRelativeSlide(_currentSlide + delta);
            Vector3 localOffset = GetCurrentLocalOffset();
            context.Joint.SolverPosition = context.Parent.SolverPosition
                + context.Parent.SolverRotation * localOffset;
        }

        private void InitializeFromSetupData(in ISegmentConstraint.SetupData setupData)
        {
            Quaternion childLocalRotation = Quaternion.Inverse(setupData.ParentRotation) * setupData.ChildRotation;
            Vector3 slideAxisParentLocal = (childLocalRotation * _config.SlideAxisNormalized).normalized;
            if (slideAxisParentLocal.sqrMagnitude < 1e-8f)
                slideAxisParentLocal = Vector3.forward;

            float capturedSlide = Vector3.Dot(setupData.RestLocalOffset, slideAxisParentLocal);
            _slideAxisParentLocal = slideAxisParentLocal;
            _fixedOffsetLocal = setupData.RestLocalOffset - slideAxisParentLocal * capturedSlide;
            _restSlideOffset = capturedSlide;
            _restSlide = ClampRelativeSlide(0f);
            _currentSlide = _restSlide;
            _maxReach = ComputeMaxReach();
        }

        private float ClampRelativeSlide(float relativeSlide)
        {
            return Mathf.Clamp(relativeSlide, _config.MinLength, _config.MaxLength);
        }

        private float GetAbsoluteSlide(float relativeSlide)
        {
            return _restSlideOffset + relativeSlide;
        }

        private Vector3 GetLocalOffset(float relativeSlide)
        {
            return _fixedOffsetLocal + _slideAxisParentLocal * GetAbsoluteSlide(relativeSlide);
        }

        private Vector3 GetAxisWorld(Quaternion parentRotation)
        {
            return parentRotation * _slideAxisParentLocal;
        }

        private float ComputeMaxReach()
        {
            Vector3 minOffset = GetLocalOffset(_config.MinLength);
            Vector3 maxOffset = GetLocalOffset(_config.MaxLength);
            return Mathf.Max(minOffset.magnitude, maxOffset.magnitude);
        }
    }

    /// <summary>Base class for the built-in angular constraints.</summary>
    public abstract class AngularConstraintBase : IAngularConstraint
    {
        // How far, in degrees, a starting rotation may lie outside the limits before it counts as outside them.
        protected const float OutsideLimitsToleranceDegrees = 0.5f;

        public virtual bool NeedsCurrentDeviation => false;
        public Quaternion ConstraintAxisRotation { get; protected set; } = Quaternion.identity;
        public abstract int DofCount { get; }

        public abstract Quaternion ClampDeviation(Quaternion deviation, Quaternion? twistSource = null);

        public virtual Quaternion ProjectDeviation(Quaternion deviation) => deviation;

        public abstract SolverDofType GetDofType(int dofIndex);
        public abstract Vector3 GetDofAxisWorld(int dofIndex, in IJacobianDofProvider.Context context);
        public abstract void ApplyDofDelta(int dofIndex, float delta, in IJacobianDofProvider.Context context);

        protected static void ApplyRotationDelta(SolverJoint joint, Vector3 axis, float radians)
        {
            joint.SolverRotation = Quaternion.AngleAxis(radians * Mathf.Rad2Deg, axis)
                * joint.SolverRotation;
        }
    }

    /// <summary>Allows any rotation. Used for joints without an angular limit.</summary>
    public sealed class FreeAngularConstraint : AngularConstraintBase
    {
        public override int DofCount => 3;

        public override Quaternion ClampDeviation(Quaternion deviation, Quaternion? twistSource = null)
        {
            return deviation;
        }

        public override SolverDofType GetDofType(int dofIndex) => SolverDofType.Rotation;

        public override Vector3 GetDofAxisWorld(int dofIndex, in IJacobianDofProvider.Context context)
        {
            Quaternion rotation = context.Joint.SolverRotation;
            return dofIndex switch
            {
                0 => rotation * Vector3.right,
                1 => rotation * Vector3.up,
                _ => rotation * Vector3.forward
            };
        }

        public override void ApplyDofDelta(int dofIndex, float delta, in IJacobianDofProvider.Context context)
        {
            ApplyRotationDelta(context.Joint, GetDofAxisWorld(dofIndex, context), delta);
        }
    }

    /// <summary>Keeps a joint at its rest rotation relative to its IK parent. Used by slider joints.</summary>
    public sealed class FixedAngularConstraint : AngularConstraintBase
    {
        public override int DofCount => 0;

        public override Quaternion ClampDeviation(Quaternion deviation, Quaternion? twistSource = null)
        {
            return Quaternion.identity;
        }

        public override Quaternion ProjectDeviation(Quaternion deviation)
        {
            return Quaternion.identity;
        }

        public override SolverDofType GetDofType(int dofIndex) => SolverDofType.Rotation;

        public override Vector3 GetDofAxisWorld(int dofIndex, in IJacobianDofProvider.Context context) => Vector3.zero;

        public override void ApplyDofDelta(int dofIndex, float delta, in IJacobianDofProvider.Context context) { }
    }

    /// <summary>Limits rotation to one axis within an angle range. HingeIKJoint creates it.</summary>
    public sealed class HingeAngularConstraint : AngularConstraintBase, IAngularMotionProvider
    {
        /// <summary>Settings that a HingeIKJoint copies into its angular constraint.</summary>
        public readonly struct Config
        {
            public readonly Vector3 HingeAxis;
            public readonly Quaternion ConstraintAxisRotation;
            public readonly float MinAngle;
            public readonly float MaxAngle;

            public Config(
                Vector3 hingeAxis,
                Quaternion constraintAxisRotation,
                float minAngle,
                float maxAngle)
            {
                HingeAxis = hingeAxis.sqrMagnitude > 1e-8f ? hingeAxis.normalized : Vector3.forward;
                ConstraintAxisRotation = constraintAxisRotation;
                MinAngle = minAngle;
                MaxAngle = maxAngle;
            }
        }

        private Config _config;
        private Vector3 _hingeAxisInLocalConstraintFrame = Vector3.forward;

        public HingeAngularConstraint(Config config)
        {
            _config = config;
            ApplyFixedConfig();
        }

        public override bool NeedsCurrentDeviation => false;
        public override int DofCount => 1;
        public Vector3 HingeAxis => _config.HingeAxis;

        /// <summary>Updates the angle limits. The hinge axis and constraint frame keep their values from setup.</summary>
        public void ApplyConfig(Config config)
        {
            _config = new Config(
                _config.HingeAxis,
                _config.ConstraintAxisRotation,
                config.MinAngle,
                config.MaxAngle);
            ApplyFixedConfig();
        }

        public override Quaternion ClampDeviation(Quaternion deviation, Quaternion? twistSource = null)
        {
            return HingeIKJoint.ClampHinge(deviation, _hingeAxisInLocalConstraintFrame, _config.MinAngle, _config.MaxAngle);
        }

        public override Quaternion ProjectDeviation(Quaternion deviation)
        {
            return HingeIKJoint.ProjectOntoHingePlane(deviation, _hingeAxisInLocalConstraintFrame);
        }

        public float GetMotionDistance(Quaternion currentDeviation, Quaternion desiredDeviation)
        {
            return Mathf.Abs(MeasureHingeTravel(currentDeviation, desiredDeviation, out _));
        }

        // Only the hinge angle uses the budget. Rotation around other axes at the start is removed at once.
        public JointMotionStep StepDeviation(
            Quaternion currentDeviation,
            Quaternion desiredDeviation,
            float maxDegrees,
            out Quaternion appliedDeviation)
        {
            Vector3 axis = _hingeAxisInLocalConstraintFrame;
            float delta = MeasureHingeTravel(currentDeviation, desiredDeviation, out float currentAngle);
            bool fullTurn = IsFullTurn;

            bool startedOutsideLimits =
                (!fullTurn && (currentAngle < _config.MinAngle - OutsideLimitsToleranceDegrees ||
                               currentAngle > _config.MaxAngle + OutsideLimitsToleranceDegrees)) ||
                Quaternion.Angle(currentDeviation, Quaternion.AngleAxis(currentAngle, axis)) > OutsideLimitsToleranceDegrees;

            if (Mathf.Abs(delta) <= maxDegrees)
            {
                appliedDeviation = desiredDeviation;
                return new JointMotionStep(JointMotionStatus.Reached, startedOutsideLimits);
            }

            appliedDeviation = Quaternion.AngleAxis(currentAngle + Mathf.Sign(delta) * maxDegrees, axis);
            return new JointMotionStep(JointMotionStatus.Limited, startedOutsideLimits);
        }

        private bool IsFullTurn => _config.MinAngle <= -180f && _config.MaxAngle >= 180f;

        // Signed hinge travel from the current to the desired angle, staying inside the range. A
        // restricted range contains zero and lies within -180 to 180, so moving straight between two
        // angles inside it never leaves it. Only a full-turn hinge wraps around, the shorter way
        // (+180 on an exact tie).
        private float MeasureHingeTravel(Quaternion currentDeviation, Quaternion desiredDeviation, out float currentAngle)
        {
            Vector3 axis = _hingeAxisInLocalConstraintFrame;
            currentAngle = HingeIKJoint.ExtractHingeAngle(currentDeviation, axis);
            float desiredAngle = HingeIKJoint.ExtractHingeAngle(desiredDeviation, axis);
            if (IsFullTurn)
                return Mathf.DeltaAngle(currentAngle, desiredAngle);

            currentAngle = UnwrapIntoRange(currentAngle);
            return UnwrapIntoRange(desiredAngle) - currentAngle;
        }

        // Replaces an angle near ±180 with the equal angle, 360 degrees apart, that lies inside the range.
        private float UnwrapIntoRange(float angle)
        {
            if (angle > _config.MaxAngle && angle - 360f >= _config.MinAngle)
                return angle - 360f;
            if (angle < _config.MinAngle && angle + 360f <= _config.MaxAngle)
                return angle + 360f;
            return angle;
        }

        public override SolverDofType GetDofType(int dofIndex) => SolverDofType.Rotation;

        public override Vector3 GetDofAxisWorld(int dofIndex, in IJacobianDofProvider.Context context)
        {
            return context.Joint.SolverRotation * _config.HingeAxis;
        }

        public override void ApplyDofDelta(int dofIndex, float delta, in IJacobianDofProvider.Context context)
        {
            ApplyRotationDelta(context.Joint, GetDofAxisWorld(dofIndex, context), delta);
        }

        private void ApplyFixedConfig()
        {
            ConstraintAxisRotation = _config.ConstraintAxisRotation;
            _hingeAxisInLocalConstraintFrame = (Quaternion.Inverse(ConstraintAxisRotation) * _config.HingeAxis).normalized;
        }
    }

    /// <summary>Limits swing to a cone and twist to a range. BallSocketIKJoint creates it.</summary>
    public sealed class BallSocketAngularConstraint : AngularConstraintBase, IAngularMotionProvider
    {
        /// <summary>Settings that a BallSocketIKJoint copies into its angular constraint.</summary>
        public readonly struct Config
        {
            public readonly Quaternion ConstraintAxisRotation;
            public readonly float SwingPitchHalfSin;
            public readonly float SwingYawHalfSin;
            public readonly float TwistHalfAngle;

            public Config(
                Quaternion constraintAxisRotation,
                float swingPitchHalfSin,
                float swingYawHalfSin,
                float twistHalfAngle)
            {
                ConstraintAxisRotation = constraintAxisRotation;
                SwingPitchHalfSin = swingPitchHalfSin;
                SwingYawHalfSin = swingYawHalfSin;
                TwistHalfAngle = twistHalfAngle;
            }
        }

        private Config _config;

        public BallSocketAngularConstraint(Config config)
        {
            _config = config;
            ConstraintAxisRotation = config.ConstraintAxisRotation;
        }

        public override bool NeedsCurrentDeviation => true;
        public override int DofCount => 3;

        /// <summary>Updates the swing and twist limits. The constraint frame keeps its value from setup.</summary>
        public void ApplyConfig(Config config)
        {
            _config = new Config(
                _config.ConstraintAxisRotation,
                config.SwingPitchHalfSin,
                config.SwingYawHalfSin,
                config.TwistHalfAngle);
            ConstraintAxisRotation = _config.ConstraintAxisRotation;
        }

        public override Quaternion ClampDeviation(Quaternion deviation, Quaternion? twistSource = null)
        {
            Quaternion swing = BallSocketIKJoint.ExtractSwingAroundForwardVector(deviation);
            Quaternion twist = BallSocketIKJoint.ExtractTwistAroundForwardVector(twistSource ?? deviation);
            Quaternion clampedSwing = BallSocketIKJoint.ClampSwing(swing, _config.SwingPitchHalfSin, _config.SwingYawHalfSin);
            Quaternion clampedTwist = BallSocketIKJoint.ClampTwist(twist, _config.TwistHalfAngle);
            return clampedSwing * clampedTwist;
        }

        // Inside the limits this is the length of the allowed swing and twist path. It can be much longer
        // than the shortest rotation when the twist range forces the long way around.
        public float GetMotionDistance(Quaternion currentDeviation, Quaternion desiredDeviation)
        {
            if (!IsWithinLimits(currentDeviation))
                return Quaternion.Angle(currentDeviation, desiredDeviation);

            return MeasureAllowedPath(currentDeviation, desiredDeviation, out _, out _, out _, out _);
        }

        // How many times a step is halved when clamping to the cone pushes it past its budget.
        private const int ClampBudgetRetries = 8;

        // Swing and twist change together at steady rates, so the joint turns at one speed along the
        // whole path, and a budget of maxDegrees covers maxDegrees / pathLength of it. A swing that would
        // leave the cone partway is pulled back to its edge. A joint that starts outside its limits moves
        // straight toward the target instead, and the step reports that.
        public JointMotionStep StepDeviation(
            Quaternion currentDeviation,
            Quaternion desiredDeviation,
            float maxDegrees,
            out Quaternion appliedDeviation)
        {
            if (!IsWithinLimits(currentDeviation))
            {
                if (Quaternion.Angle(currentDeviation, desiredDeviation) <= maxDegrees)
                {
                    appliedDeviation = desiredDeviation;
                    return new JointMotionStep(JointMotionStatus.Reached, true);
                }

                appliedDeviation = Quaternion.RotateTowards(currentDeviation, desiredDeviation, Mathf.Max(0f, maxDegrees));
                return new JointMotionStep(JointMotionStatus.Limited, true);
            }

            float pathLength = MeasureAllowedPath(
                currentDeviation,
                desiredDeviation,
                out Quaternion currentSwing,
                out Quaternion desiredSwing,
                out float currentTwist,
                out float twistDelta);

            if (pathLength <= maxDegrees)
            {
                appliedDeviation = desiredDeviation;
                return new JointMotionStep(JointMotionStatus.Reached, false);
            }

            if (maxDegrees <= 0f)
            {
                appliedDeviation = currentDeviation;
                return new JointMotionStep(JointMotionStatus.Limited, false);
            }

            float t = maxDegrees / pathLength;
            appliedDeviation = EvaluateAllowedPath(currentSwing, desiredSwing, currentTwist, twistDelta, t);

            // The unclamped path never exceeds the budget; only the cone clamp can, so back off if it did.
            for (int i = 0; i < ClampBudgetRetries && Quaternion.Angle(currentDeviation, appliedDeviation) > maxDegrees + 1e-3f; i++)
            {
                t *= 0.5f;
                appliedDeviation = EvaluateAllowedPath(currentSwing, desiredSwing, currentTwist, twistDelta, t);
            }

            bool madeProgress = MeasureAllowedPath(appliedDeviation, desiredDeviation, out _, out _, out _, out _)
                < pathLength - 1e-4f;
            return new JointMotionStep(
                madeProgress ? JointMotionStatus.Limited : JointMotionStatus.Blocked,
                false);
        }

        // Length in degrees of the allowed path between two rotations: the swing slerps and the twist
        // moves along its allowed arc. Both change at steady rates, so the length is the swing rotation
        // (angle θs around axis a) plus the twist rotation (Δτ around the twist axis f), added as
        // vectors: |θs·a + Δτ·f|.
        private float MeasureAllowedPath(
            Quaternion currentDeviation,
            Quaternion desiredDeviation,
            out Quaternion currentSwing,
            out Quaternion desiredSwing,
            out float currentTwist,
            out float twistDelta)
        {
            currentSwing = BallSocketIKJoint.ExtractSwingAroundForwardVector(currentDeviation);
            desiredSwing = BallSocketIKJoint.ExtractSwingAroundForwardVector(desiredDeviation);
            currentTwist = GetTwistAngle(currentDeviation);
            float desiredTwist = GetTwistAngle(desiredDeviation);
            // A full-turn twist range takes the short way. A smaller range cannot pass through ±180°,
            // so the twist goes directly from one angle to the other.
            twistDelta = _config.TwistHalfAngle >= 180f
                ? Mathf.DeltaAngle(currentTwist, desiredTwist)
                : desiredTwist - currentTwist;

            // Slerp takes the shorter of the two quaternion arcs.
            Quaternion relative = Quaternion.Inverse(currentSwing) * desiredSwing;
            if (relative.w < 0f)
                relative = new Quaternion(-relative.x, -relative.y, -relative.z, -relative.w);

            Vector3 halfAxis = new Vector3(relative.x, relative.y, relative.z);
            float sinHalf = halfAxis.magnitude;
            Vector3 swingVelocity = Vector3.zero;
            if (sinHalf > 1e-7f)
            {
                float swingDegrees = 2f * Mathf.Atan2(sinHalf, relative.w) * Mathf.Rad2Deg;
                swingVelocity = halfAxis * (swingDegrees / sinHalf);
            }

            return (swingVelocity + Vector3.forward * twistDelta).magnitude;
        }

        private Quaternion EvaluateAllowedPath(
            Quaternion currentSwing,
            Quaternion desiredSwing,
            float currentTwist,
            float twistDelta,
            float t)
        {
            Quaternion swing = BallSocketIKJoint.ClampSwing(
                Quaternion.Slerp(currentSwing, desiredSwing, t),
                _config.SwingPitchHalfSin,
                _config.SwingYawHalfSin);
            return swing * Quaternion.AngleAxis(currentTwist + twistDelta * t, Vector3.forward);
        }

        private bool IsWithinLimits(Quaternion deviation)
        {
            return Quaternion.Angle(ClampDeviation(deviation), deviation) <= OutsideLimitsToleranceDegrees;
        }

        private static float GetTwistAngle(Quaternion deviation)
        {
            Quaternion twist = BallSocketIKJoint.ExtractTwistAroundForwardVector(deviation);
            return Mathf.DeltaAngle(0f, 2f * Mathf.Atan2(twist.z, twist.w) * Mathf.Rad2Deg);
        }

        public override SolverDofType GetDofType(int dofIndex) => SolverDofType.Rotation;

        public override Vector3 GetDofAxisWorld(int dofIndex, in IJacobianDofProvider.Context context)
        {
            Quaternion rotation = context.Joint.SolverRotation;
            return dofIndex switch
            {
                0 => rotation * Vector3.right,
                1 => rotation * Vector3.up,
                _ => rotation * Vector3.forward
            };
        }

        public override void ApplyDofDelta(int dofIndex, float delta, in IJacobianDofProvider.Context context)
        {
            ApplyRotationDelta(context.Joint, GetDofAxisWorld(dofIndex, context), delta);
        }
    }
}
