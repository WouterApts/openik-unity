using UnityEngine;

namespace OpenIK
{
    // -- Interfaces --
    public interface IAngularConstraint : IJacobianDofProvider
    {
        /// Rest-pose data used when a runtime angular constraint is created or rebound.
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

    public interface ISegmentConstraint : IJacobianDofProvider
    {
        /// Rest-pose data used when a runtime segment constraint is created or rebound.
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

    public sealed class SliderSegmentConstraint : ISegmentConstraint, ISegmentMotionProvider
    {
        /// Distance in metres a starting offset may leave the slide axis or travel range before it counts as outside the limits.
        private const float OutsideLimitsTolerance = 1e-3f;

        /// Runtime configuration copied from a <see cref="SliderIKJoint"/> into its segment constraint.
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

        /// Updates slider limits while keeping the bound slide axis and rest offset stable.
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

        /// <summary>
        /// Moves the slide travel from <paramref name="currentLocalOffset"/> toward
        /// <paramref name="desiredLocalOffset"/> by at most <paramref name="maxDistance"/> metres
        /// along the slide axis, keeping the fixed lateral offset.
        /// </summary>
        /// <remarks>
        /// Offsets are in the IK parent's rotation frame in world units, so travel is measured in
        /// metres under the uniform scale captured at initialization. Does not modify the segment's
        /// runtime solver state.
        /// </remarks>
        public float GetMotionDistance(Vector3 currentLocalOffset, Vector3 desiredLocalOffset)
        {
            return Mathf.Abs(Vector3.Dot(desiredLocalOffset - currentLocalOffset, _slideAxisParentLocal));
        }

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

    public abstract class AngularConstraintBase : IAngularConstraint
    {
        /// Angle in degrees a starting deviation may leave the constraint before it counts as outside the limits.
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

    public sealed class HingeAngularConstraint : AngularConstraintBase, IAngularMotionProvider
    {
        /// Runtime configuration copied from a <see cref="HingeIKJoint"/> into its angular constraint.
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

        /// Updates hinge angle limits while keeping the bound hinge axis and constraint frame stable.
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

        /// <summary>
        /// Moves the signed hinge angle from <paramref name="currentDeviation"/> toward
        /// <paramref name="desiredDeviation"/> by at most <paramref name="maxDegrees"/>.
        /// </summary>
        /// <remarks>
        /// A restricted range always contains zero and lies within [-180, 180], so the direct
        /// angular path between two in-range angles never crosses the forbidden arc. Only a
        /// full-turn hinge wraps, along the shortest path (+180 on an exact tie).
        /// </remarks>
        public float GetMotionDistance(Quaternion currentDeviation, Quaternion desiredDeviation)
        {
            return Mathf.Abs(MeasureHingeTravel(currentDeviation, desiredDeviation, out _));
        }

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

        /// Signed hinge travel from the current to the desired deviation along the legal path.
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

        /// Maps an angle at the +/-180 seam onto the side that lies inside a restricted range.
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

    public sealed class BallSocketAngularConstraint : AngularConstraintBase, IAngularMotionProvider
    {
        /// Runtime configuration copied from a <see cref="BallSocketIKJoint"/> into its angular constraint.
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

        /// Updates ball-socket limits while keeping the bound constraint frame stable.
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

        public float GetMotionDistance(Quaternion currentDeviation, Quaternion desiredDeviation)
        {
            return Quaternion.Angle(currentDeviation, desiredDeviation);
        }

        /// Bisection steps used to find the furthest legal point within the angular budget.
        private const int MotionSearchIterations = 14;

        /// <summary>
        /// Moves <paramref name="currentDeviation"/> toward <paramref name="desiredDeviation"/> by at
        /// most <paramref name="maxDegrees"/> of total relative rotation while staying inside the
        /// swing cone and twist range.
        /// </summary>
        /// <remarks>
        /// Swing and twist advance together along a path whose swing is projected back into the
        /// cone, so every point of the path is legal. A bounded bisection then picks the furthest
        /// point on that path whose rotation from the current deviation fits the budget. A start
        /// outside the limits has no legal path; it moves straight toward the target within the
        /// budget and is reported.
        /// </remarks>
        public JointMotionStep StepDeviation(
            Quaternion currentDeviation,
            Quaternion desiredDeviation,
            float maxDegrees,
            out Quaternion appliedDeviation)
        {
            bool startedOutsideLimits = !IsWithinLimits(currentDeviation);

            float remaining = Quaternion.Angle(currentDeviation, desiredDeviation);
            if (remaining <= maxDegrees)
            {
                appliedDeviation = desiredDeviation;
                return new JointMotionStep(JointMotionStatus.Reached, startedOutsideLimits);
            }

            if (maxDegrees <= 0f)
            {
                appliedDeviation = currentDeviation;
                return new JointMotionStep(JointMotionStatus.Limited, startedOutsideLimits);
            }

            if (startedOutsideLimits)
            {
                appliedDeviation = Quaternion.RotateTowards(currentDeviation, desiredDeviation, maxDegrees);
                return new JointMotionStep(JointMotionStatus.Limited, true);
            }

            Quaternion currentSwing = BallSocketIKJoint.ExtractSwingAroundForwardVector(currentDeviation);
            Quaternion desiredSwing = BallSocketIKJoint.ExtractSwingAroundForwardVector(desiredDeviation);
            float currentTwist = GetTwistAngle(currentDeviation);
            float desiredTwist = GetTwistAngle(desiredDeviation);
            // A twist range below a full turn is one contiguous arc around zero, so the direct path stays legal.
            float twistDelta = _config.TwistHalfAngle >= 180f
                ? Mathf.DeltaAngle(currentTwist, desiredTwist)
                : desiredTwist - currentTwist;

            float lo = 0f;
            float hi = 1f;
            Quaternion best = currentDeviation;
            for (int i = 0; i < MotionSearchIterations; i++)
            {
                float t = 0.5f * (lo + hi);
                Quaternion swing = BallSocketIKJoint.ClampSwing(
                    Quaternion.Slerp(currentSwing, desiredSwing, t),
                    _config.SwingPitchHalfSin,
                    _config.SwingYawHalfSin);
                Quaternion candidate = swing * Quaternion.AngleAxis(currentTwist + twistDelta * t, Vector3.forward);

                if (Quaternion.Angle(currentDeviation, candidate) <= maxDegrees)
                {
                    lo = t;
                    best = candidate;
                }
                else
                {
                    hi = t;
                }
            }

            appliedDeviation = best;
            bool madeProgress = Quaternion.Angle(best, desiredDeviation) < remaining - 1e-4f;
            return new JointMotionStep(
                madeProgress ? JointMotionStatus.Limited : JointMotionStatus.Blocked,
                false);
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
