using System;
using UnityEngine;

namespace OpenIK
{
    /// <summary>
    /// What happened when a solver's output was applied to the scene transforms.
    /// </summary>
    /// <remarks>
    /// <see cref="IKSolverOutput.Converged"/> describes the solve. This status describes the actual
    /// transforms, which can lag the solution while joint speed limits are active.
    /// </remarks>
    public readonly struct IKApplicationStatus
    {
        /// <summary>True when the latest solve was written to the transforms.</summary>
        public bool Applied { get; }

        /// <summary>True when at least one joint used its speed limit during application.</summary>
        public bool UsedSpeedLimits { get; }

        /// <summary>True when at least one joint's speed limit stopped it short of the solved pose.</summary>
        public bool AnyJointLimited { get; }

        /// <summary>True when at least one joint could not make valid progress despite its speed budget.</summary>
        public bool AnyJointBlocked { get; }

        /// <summary>
        /// True when at least one limited joint started outside its constraint, for example after an
        /// external teleport or a tightened range. Such joints still move toward the solved pose at their
        /// normal speed. A hinge's off-axis rotation and a slider's sideways offset are removed at once,
        /// because the joint cannot move in those directions. Call
        /// <see cref="OpenIKSolverBase.SnapToSolution"/> to fix everything immediately.
        /// </summary>
        public bool AnyJointStartedOutsideLimits { get; }

        /// <summary>
        /// True when the transforms now hold the solved pose. This is not arrival at the target:
        /// an unreachable target produces a settled arm with a nonzero <see cref="PositionError"/>.
        /// </summary>
        public bool ReachedSolution => Applied && !AnyJointLimited && !AnyJointBlocked;

        /// <summary>
        /// Distance in metres from the actual end effector to the target after application.
        /// NaN when nothing was applied.
        /// </summary>
        public float PositionError { get; }

        /// <summary>
        /// Angle in degrees between the actual end orientation and the target's, for solvers with an
        /// orientation objective. NaN otherwise.
        /// </summary>
        public float OrientationError { get; }

        /// <summary>True when <see cref="OrientationError"/> holds a value.</summary>
        public bool HasOrientationError => !float.IsNaN(OrientationError);

        /// <summary>Status for a solve that was not written to the transforms (for example in SolveOnly mode).</summary>
        public static IKApplicationStatus NotApplied => new(false, false, false, false, false, float.NaN, float.NaN);

        internal IKApplicationStatus(
            bool applied,
            bool usedSpeedLimits,
            bool anyJointLimited,
            bool anyJointBlocked,
            bool anyJointStartedOutsideLimits,
            float positionError,
            float orientationError)
        {
            Applied = applied;
            UsedSpeedLimits = usedSpeedLimits;
            AnyJointLimited = anyJointLimited;
            AnyJointBlocked = anyJointBlocked;
            AnyJointStartedOutsideLimits = anyJointStartedOutsideLimits;
            PositionError = positionError;
            OrientationError = orientationError;
        }

        internal IKApplicationStatus WithTargetErrors(float positionError, float orientationError)
        {
            return new IKApplicationStatus(
                Applied,
                UsedSpeedLimits,
                AnyJointLimited,
                AnyJointBlocked,
                AnyJointStartedOutsideLimits,
                positionError,
                orientationError);
        }
    }

    /// <summary>
    /// Writes a solver's desired pose to the chain's transforms, moving speed-limited joints toward it
    /// using the configured maximum speed multiplied by the step's delta time.
    /// </summary>
    /// <remarks>
    /// Without speed limits this is the same as <see cref="IKSolverOutput.ApplyTo"/>. With them, each
    /// limited joint turns or slides toward its solved pose by at most its speed times deltaTime.
    /// Movement is measured relative to the joint's IK parent, so a joint carried along by its parent
    /// spends none of its own budget. Joints are placed from root to tip on the parent's actual new pose.
    /// The current pose is read from the transforms on every call; nothing is stored between calls.
    /// </remarks>
    public sealed class IKPoseApplier
    {
        private Vector3[] _actualPositions = Array.Empty<Vector3>();
        private Quaternion[] _actualRotations = Array.Empty<Quaternion>();
        private Vector3[] _appliedPositions = Array.Empty<Vector3>();
        private Quaternion[] _appliedRotations = Array.Empty<Quaternion>();
        private bool[] _warnedUnsupported = Array.Empty<bool>();

        // Per-joint coordinates and remaining travel of the limited joints, filled before any pose is built.
        private Quaternion[] _currentDeviations = Array.Empty<Quaternion>();
        private Quaternion[] _desiredDeviations = Array.Empty<Quaternion>();
        private Vector3[] _currentOffsets = Array.Empty<Vector3>();
        private Vector3[] _desiredOffsets = Array.Empty<Vector3>();
        private float[] _angularRemaining = Array.Empty<float>();
        private float[] _linearRemaining = Array.Empty<float>();

        /// <summary>
        /// True when at least one joint in <paramref name="chain"/> has an enabled speed limit that its
        /// runtime constraints can honor.
        /// </summary>
        public static bool HasActiveSpeedLimits(SolverChain chain)
        {
            SolverJoint[] joints = chain.Joints;
            for (int i = 0; i < joints.Length; i++)
            {
                if (UsesAngularLimit(joints[i], out _) || UsesSegmentLimit(joints[i], out _))
                    return true;
            }

            return false;
        }

        /// <summary>
        /// Applies <paramref name="output"/> to its source transforms, honoring the speed limits bound to
        /// <paramref name="chain"/>.
        /// </summary>
        /// <param name="output">The desired pose. Must have been populated from <paramref name="chain"/>.</param>
        /// <param name="chain">The chain whose relationships, rest frames, and runtime constraints drive application.</param>
        /// <param name="deltaTime">
        /// Elapsed time for the speed budget. Zero, negative, or NaN gives a zero budget: limited joints
        /// do not move, except that hinge off-axis rotation and slider sideways offset are still removed.
        /// </param>
        /// <param name="synchronizeJoints">
        /// When a limited joint cannot reach its solved pose within its budget, slow every limited joint
        /// by the same factor so they all arrive together. No joint ever exceeds its own speed.
        /// </param>
        /// <returns>The joint-level application status. Target errors are left NaN for the caller to fill in.</returns>
        public IKApplicationStatus Apply(IKSolverOutput output, SolverChain chain, float deltaTime, bool synchronizeJoints = false)
        {
            if (output == null) throw new ArgumentNullException(nameof(output));
            if (chain == null) throw new ArgumentNullException(nameof(chain));
            if (output.JointCount != chain.Count)
                throw new ArgumentException(
                    $"Output joint count ({output.JointCount}) does not match chain joint count ({chain.Count}).",
                    nameof(output));

            int count = chain.Count;
            if (count == 0)
                return IKApplicationStatus.NotApplied;

            EnsureCapacity(count);
            WarnAboutUnsupportedLimits(chain);

            if (!HasActiveSpeedLimits(chain))
            {
                output.ApplyToSourceTransforms();
                return new IKApplicationStatus(true, false, false, false, false, float.NaN, float.NaN);
            }

            float dt = deltaTime > 0f ? deltaTime : 0f;
            SolverJoint[] joints = chain.Joints;

            // Snapshot every actual pose before writing anything: writing a parent moves its children.
            for (int i = 0; i < count; i++)
                joints[i].Transform.GetPositionAndRotation(out _actualPositions[i], out _actualRotations[i]);

            Quaternion rootParentRotation = chain.RootParentTransform != null
                ? chain.RootParentTransform.rotation
                : Quaternion.identity;

            bool anyLimited = false;
            bool anyBlocked = false;
            bool anyOutside = false;
            int endIdx = count - 1;

            // Pass 1: joint coordinates and remaining travel of every limited joint. These depend only on
            // the actual and desired poses, not on the poses applied this step.
            float commonScale = 1f;
            for (int i = 0; i < count; i++)
            {
                SolverJoint joint = joints[i];
                int parentIdx = joint.ParentIndex;
                Quaternion actualParentRotation = parentIdx >= 0 ? _actualRotations[parentIdx] : rootParentRotation;
                Quaternion desiredParentRotation = parentIdx >= 0 ? output.WorldRotations[parentIdx] : rootParentRotation;

                _linearRemaining[i] = -1f;
                if (parentIdx >= 0 && output.RequiresExplicitPosition[i] &&
                    UsesSegmentLimit(joint, out ISegmentMotionProvider segmentMotion))
                {
                    _currentOffsets[i] = Quaternion.Inverse(actualParentRotation)
                        * (_actualPositions[i] - _actualPositions[parentIdx]);
                    _desiredOffsets[i] = Quaternion.Inverse(desiredParentRotation)
                        * (output.WorldPositions[i] - output.WorldPositions[parentIdx]);
                    _linearRemaining[i] = segmentMotion.GetMotionDistance(_currentOffsets[i], _desiredOffsets[i]);
                    commonScale = Mathf.Min(commonScale, BudgetRatio(joint.MotionLimit.MaxLinearSpeed * dt, _linearRemaining[i]));
                }

                _angularRemaining[i] = -1f;
                if ((i < endIdx || output.WriteEndRotation) &&
                    UsesAngularLimit(joint, out IAngularMotionProvider angularMotion))
                {
                    _currentDeviations[i] = joint.ToConstraintDeviation(actualParentRotation, _actualRotations[i]);
                    _desiredDeviations[i] = joint.ToConstraintDeviation(desiredParentRotation, output.WorldRotations[i]);
                    _angularRemaining[i] = angularMotion.GetMotionDistance(_currentDeviations[i], _desiredDeviations[i]);
                    commonScale = Mathf.Min(commonScale, BudgetRatio(joint.MotionLimit.MaxAngularSpeed * dt, _angularRemaining[i]));
                }
            }

            // Pass 2: step the limited joints and rebuild the applied pose from root to tip.
            for (int i = 0; i < count; i++)
            {
                SolverJoint joint = joints[i];
                int parentIdx = joint.ParentIndex;
                bool hasParent = parentIdx >= 0;

                Quaternion actualParentRotation = hasParent ? _actualRotations[parentIdx] : rootParentRotation;
                Quaternion desiredParentRotation = hasParent ? output.WorldRotations[parentIdx] : rootParentRotation;
                Quaternion appliedParentRotation = hasParent ? _appliedRotations[parentIdx] : rootParentRotation;

                // Position: the root follows the solver; rigid segments keep their actual offset; explicit
                // (slider) segments move toward the solved offset.
                if (!hasParent)
                {
                    _appliedPositions[i] = output.WorldPositions[i];
                }
                else
                {
                    Vector3 localOffset;
                    if (output.RequiresExplicitPosition[i])
                    {
                        if (_linearRemaining[i] >= 0f && UsesSegmentLimit(joint, out ISegmentMotionProvider segmentMotion))
                        {
                            float budget = synchronizeJoints
                                ? _linearRemaining[i] * commonScale
                                : joint.MotionLimit.MaxLinearSpeed * dt;
                            JointMotionStep step = segmentMotion.StepLocalOffset(
                                _currentOffsets[i],
                                _desiredOffsets[i],
                                budget,
                                out localOffset);
                            Accumulate(step, ref anyLimited, ref anyBlocked, ref anyOutside);
                        }
                        else
                        {
                            localOffset = Quaternion.Inverse(desiredParentRotation)
                                * (output.WorldPositions[i] - output.WorldPositions[parentIdx]);
                        }
                    }
                    else
                    {
                        localOffset = Quaternion.Inverse(actualParentRotation)
                            * (_actualPositions[i] - _actualPositions[parentIdx]);
                    }

                    _appliedPositions[i] = _appliedPositions[parentIdx] + appliedParentRotation * localOffset;
                }

                // Rotation.
                bool writesRotation = i < endIdx || output.WriteEndRotation;
                if (!writesRotation)
                {
                    // Not written: the joint keeps its actual rotation relative to its parent.
                    _appliedRotations[i] = appliedParentRotation
                        * (Quaternion.Inverse(actualParentRotation) * _actualRotations[i]);
                }
                else if (_angularRemaining[i] >= 0f && UsesAngularLimit(joint, out IAngularMotionProvider angularMotion))
                {
                    float budget = synchronizeJoints
                        ? _angularRemaining[i] * commonScale
                        : joint.MotionLimit.MaxAngularSpeed * dt;
                    JointMotionStep step = angularMotion.StepDeviation(
                        _currentDeviations[i],
                        _desiredDeviations[i],
                        budget,
                        out Quaternion appliedDeviation);
                    Accumulate(step, ref anyLimited, ref anyBlocked, ref anyOutside);
                    _appliedRotations[i] = joint.FromConstraintDeviation(appliedParentRotation, appliedDeviation);
                }
                else
                {
                    _appliedRotations[i] = appliedParentRotation
                        * (Quaternion.Inverse(desiredParentRotation) * output.WorldRotations[i]);
                }
            }

            // Write in the same order as IKSolverOutput.ApplyTo.
            joints[0].Transform.position = _appliedPositions[0];

            int rotationLoopEnd = output.WriteEndRotation ? count : endIdx;
            for (int i = 0; i < rotationLoopEnd; i++)
                joints[i].Transform.rotation = _appliedRotations[i];

            for (int i = 1; i < count; i++)
            {
                if (output.RequiresExplicitPosition[i])
                    joints[i].Transform.position = _appliedPositions[i];
            }

            return new IKApplicationStatus(true, true, anyLimited, anyBlocked, anyOutside, float.NaN, float.NaN);
        }

        /// Fraction of the remaining travel that fits in the budget, at most 1.
        private static float BudgetRatio(float budget, float remaining)
        {
            return remaining > budget ? budget / remaining : 1f;
        }

        private static bool UsesAngularLimit(SolverJoint joint, out IAngularMotionProvider provider)
        {
            provider = joint.MotionLimit.LimitSpeed ? joint.Angular as IAngularMotionProvider : null;
            return provider != null;
        }

        private static bool UsesSegmentLimit(SolverJoint joint, out ISegmentMotionProvider provider)
        {
            provider = joint.MotionLimit.LimitSpeed && joint.Segment.RequiresExplicitTransformPosition
                ? joint.Segment as ISegmentMotionProvider
                : null;
            return provider != null;
        }

        private static void Accumulate(in JointMotionStep step, ref bool anyLimited, ref bool anyBlocked, ref bool anyOutside)
        {
            anyLimited |= step.Status == JointMotionStatus.Limited;
            anyBlocked |= step.Status == JointMotionStatus.Blocked;
            anyOutside |= step.StartedOutsideLimits;
        }

        /// Logs once per joint when Limit Speed is enabled on a joint whose runtime constraints cannot honor it.
        private void WarnAboutUnsupportedLimits(SolverChain chain)
        {
            SolverJoint[] joints = chain.Joints;
            for (int i = 0; i < joints.Length; i++)
            {
                SolverJoint joint = joints[i];
                if (_warnedUnsupported[i] || !joint.MotionLimit.LimitSpeed)
                    continue;

                if (joint.Angular is IAngularMotionProvider || joint.Segment is ISegmentMotionProvider)
                    continue;

                _warnedUnsupported[i] = true;
                Debug.LogWarning(
                    $"[OpenIK] \"{joint.Transform.name}\" has Limit Speed enabled, but its joint type does not support speed limits. It moves without a speed limit.",
                    joint.Transform);
            }
        }

        private void EnsureCapacity(int count)
        {
            if (_actualPositions.Length == count)
                return;

            _actualPositions = new Vector3[count];
            _actualRotations = new Quaternion[count];
            _appliedPositions = new Vector3[count];
            _appliedRotations = new Quaternion[count];
            _warnedUnsupported = new bool[count];
            _currentDeviations = new Quaternion[count];
            _desiredDeviations = new Quaternion[count];
            _currentOffsets = new Vector3[count];
            _desiredOffsets = new Vector3[count];
            _angularRemaining = new float[count];
            _linearRemaining = new float[count];
        }
    }
}
