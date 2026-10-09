using System;
using System.Collections.Generic;
using UnityEngine;

namespace OpenIK
{
    public enum SolverDofType
    {
        Rotation,
        Translation
    }

    /// <summary>
    /// Selects whether a solver writes its solved pose back to the scene transforms.
    /// </summary>
    public enum SolveMode
    {
        /// <summary>Solve and write the solved pose to the chain's transforms (default behavior).</summary>
        SolveAndApply,
        /// <summary>Solve only; expose the solved pose via the solver's output without touching transforms.</summary>
        SolveOnly
    }

    /// <summary>
    /// Standardized world-space output of an IK solve.
    /// <para>
    /// One instance per solver, populated each frame with the solved chain pose plus convergence
    /// statistics. The internal arrays are <strong>reused</strong> between frames to avoid per-frame
    /// Garbage Collection.
    /// </para>
    /// <remarks>
    /// Subscribers to a solver's <c>Solved</c> event must clone the data they want to retain
    /// across frames; the next solve will overwrite the buffers in place.
    /// </remarks>
    /// </summary>
    public sealed class IKSolverOutput
    {
        private Vector3[] _positions = Array.Empty<Vector3>();
        private Quaternion[] _rotations = Array.Empty<Quaternion>();
        private bool[] _requiresExplicitPosition = Array.Empty<bool>();
        private Transform[] _chainTransforms = Array.Empty<Transform>();
        private int _jointCount;
        private bool _writeEndRotation;

        /// <summary>Number of joints in the chain at the time of the last <c>Populate</c>.</summary>
        public int JointCount => _jointCount;

        /// <summary>World-space joint positions in root-to-end order. Reused buffer; do not retain.</summary>
        public IReadOnlyList<Vector3> WorldPositions => _positions;

        /// <summary>World-space joint rotations in root-to-end order. Reused buffer; do not retain.</summary>
        public IReadOnlyList<Quaternion> WorldRotations => _rotations;

        /// <summary>
        /// Per-joint flag mirroring <see cref="ISegmentConstraint.RequiresExplicitTransformPosition"/>
        /// at populate time.
        /// </summary>
        public IReadOnlyList<bool> RequiresExplicitPosition => _requiresExplicitPosition;

        /// <summary>
        /// Source <see cref="Transform"/> for each chain index, in root-to-end order.
        /// </summary>
        public IReadOnlyList<Transform> ChainTransforms => _chainTransforms;

        /// <summary>Number of iterations executed by the solver.</summary>
        public int IterationsUsed { get; private set; }

        /// <summary>Final residual error reported by the solver.</summary>
        public float FinalError { get; private set; }

        /// <summary>True when the solver converged within the tolerance.</summary>
        public bool Converged { get; private set; }

        /// <summary>
        /// True when the solver explicitly solved for the end-effector's rotation. When true,
        /// <see cref="ApplyTo"/> writes the end joint's rotation; when false the end rotation is
        /// left untouched (its world rotation falls out of its parent's solved orientation).
        /// </summary>
        public bool WriteEndRotation => _writeEndRotation;

        /// <summary>
        /// Fills the internal buffers from <paramref name="chain"/>'s solver state and records
        /// convergence statistics. Resizes only when the chain length changed; otherwise overwrites
        /// the existing buffers in place.
        /// </summary>
        internal void Populate(SolverChain chain, int iterations, float error, bool converged, bool writeEndRotation = false)
        {
            int count = chain.Count;
            EnsureCapacity(count);
            _jointCount = count;

            for (int i = 0; i < count; i++)
            {
                SolverJoint joint = chain.Joints[i];
                _positions[i] = joint.SolverPosition;
                _rotations[i] = joint.SolverRotation;
                _requiresExplicitPosition[i] = joint.Segment != null && joint.Segment.RequiresExplicitTransformPosition;
                _chainTransforms[i] = joint.Transform;
            }

            IterationsUsed = iterations;
            FinalError = error;
            Converged = converged;
            _writeEndRotation = writeEndRotation;
        }

        /// <summary>
        /// Applies this output to <paramref name="transforms"/>. Writes the root position, every
        /// non-end joint's rotation (and the end joint's rotation when <see cref="WriteEndRotation"/>
        /// is true), and any slider segments' world-space positions.
        /// </summary>
        /// <param name="transforms">Target transform list, root-to-end. Length must match <see cref="JointCount"/>.</param>
        public void ApplyTo(IList<Transform> transforms)
        {
            if (transforms == null) throw new ArgumentNullException(nameof(transforms));
            if (transforms.Count != _jointCount)
                throw new ArgumentException(
                    $"Transform list length ({transforms.Count}) does not match output joint count ({_jointCount}).",
                    nameof(transforms));

            int endIdx = _jointCount - 1;
            if (endIdx < 0) return;

            transforms[0].position = _positions[0];

            int rotationLoopEnd = _writeEndRotation ? _jointCount : endIdx;
            for (int i = 0; i < rotationLoopEnd; i++)
                transforms[i].rotation = _rotations[i];

            for (int i = 1; i <= endIdx; i++)
            {
                if (_requiresExplicitPosition[i])
                    transforms[i].position = _positions[i];
            }
        }

        /// Applies this output immediately to the transforms it was populated from.
        internal void ApplyToSourceTransforms()
        {
            ApplyTo(_chainTransforms);
        }

        /// <summary>
        /// True when this output was populated from <paramref name="chain"/>'s current joints and all of
        /// its source transforms still exist.
        /// </summary>
        internal bool MatchesChain(SolverChain chain)
        {
            if (chain == null || _jointCount == 0 || _jointCount != chain.Count)
                return false;

            for (int i = 0; i < _jointCount; i++)
            {
                if (_chainTransforms[i] == null || _chainTransforms[i] != chain.Joints[i].Transform)
                    return false;
            }

            return true;
        }

        private void EnsureCapacity(int count)
        {
            if (_positions.Length == count)
                return;

            Array.Resize(ref _positions, count);
            Array.Resize(ref _rotations, count);
            Array.Resize(ref _requiresExplicitPosition, count);
            Array.Resize(ref _chainTransforms, count);
        }
    }

    /// <summary>
    /// Runtime record for one transform in an IK chain.
    /// </summary>
    /// <remarks>
    /// Built by <see cref="SolverChain"/> and used by OpenIK solvers to store rest-pose data,
    /// intermediate solver state, and the runtime segment/angular constraints bound to the joint.
    /// </remarks>
    public sealed class SolverJoint
    {
        public Transform Transform;
        public int ParentIndex;
        public ConstrainedJoint ConstrainedJoint;

        public Vector3 SolverPosition;
        public Quaternion SolverRotation;

        public Vector3 RestLocalOffset;
        public float RestLocalOffsetLength;
        public Quaternion RestLocalRotation;

        public ISegmentConstraint Segment;
        public IAngularConstraint Angular;

        /// <summary>Speed-limit settings of the bound joint component; unlimited for the default fallback.</summary>
        public JointMotionLimit MotionLimit;

        private bool _bindingsUseJointComponent;

        /// <summary>
        /// Expresses <paramref name="worldRotation"/> as this joint's deviation from its rest pose,
        /// relative to <paramref name="parentRotation"/> and in the constraint-axis frame of
        /// <see cref="Angular"/>. This is the frame every <see cref="IAngularConstraint"/> clamps in.
        /// </summary>
        public Quaternion ToConstraintDeviation(Quaternion parentRotation, Quaternion worldRotation)
        {
            Quaternion deviationFromRest = Quaternion.Inverse(parentRotation * RestLocalRotation) * worldRotation;
            Quaternion axisRotation = Angular.ConstraintAxisRotation;
            return Quaternion.Inverse(axisRotation) * deviationFromRest * axisRotation;
        }

        /// <summary>
        /// Inverse of <see cref="ToConstraintDeviation"/>: rebuilds the joint's world rotation from a
        /// constraint-frame deviation and its IK parent's world rotation.
        /// </summary>
        public Quaternion FromConstraintDeviation(Quaternion parentRotation, Quaternion constraintDeviation)
        {
            Quaternion axisRotation = Angular.ConstraintAxisRotation;
            Quaternion deviationFromRest = axisRotation * constraintDeviation * Quaternion.Inverse(axisRotation);
            return parentRotation * RestLocalRotation * deviationFromRest;
        }

        /// <summary>
        /// Clamps <paramref name="desiredRotation"/> against <see cref="Angular"/> relative to
        /// <paramref name="parentRotation"/>. Constraints that preserve twist read it from
        /// <paramref name="currentRotation"/>.
        /// </summary>
        /// <returns>
        /// <paramref name="desiredRotation"/> unchanged when it already satisfies the constraint;
        /// otherwise the clamped world rotation.
        /// </returns>
        public Quaternion ClampRotation(Quaternion parentRotation, Quaternion currentRotation, Quaternion desiredRotation)
        {
            Quaternion? currentDeviation = Angular.NeedsCurrentDeviation
                ? ToConstraintDeviation(parentRotation, currentRotation)
                : null;
            Quaternion desiredDeviation = ToConstraintDeviation(parentRotation, desiredRotation);

            Quaternion clampedDeviation = Angular.ClampDeviation(desiredDeviation, currentDeviation);

            // Keep the original rotation when nothing was clamped to avoid float drift.
            if (clampedDeviation == desiredDeviation)
                return desiredRotation;

            return FromConstraintDeviation(parentRotation, clampedDeviation);
        }

        private bool ShouldUseJointComponent()
        {
            return ConstrainedJoint != null && ConstrainedJoint.jointIsEnabled;
        }

        /// <summary>
        /// Ensures the runtime constraints use either the enabled joint component or the default fallback.
        /// </summary>
        /// <returns>True when the runtime constraints were rebound, False when the existing bindings were already current.</returns>
        public bool RefreshConstraintBindings(
            in ISegmentConstraint.SetupData segmentSetupData,
            in IAngularConstraint.SetupData angularSetupData)
        {
            bool useJointComponent = ShouldUseJointComponent();

            bool bindingsAreCurrent = _bindingsUseJointComponent == useJointComponent && Segment != null && Angular != null;
            if (bindingsAreCurrent)
                return false;

            _bindingsUseJointComponent = useJointComponent;
            Segment = useJointComponent
                ? ConstrainedJoint.CreateSegmentConstraint(segmentSetupData)
                : new RigidSegmentConstraint(segmentSetupData);
            Angular = useJointComponent
                ? ConstrainedJoint.CreateAngularConstraint(angularSetupData)
                : new FreeAngularConstraint();

            return true;
        }

        /// Pushes the latest serialized joint settings, including speed limits, into the already-bound
        /// runtime constraints.
        public void ApplyRuntimeConfigs()
        {
            if (!_bindingsUseJointComponent)
            {
                MotionLimit = JointMotionLimit.Unlimited;
                return;
            }

            ConstrainedJoint.ApplySegmentConfig(Segment);
            ConstrainedJoint.ApplyAngularConfig(Angular);
            MotionLimit = ConstrainedJoint.GetMotionLimit();
        }
    }


    /// <summary>
    /// Adapts a Constraint (<see cref="IAngularConstraint"/> or <see cref="ISegmentConstraint"/>)
    /// into one or more scalar Jacobian DOF scalars for the solver.
    /// </summary>
    public interface IJacobianDofProvider
    {
        /// <summary>
        /// Runtime context for Jacobian DOF queries and mutations.
        /// </summary>
        /// <remarks>
        /// <see cref="Context.Joint"/> is the joint owning the DOF.
        /// <see cref="Context.Parent"/> is provided for world-space conversion and can be null for root-level joints.
        /// </remarks>
        public readonly struct Context
        {
            public readonly SolverJoint Joint;
            public readonly SolverJoint Parent;

            public Context(SolverJoint joint, SolverJoint parent)
            {
                Joint = joint;
                Parent = parent;
            }
        }

        /// <summary> Number of DOFs exposed by this provider (rows in Jacobian). </summary>
        int DofCount { get; }

        SolverDofType GetDofType(int dofIndex);

        /// <summary>
        /// Returns the current world-space axis/direction used for the Jacobian column of this DOF.
        /// Implementations MUST derive the axis from <see cref="SolverJoint.SolverRotation"/> on
        /// <c>Context.Joint</c> / <c>Context.Parent</c>, and NOT from <see cref="SolverJoint.Transform"/>.
        /// </summary>
        Vector3 GetDofAxisWorld(int dofIndex, in Context context);

        /// <summary>
        /// Applies the solver step for this DOF, using step-size delta.
        /// Implementations must mutate <see cref="SolverJoint.SolverPosition"/> /
        /// <see cref="SolverJoint.SolverRotation"/> on <c>Context.Joint</c>, and NOT touch
        /// <see cref="SolverJoint.Transform"/>.
        /// </summary>
        void ApplyDofDelta(int dofIndex, float delta, in Context context);
    }

    internal static class SolverSetupValidation
    {
        public static bool Validate(
            MonoBehaviour solver,
            string solverName,
            Transform target,
            IReadOnlyList<Transform> chainJoints)
        {
            string error = null;
            if (target == null)
                error = "target is not assigned.";
            else if (chainJoints == null || chainJoints.Count < 2)
                error = "at least two chain joints are required.";
            else
            {
                for (int i = 0; i < chainJoints.Count; i++)
                {
                    if (chainJoints[i] == null)
                    {
                        error = $"chainJoints[{i}] is not assigned.";
                        break;
                    }
                }
            }

            if (error == null)
                return true;

            Debug.LogError($"[OpenIK] {solverName} disabled: {error}", solver);
            solver.enabled = false;
            return false;
        }
    }

}
