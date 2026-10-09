using System.Collections.Generic;
using UnityEngine;

namespace OpenIK
{
    /// <summary>
    /// Shared lifecycle owner for an IK chain: builds <see cref="SolverJoint"/> entries from
    /// a list of transforms, captures rest-pose data, and manages per-frame constraint rebinding.
    ///
    /// Used by <see cref="FABRIKSolver"/>, <see cref="JacobianIKSolver"/>, and
    /// <see cref="CCDIKSolver"/>.
    /// </summary>
    public sealed class SolverChain
    {
        public SolverJoint[] Joints { get; private set; } = System.Array.Empty<SolverJoint>();
        public Transform RootParentTransform { get; private set; }
        public float ChainLength { get; private set; }

        public int Count => Joints.Length;

        /// <summary>
        /// Initializes the chain in the valid setup order.
        /// </summary>
        public void Initialize(IReadOnlyList<Transform> chainJoints)
        {
            Build(chainJoints);
            ComputeRestLocalOffsets();
            ComputeRestPoses();
            CaptureRestLocalRotations();
            RefreshConstraintBindings();
            ApplyRuntimeConfigs();
            ComputeChainLength();
        }

        /// <summary>
        /// Populates <see cref="Joints"/> from the authored transform list, assigns parent indices,
        /// resolves the root's non-chain parent, and calls <see cref="ConstrainedJoint.Initialize"/>
        /// on every constrained joint component.
        /// </summary>
        public void Build(IReadOnlyList<Transform> chainJoints)
        {
            Joints = new SolverJoint[chainJoints.Count];
            for (int i = 0; i < chainJoints.Count; i++)
            {
                Transform t = chainJoints[i];
                var constrainedJoint = t.GetComponent<ConstrainedJoint>();
                Joints[i] = new SolverJoint
                {
                    Transform = t,
                    SolverPosition = t.position,
                    SolverRotation = t.rotation,
                    ParentIndex = i == 0 ? -1 : i - 1,
                    ConstrainedJoint = constrainedJoint
                };
            }

            RootParentTransform = Joints.Length > 0 ? Joints[0].Transform.parent : null;

            foreach (SolverJoint joint in Joints)
                joint.ConstrainedJoint?.Initialize();
        }

        /// Wires each constrained joint's IK parent transform and saves its rest pose.
        public void ComputeRestPoses()
        {
            for (int i = 0; i < Joints.Length; i++)
            {
                Transform parentTransform = GetSetupParentTransform(i);
                ConstrainedJoint cj = Joints[i].ConstrainedJoint;
                if (cj == null)
                    continue;

                cj.IKParentTransform = parentTransform;
                if (parentTransform != null)
                    cj.ComputeRestPose(parentTransform.rotation);
            }
        }

        /// Captures the joint's current rotation expressed in its parent's frame. Must run after <see cref="Build"/>.
        public void CaptureRestLocalRotations()
        {
            for (int i = 0; i < Joints.Length; i++)
            {
                Transform parentTransform = GetSetupParentTransform(i);
                Quaternion parentRotation = parentTransform != null ? parentTransform.rotation : Quaternion.identity;
                Joints[i].RestLocalRotation = Quaternion.Inverse(parentRotation) * Joints[i].Transform.rotation;
            }
        }

        /// Captures each joint's rest-pose offset in its parent's local frame. Must run after <see cref="Build"/>.
        public void ComputeRestLocalOffsets()
        {
            for (int i = 0; i < Joints.Length; i++)
            {
                Transform parentTransform = GetSetupParentTransform(i);
                if (parentTransform == null)
                {
                    Joints[i].RestLocalOffset = Vector3.zero;
                    Joints[i].RestLocalOffsetLength = 0f;
                    continue;
                }

                Vector3 worldOffset = Joints[i].Transform.position - parentTransform.position;
                Joints[i].RestLocalOffset = Quaternion.Inverse(parentTransform.rotation) * worldOffset;
                Joints[i].RestLocalOffsetLength = Joints[i].RestLocalOffset.magnitude;
            }
        }

        /// Sum of each segment's max reach (used for unreachable-target clamping / singularity thresholds).
        public void ComputeChainLength()
        {
            ChainLength = 0f;
            for (int i = 1; i < Joints.Length; i++)
                ChainLength += Joints[i].Segment.MaxReach;
        }

        /// Lets each joint component recompute any inspector-driven cached data for the upcoming solve.
        public void UpdateConstraints()
        {
            for (int i = 0; i < Joints.Length; i++)
                Joints[i].ConstrainedJoint?.UpdateConstraints();
        }

        /// <summary>
        /// Rebinds every joint's runtime segment/angular constraints to match the currently-active
        /// <see cref="ConstrainedJoint"/> component. Returns true if at least one joint changed bindings.
        /// </summary>
        /// <returns>True when any joint was rebound; false when all existing bindings remained the same.</returns>
        public bool RefreshConstraintBindings()
        {
            bool anyBindingsChanged = false;
            for (int i = 0; i < Joints.Length; i++)
            {
                anyBindingsChanged |= Joints[i].RefreshConstraintBindings(
                    CreateSegmentSetupData(i),
                    CreateAngularSetupData(i));
            }

            return anyBindingsChanged;
        }

        /// Pushes the latest per-joint config values into already-bound runtime constraint instances.
        public void ApplyRuntimeConfigs()
        {
            for (int i = 0; i < Joints.Length; i++)
                Joints[i].ApplyRuntimeConfigs();
        }

        /// <summary>
        /// Seeds every joint's solver pose from its <see cref="SolverJoint.Transform"/> and then
        /// synchronizes segment runtime state from those solver positions.
        /// </summary>
        public void SyncSolverStateFromTransforms()
        {
            for (int i = 0; i < Joints.Length; i++)
            {
                Joints[i].SolverPosition = Joints[i].Transform.position;
                Joints[i].SolverRotation = Joints[i].Transform.rotation;
            }

            SyncSegmentsFromSolverPositions();
        }

        /// <summary>
        /// Rebuilds every joint's solver pose from rest-pose data. The root takes its rotation from
        /// the current root-parent transform composed with its rest local rotation; downstream joints
        /// rebuild from <see cref="SolverJoint.RestLocalRotation"/> and each segment's rest offset.
        /// Each segment is reset to its rest state.
        /// </summary>
        public void SyncSolverStateFromRestPose()
        {
            if (Joints.Length == 0)
                return;

            Quaternion rootParentRotation = RootParentTransform != null ? RootParentTransform.rotation : Quaternion.identity;
            Joints[0].SolverRotation = rootParentRotation * Joints[0].RestLocalRotation;
            Joints[0].SolverPosition = Joints[0].Transform.position;

            for (int i = 1; i < Joints.Length; i++)
            {
                int parentIdx = Joints[i].ParentIndex;
                Joints[i].SolverRotation = Joints[parentIdx].SolverRotation * Joints[i].RestLocalRotation;
                Joints[i].Segment.ResetToRest();
                Joints[i].SolverPosition = Joints[parentIdx].SolverPosition
                    + Joints[parentIdx].SolverRotation * Joints[i].Segment.GetRestLocalOffset();
            }
        }

        /// <summary>
        /// Rebuilds every joint's solver pose from per-joint poses relative to the IK parent, anchored
        /// at the root's current transform position and the current root-parent rotation, then syncs
        /// segment runtime state from the result.
        /// </summary>
        /// <param name="localRotations">Each joint's rotation relative to its IK parent (the root parent for the root).</param>
        /// <param name="localOffsets">Each joint's offset from its IK parent in the parent's rotation frame. Index 0 is ignored.</param>
        public void SyncSolverStateFromLocalPose(IReadOnlyList<Quaternion> localRotations, IReadOnlyList<Vector3> localOffsets)
        {
            if (Joints.Length == 0)
                return;

            Quaternion rootParentRotation = RootParentTransform != null ? RootParentTransform.rotation : Quaternion.identity;
            Joints[0].SolverRotation = rootParentRotation * localRotations[0];
            Joints[0].SolverPosition = Joints[0].Transform.position;

            for (int i = 1; i < Joints.Length; i++)
            {
                int parentIdx = Joints[i].ParentIndex;
                Joints[i].SolverRotation = Joints[parentIdx].SolverRotation * localRotations[i];
                Joints[i].SolverPosition = Joints[parentIdx].SolverPosition
                    + Joints[parentIdx].SolverRotation * localOffsets[i];
            }

            SyncSegmentsFromSolverPositions();
        }

        /// <summary>
        /// Captures slider offsets implied by the current solver positions back into each segment's
        /// runtime state. Rigid segments ignore the synced offset.
        /// </summary>
        public void SyncSegmentsFromSolverPositions()
        {
            for (int i = 1; i < Joints.Length; i++)
            {
                int parentIdx = Joints[i].ParentIndex;
                Quaternion parentRotation = Joints[parentIdx].SolverRotation;
                Vector3 localOffset = Quaternion.Inverse(parentRotation)
                    * (Joints[i].SolverPosition - Joints[parentIdx].SolverPosition);
                Joints[i].Segment.SyncFromLocalOffset(localOffset);
            }
        }

        /// <summary>
        /// Applies <paramref name="delta"/> to every solver rotation downstream of
        /// <paramref name="jointIdx"/> and rebuilds their solver positions from the parent solver
        /// pose plus current segment offset. No-op when <paramref name="jointIdx"/> is the end joint.
        /// </summary>
        public void RotateAndRepositionDownstream(int jointIdx, Quaternion delta)
        {
            int endIdx = Joints.Length - 1;
            if (jointIdx >= endIdx)
                return;

            int currentIdx = jointIdx;
            int childIdx = currentIdx + 1;
            while (true)
            {
                Joints[childIdx].SolverRotation = delta * Joints[childIdx].SolverRotation;
                Joints[childIdx].SolverPosition = Joints[currentIdx].SolverPosition
                    + Joints[currentIdx].SolverRotation * Joints[childIdx].Segment.GetCurrentLocalOffset();

                if (childIdx == endIdx)
                    break;

                currentIdx = childIdx;
                childIdx = currentIdx + 1;
            }
        }

        /// The transform that acts as the IK parent during setup (unity-parent for root, previous joint otherwise).
        public Transform GetSetupParentTransform(int jointIndex)
        {
            if (jointIndex == 0)
                return RootParentTransform;
            return Joints[jointIndex - 1].Transform;
        }

        private ISegmentConstraint.SetupData CreateSegmentSetupData(int jointIndex)
        {
            Transform parentTransform = GetSetupParentTransform(jointIndex);
            Quaternion parentRotation = parentTransform != null
                ? parentTransform.rotation
                : Quaternion.identity;

            return new ISegmentConstraint.SetupData(
                Joints[jointIndex].RestLocalOffset,
                Joints[jointIndex].RestLocalOffsetLength,
                parentRotation,
                Joints[jointIndex].Transform.rotation,
                parentTransform != null);
        }

        private IAngularConstraint.SetupData CreateAngularSetupData(int jointIndex)
        {
            return new IAngularConstraint.SetupData(Joints[jointIndex].RestLocalRotation);
        }
    }
}
