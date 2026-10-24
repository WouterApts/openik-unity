using System;
using System.Collections.Generic;
using UnityEngine;

namespace OpenIK
{
    /// <summary>
    /// Shared lifecycle owner for an IK chain: builds <see cref="SolverJoint"/> entries from
    /// a list of transforms, captures rest-pose data, and manages per-frame constraint rebinding.
    /// </summary>
    /// <remarks>
    /// The joints usually form a single chain, where each joint's parent is the joint before it. Built
    /// with parent indices, they can form a tree instead. Every parent comes before its children in
    /// <see cref="Joints"/>.
    /// </remarks>
    public sealed class SolverChain
    {
        public SolverJoint[] Joints { get; private set; } = Array.Empty<SolverJoint>();

        /// <summary>
        /// The root joint's parent transform, or null when the root has none. A root without a parent is
        /// constrained relative to world space.
        /// </summary>
        public Transform RootParentTransform { get; private set; }

        /// <summary>
        /// World rotation of <see cref="RootParentTransform"/>, or identity without one. Captured each
        /// time the solver state is synced, so a solve never reads it from the scene mid-solve.
        /// </summary>
        public Quaternion RootParentRotation { get; private set; } = Quaternion.identity;

        /// <summary>Sum of every segment's maximum reach. For a tree, use <see cref="GetPathLength"/> per tip.</summary>
        public float ChainLength { get; private set; }

        public int Count => Joints.Length;

        /// <summary>True when any joint's parent is not the joint before it, so the joints form a tree.</summary>
        public bool IsTree { get; private set; }

        // Children of joint i, in index order: _children[_childStart[i]] to _children[_childStart[i] + _childCount[i] - 1].
        private int[] _childStart = Array.Empty<int>();
        private int[] _childCount = Array.Empty<int>();
        private int[] _children = Array.Empty<int>();

        // Summed maximum reach of the segments from the root to each joint.
        private float[] _pathLength = Array.Empty<float>();

        // Marks the joints RotateAndRepositionDownstream moved on a tree. All false between calls.
        private bool[] _moved = Array.Empty<bool>();

        /// <summary>
        /// Initializes the chain in the valid setup order.
        /// </summary>
        public void Initialize(IReadOnlyList<Transform> chainJoints)
        {
            Initialize(chainJoints, null);
        }

        /// <summary>Initializes a tree of joints in the valid setup order.</summary>
        /// <param name="joints">The joint transforms, each listed after its parent.</param>
        /// <param name="parentIndices">
        /// The index of each joint's parent in <paramref name="joints"/>, and -1 for the first joint. Null
        /// makes each joint the child of the joint before it.
        /// </param>
        /// <exception cref="ArgumentException">Thrown when a parent index does not point to an earlier joint.</exception>
        public void Initialize(IReadOnlyList<Transform> joints, IReadOnlyList<int> parentIndices)
        {
            Build(joints, parentIndices);
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
            Build(chainJoints, null);
        }

        /// <summary>
        /// Populates <see cref="Joints"/> from a list of transforms and their parent indices, builds the
        /// child lists, resolves the root's non-chain parent, and calls
        /// <see cref="ConstrainedJoint.Initialize"/> on every constrained joint component.
        /// </summary>
        /// <param name="joints">The joint transforms, each listed after its parent.</param>
        /// <param name="parentIndices">
        /// The index of each joint's parent in <paramref name="joints"/>, and -1 for the first joint. Null
        /// makes each joint the child of the joint before it.
        /// </param>
        /// <exception cref="ArgumentException">Thrown when a parent index does not point to an earlier joint.</exception>
        public void Build(IReadOnlyList<Transform> joints, IReadOnlyList<int> parentIndices)
        {
            if (parentIndices != null && parentIndices.Count != joints.Count)
                throw new ArgumentException($"Got {parentIndices.Count} parent indices for {joints.Count} joints.", nameof(parentIndices));

            Joints = new SolverJoint[joints.Count];
            IsTree = false;
            for (int i = 0; i < joints.Count; i++)
            {
                int parentIndex = parentIndices != null ? parentIndices[i] : i - 1;
                if (i == 0 ? parentIndex != -1 : parentIndex < 0 || parentIndex >= i)
                {
                    throw new ArgumentException(
                        $"Joint {i} has parent index {parentIndex}. The first joint needs -1, and every other joint the index of an earlier joint.",
                        nameof(parentIndices));
                }

                IsTree |= parentIndex != i - 1;
                Transform t = joints[i];
                var constrainedJoint = t.GetComponent<ConstrainedJoint>();
                Joints[i] = new SolverJoint
                {
                    Transform = t,
                    SolverPosition = t.position,
                    SolverRotation = t.rotation,
                    ParentIndex = parentIndex,
                    ConstrainedJoint = constrainedJoint
                };
            }

            BuildChildLists();
            _pathLength = new float[Joints.Length];
            _moved = new bool[Joints.Length];

            RootParentTransform = Joints.Length > 0 ? Joints[0].Transform.parent : null;
            CaptureRootParentRotation();

            foreach (SolverJoint joint in Joints)
                joint.ConstrainedJoint?.Initialize();
        }

        /// <summary>The number of joints whose parent is joint <paramref name="jointIndex"/>.</summary>
        public int GetChildCount(int jointIndex) => _childCount[jointIndex];

        /// <summary>The index of a child of joint <paramref name="jointIndex"/>. Children are in index order.</summary>
        /// <param name="jointIndex">The parent joint's index.</param>
        /// <param name="childNumber">Which child, from 0 to <see cref="GetChildCount"/> minus 1.</param>
        public int GetChild(int jointIndex, int childNumber) => _children[_childStart[jointIndex] + childNumber];

        /// <summary>True when no joint has joint <paramref name="jointIndex"/> as its parent, such as the end of a chain.</summary>
        public bool IsLeaf(int jointIndex) => _childCount[jointIndex] == 0;

        /// <summary>
        /// The summed maximum reach of the segments from the root to joint <paramref name="jointIndex"/>,
        /// as of the last <see cref="ComputeChainLength"/>.
        /// </summary>
        public float GetPathLength(int jointIndex) => _pathLength[jointIndex];

        private void BuildChildLists()
        {
            int count = Joints.Length;
            _childStart = new int[count];
            _childCount = new int[count];
            _children = new int[Math.Max(0, count - 1)];

            for (int i = 1; i < count; i++)
                _childCount[Joints[i].ParentIndex]++;

            int start = 0;
            for (int i = 0; i < count; i++)
            {
                _childStart[i] = start;
                start += _childCount[i];
            }

            // Fill in index order, so each joint's children stay sorted.
            var filled = new int[count];
            for (int i = 1; i < count; i++)
            {
                int parent = Joints[i].ParentIndex;
                _children[_childStart[parent] + filled[parent]++] = i;
            }
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
                    cj.ComputeRestPose(parentTransform.position, parentTransform.rotation);
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

        /// <summary>
        /// Sums the segments' maximum reach into <see cref="ChainLength"/>, and from the root to each joint
        /// for <see cref="GetPathLength"/>. Solvers use them to clamp unreachable targets.
        /// </summary>
        public void ComputeChainLength()
        {
            ChainLength = 0f;
            if (Joints.Length == 0)
                return;

            _pathLength[0] = 0f;
            for (int i = 1; i < Joints.Length; i++)
            {
                float reach = Joints[i].Segment.MaxReach;
                ChainLength += reach;
                _pathLength[i] = _pathLength[Joints[i].ParentIndex] + reach;
            }
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
            CaptureRootParentRotation();
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

            CaptureRootParentRotation();
            Joints[0].SolverRotation = RootParentRotation * Joints[0].RestLocalRotation;
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

        // Sets the solver state to a previous solve's output, keeping each joint's pose relative to its
        // IK parent. The pose is placed at the root's current position and the current root-parent
        // rotation, so it follows the chain if its base moved since that solve. solvedRootParentRotation
        // is the root-parent rotation the output was solved with. The output must match this chain.
        internal void SyncSolverStateFromOutput(IKSolverOutput output, Quaternion solvedRootParentRotation)
        {
            if (Joints.Length == 0)
                return;

            IReadOnlyList<Vector3> positions = output.WorldPositions;
            IReadOnlyList<Quaternion> rotations = output.WorldRotations;

            Quaternion rootLocalRotation = Quaternion.Normalize(Quaternion.Inverse(solvedRootParentRotation) * rotations[0]);
            CaptureRootParentRotation();
            Joints[0].SolverRotation = RootParentRotation * rootLocalRotation;
            Joints[0].SolverPosition = Joints[0].Transform.position;

            for (int i = 1; i < Joints.Length; i++)
            {
                int parentIdx = Joints[i].ParentIndex;
                Quaternion solvedParentRotation = rotations[parentIdx];
                Quaternion localRotation = Quaternion.Normalize(Quaternion.Inverse(solvedParentRotation) * rotations[i]);
                Vector3 localOffset = Quaternion.Inverse(solvedParentRotation) * (positions[i] - positions[parentIdx]);

                Joints[i].SolverRotation = Joints[parentIdx].SolverRotation * localRotation;
                Joints[i].SolverPosition = Joints[parentIdx].SolverPosition
                    + Joints[parentIdx].SolverRotation * localOffset;
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
        /// pose plus current segment offset. On a tree, only the joint's own subtree moves. No-op
        /// when <paramref name="jointIdx"/> has no children.
        /// </summary>
        public void RotateAndRepositionDownstream(int jointIdx, Quaternion delta)
        {
            int endIdx = Joints.Length - 1;
            if (jointIdx >= endIdx)
                return;

            if (IsTree)
            {
                RotateAndRepositionSubtree(jointIdx, delta);
                return;
            }

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

        // Parents come before their children, so one pass in index order reaches the whole subtree:
        // a joint moves when its parent moved.
        private void RotateAndRepositionSubtree(int jointIdx, Quaternion delta)
        {
            if (_childCount[jointIdx] == 0)
                return;

            _moved[jointIdx] = true;
            for (int i = jointIdx + 1; i < Joints.Length; i++)
            {
                int parentIdx = Joints[i].ParentIndex;
                if (!_moved[parentIdx])
                    continue;

                _moved[i] = true;
                Joints[i].SolverRotation = delta * Joints[i].SolverRotation;
                Joints[i].SolverPosition = Joints[parentIdx].SolverPosition
                    + Joints[parentIdx].SolverRotation * Joints[i].Segment.GetCurrentLocalOffset();
            }

            Array.Clear(_moved, jointIdx, Joints.Length - jointIdx);
        }

        // Turns the joints from start up to, but not including, end by swivel, around the start joint's
        // position. When swivel turns around the line from start to end, segment lengths, the start and
        // end positions, and every joint from end onward stay unchanged.
        //
        // A rigid turn also spins each bone around its own length. Constrained joints inside the section
        // need that, because they keep their local rotations. With smallestTurn, which suits sections
        // whose inner joints are free, each bone is instead re-aimed with the smallest rotation, so
        // repeated turns do not build up spin.
        internal void SwivelSection(int start, int end, Quaternion swivel, bool smallestTurn)
        {
            for (int i = start; i < end; i++)
                Joints[i].SolverRotation = GetSwivelledRotation(start, end, swivel, smallestTurn, i);

            Vector3 origin = Joints[start].SolverPosition;
            for (int i = start + 1; i < end; i++)
                Joints[i].SolverPosition = origin + swivel * (Joints[i].SolverPosition - origin);
        }

        // The world rotation SwivelSection gives joint index, from the state before the turn.
        internal Quaternion GetSwivelledRotation(int start, int end, Quaternion swivel, bool smallestTurn, int index)
        {
            Quaternion rotation = Joints[index].SolverRotation;
            if (!smallestTurn)
                return swivel * rotation;

            Vector3 origin = Joints[start].SolverPosition;
            Vector3 position = index == start ? origin : origin + swivel * (Joints[index].SolverPosition - origin);
            Vector3 childPosition = index + 1 == end
                ? Joints[end].SolverPosition
                : origin + swivel * (Joints[index + 1].SolverPosition - origin);
            Vector3 bone = rotation * Joints[index + 1].Segment.GetCurrentLocalOffset();
            return Quaternion.FromToRotation(bone, childPosition - position) * rotation;
        }

        /// <summary>
        /// World rotation of the IK parent frame of joint <paramref name="jointIndex"/>, from solver
        /// state: the parent joint's solver rotation, or <see cref="RootParentRotation"/> for the root.
        /// </summary>
        public Quaternion GetParentSolverRotation(int jointIndex)
        {
            int parentIdx = Joints[jointIndex].ParentIndex;
            return parentIdx >= 0 ? Joints[parentIdx].SolverRotation : RootParentRotation;
        }

        /// <summary>
        /// Rebuilds every solver rotation from its forward and up axes. Iterative solvers call this
        /// after each iteration so floating-point drift from many small rotations does not build up.
        /// </summary>
        public void ReconstructRotations()
        {
            for (int i = 0; i < Joints.Length; i++)
            {
                Quaternion q = Joints[i].SolverRotation;
                Vector3 forward = q * Vector3.forward;
                Vector3 up = q * Vector3.up;

                // LookRotation needs meaningful axes; skip degenerate rotations instead of normalizing noise.
                if (forward.sqrMagnitude > 0.001f && up.sqrMagnitude > 0.001f)
                    Joints[i].SolverRotation = Quaternion.LookRotation(forward, up);
            }
        }

        private void CaptureRootParentRotation()
        {
            RootParentRotation = RootParentTransform != null ? RootParentTransform.rotation : Quaternion.identity;
        }

        /// The transform that acts as the IK parent during setup: the root's transform parent for the
        /// root, the parent joint's transform otherwise.
        public Transform GetSetupParentTransform(int jointIndex)
        {
            int parentIndex = Joints[jointIndex].ParentIndex;
            return parentIndex < 0 ? RootParentTransform : Joints[parentIndex].Transform;
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
