using System.Collections.Generic;
using UnityEngine;

namespace OpenIK
{
    /// <summary>
    /// Jacobian Damped Least Squares (DLS) Solver ― Each iteration assembles a 6xN Jacobian from
    /// every joint's rotational and translational degrees of freedom, then solves a damped linear
    /// system to produce the step that best reduces the end-effector error.
    /// Damping stabilizes the step near singularities at the cost of slower convergence.
    /// </summary>
    /// <remarks>
    /// Supports prismatic (slider) segments as translational DOFs. Can solve for end-effector
    /// position alone or position combined with orientation; see <see cref="OrientationMode"/>.
    /// </remarks>
    public class JacobianIKSolver : OpenIKSolverBase
    {
        [SerializeField] private Transform target;
        [Tooltip("Convergence threshold for the combined position + orientation error.")]
        [SerializeField] private float tolerance = 0.001f;
        [SerializeField] private int maxIterations = 10;
        [SerializeField] private List<Transform> chainJoints = new();

        [Header("Performance")]
        [Tooltip("Enable when the chain, constraints, and runtime settings stay fixed. This skips per-frame constraint refreshes, runtime config application (including joint speed limits), and chain length recomputation.")]
        [SerializeField] private bool staticSolverConfiguration = false;

        [Header("Damped Least Squares")]
        [Tooltip("Damping factor (λ). Higher = more stable near singularities, but slower convergence.")]
        [SerializeField] private float damping = 0.1f;

        [Tooltip("Step size multiplier applied to the joint angle deltas each iteration.")]
        [Range(0.01f, 1f)]
        [SerializeField] private float stepSize = 1f;

        [Header("Orientation")]
        [SerializeField] private OrientationMode orientationMode = OrientationMode.FullRotation;

        [Tooltip("Scales rotational error relative to position error. Higher values make the solver spend more effort matching the selected orientation mode.")]
        [Range(0f, 3f)]
        [SerializeField] private float orientationWeight = 1f;

        private enum OrientationMode
        {
            /// <summary> Ignore the target's rotation entirely. The Jacobian carries only position error. </summary>
            None,

            /// <summary> Aim the final bone to match the full target rotation. </summary>
            FullRotation,

            /// <summary> Aim the final bone along the target's forward-axis (Z-axis) without constraining twist. </summary>
            BoneDirection,
        }

        private struct DOF
        {
            public int jointIndex;
            public IJacobianDofProvider provider;
            public int providerDofIndex;
            public SolverDofType type;
            public Vector3 axis;
        }

        private readonly SolverChain _chain = new();

        protected override SolverChain Chain => _chain;
        protected override Transform SolveTarget => target;
        private SolverJoint[] _joints;
        private int _jointCount;
        private DOF[] _dofs;
        private int _dofCount;
        private Matrix6xN _jacobian;
        private float[] _deltaTheta;

        private void Awake()
        {
            if (!SolverSetupValidation.Validate(this, "Jacobian Solver", target, chainJoints))
                return;

            _chain.Initialize(chainJoints);
            SyncCachedChainState();
            // Seed solver state from transforms, then pull prismatic positions into runtime segment state
            // before the first solve.
            _chain.SyncSolverStateFromTransforms();
            BuildDOFs();
        }

        private void SyncCachedChainState()
        {
            _joints = _chain.Joints;
            _jointCount = _joints.Length;
        }

        private bool RefreshConstraintBindings()
        {
            return _chain.RefreshConstraintBindings();
        }

        private void ApplyRuntimeConfigs()
        {
            _chain.ApplyRuntimeConfigs();
        }

        private void BuildDOFs()
        {
            var dofList = new List<DOF>();
            int endIdx = _jointCount - 1;

            for (int i = 0; i < _jointCount; i++)
            {
                AddProviderDOFs(dofList, i, _joints[i].Segment);

                if (i != endIdx)
                    AddProviderDOFs(dofList, i, _joints[i].Angular);
            }

            _dofs = dofList.ToArray();
            _dofCount = _dofs.Length;
            _jacobian = new Matrix6xN(_dofCount);
            _deltaTheta = new float[_dofCount];
        }

        private static void AddProviderDOFs(List<DOF> dofs, int jointIndex, IJacobianDofProvider provider)
        {
            for (int i = 0; i < provider.DofCount; i++)
            {
                dofs.Add(new DOF
                {
                    jointIndex = jointIndex,
                    provider = provider,
                    providerDofIndex = i,
                    type = provider.GetDofType(i)
                });
            }
        }

        private void LateUpdate()
        {
            if (_jointCount < 2 || target == null) return;

            bool bindingsChanged = false;
            if (!staticSolverConfiguration)
            {
                _chain.UpdateConstraints();
                bindingsChanged = RefreshConstraintBindings();
                ApplyRuntimeConfigs();
                _chain.ComputeChainLength();
            }

            // Seed solver state from current transforms (any external transform edits since the last
            // solve are picked up here), then derive slider segment state from those solver positions.
            // While speed-limited joints lag behind the previous solution, the solve continues from it.
            SyncSolverStateForSolve(solveFromRestPose: false);
            if (bindingsChanged)
                BuildDOFs();
            Solve();
        }

        private void Solve()
        {
            int endIdx = _jointCount - 1;
            int boneIdx = endIdx - 1; // last visible bone (the one we orient)
            bool useOri = orientationMode != OrientationMode.None;

            int iter = 0;
            float totalError = ComputeError(endIdx, boneIdx, out Vector3 posError, out Vector3 oriError);
            for (; iter < maxIterations; iter++)
            {
                if (totalError < tolerance)
                    break;

                Vector3 endEffectorPos = _joints[endIdx].SolverPosition;
                UpdateDOFAxes();

                // Build the 6×N Jacobian
                _jacobian.Clear();
                for (int d = 0; d < _dofCount; d++)
                {
                    int ji = _dofs[d].jointIndex;
                    Vector3 dofAxis = _dofs[d].axis;
                    if (_dofs[d].type == SolverDofType.Translation)
                    {
                        _jacobian[d] = new float6(
                            dofAxis.x, dofAxis.y, dofAxis.z,
                            0f, 0f, 0f
                        );
                    }
                    else
                    {
                        Vector3 jointToEnd = endEffectorPos - _joints[ji].SolverPosition;
                        Vector3 posCol = Vector3.Cross(dofAxis, jointToEnd);
                        _jacobian[d] = new float6(
                            posCol.x, posCol.y, posCol.z,
                            useOri && ji <= boneIdx ? dofAxis.x * orientationWeight : 0f,
                            useOri && ji <= boneIdx ? dofAxis.y * orientationWeight : 0f,
                            useOri && ji <= boneIdx ? dofAxis.z * orientationWeight : 0f
                        );
                    }
                }

                float6 e = new float6(
                    posError.x, posError.y, posError.z,
                    oriError.x, oriError.y, oriError.z
                );

                // Δθ = Jᵀ (J Jᵀ + λ²I)⁻¹ e
                Matrix6x6 A = _jacobian.MultiplyJJt() + Matrix6x6.identity * (damping * damping);
                A.CholeskySolve(e, out float6 w);
                _jacobian.MultiplyJtVec(w, _deltaTheta);

                for (int d = 0; d < _dofCount; d++)
                {
                    float delta = _deltaTheta[d] * stepSize;
                    int jointIndex = _dofs[d].jointIndex;
                    SolverJoint joint = _joints[jointIndex];
                    SolverJoint parent = joint.ParentIndex >= 0 ? _joints[joint.ParentIndex] : null;

                    bool isRotation = _dofs[d].type == SolverDofType.Rotation;
                    Quaternion preRotation = isRotation ? joint.SolverRotation : Quaternion.identity;

                    _dofs[d].provider.ApplyDofDelta(
                        _dofs[d].providerDofIndex,
                        delta,
                        new IJacobianDofProvider.Context(joint, parent));

                    if (isRotation && jointIndex < _jointCount - 1)
                    {
                        Quaternion appliedDelta = joint.SolverRotation * Quaternion.Inverse(preRotation);
                        PropagateRotationDeltaDownstream(jointIndex, appliedDelta);
                    }
                }

                // Rebuild every joint's world position from its parent's now-current rotation +
                // segment offset. Rotations were already inherited inside the per-DOF loop above.
                RebuildDownstreamPositions();

                ApplyConstraints();
                RebuildDownstreamPositions();

                // Refresh the error so a final non-converged iteration reports its post-step error.
                totalError = ComputeError(endIdx, boneIdx, out posError, out oriError);
            }

            _output.Populate(_chain, iter, totalError, totalError < tolerance, writeEndRotation: true);

            ApplyAndRaiseSolved();
        }

        /// <summary>
        /// Computes position and orientation error from the current solver state against
        /// <see cref="target"/>. Returns (posError.magnitude + oriError.magnitude) as a single scalar.
        /// </summary>
        /// <remarks>
        /// The position error magnitude is in meters; the orientation error magnitude is in radians,
        /// pre-scaled by <see cref="orientationWeight"/>. The two are summed directly, so the return
        /// value is a mixed-units scalar rather than a pure distance. The same scalar is compared
        /// against <see cref="tolerance"/> for convergence — tuning <see cref="orientationWeight"/>
        /// biases the convergence test toward position (lower) or orientation (higher). When
        /// <see cref="orientationMode"/> is <see cref="OrientationMode.None"/>, <paramref name="oriError"/>
        /// is zero and the scalar reduces to pure position error in meters.
        /// </remarks>
        private float ComputeError(int endIdx, int boneIdx, out Vector3 posError, out Vector3 oriError)
        {
            Vector3 endEffectorPos = _joints[endIdx].SolverPosition;
            posError = target.position - endEffectorPos;
            oriError = Vector3.zero;

            if (orientationMode == OrientationMode.FullRotation)
            {
                Quaternion oriDelta = target.rotation * Quaternion.Inverse(_joints[boneIdx].SolverRotation);
                if (oriDelta.w < 0f)
                {
                    oriDelta.x = -oriDelta.x;
                    oriDelta.y = -oriDelta.y;
                    oriDelta.z = -oriDelta.z;
                    oriDelta.w = -oriDelta.w;
                }
                oriDelta.ToAngleAxis(out float angle, out Vector3 axis);
                if (angle > 180f) angle -= 360f;
                oriError = axis.normalized * (angle * Mathf.Deg2Rad) * orientationWeight;
            }
            else if (orientationMode == OrientationMode.BoneDirection)
            {
                Vector3 boneDir = (endEffectorPos - _joints[boneIdx].SolverPosition).normalized;
                Vector3 desiredDir = target.forward;
                // Cross product gives axis and sin(angle), and a smooth error signal for the Jacobian to drive.
                oriError = Vector3.Cross(boneDir, desiredDir) * orientationWeight;
            }

            return posError.magnitude + oriError.magnitude;
        }

        /// <summary>
        /// Reads the actual orientation error from the transforms, matching <see cref="ComputeError"/>'s
        /// orientation objective: the last bone's rotation for <see cref="OrientationMode.FullRotation"/>,
        /// its direction for <see cref="OrientationMode.BoneDirection"/>.
        /// </summary>
        protected override float ComputeActualOrientationError()
        {
            if (_jointCount < 2 || target == null)
                return float.NaN;

            Transform end = _joints[_jointCount - 1].Transform;
            Transform bone = _joints[_jointCount - 2].Transform;
            switch (orientationMode)
            {
                case OrientationMode.FullRotation:
                    return Quaternion.Angle(target.rotation, bone.rotation);
                case OrientationMode.BoneDirection:
                    return Vector3.Angle(end.position - bone.position, target.forward);
                default:
                    return float.NaN;
            }
        }

        /// <summary>
        /// Walks the chain from root-to-end and recomputes each joint's world solver position from
        /// its parent's solver position/rotation and the segment's current local offset.
        /// </summary>
        private void RebuildDownstreamPositions()
        {
            for (int i = 1; i < _jointCount; i++)
            {
                int parentIdx = _joints[i].ParentIndex;
                _joints[i].SolverPosition = _joints[parentIdx].SolverPosition
                    + _joints[parentIdx].SolverRotation * _joints[i].Segment.GetCurrentLocalOffset();
            }
        }

        private void UpdateDOFAxes()
        {
            for (int d = 0; d < _dofCount; d++)
            {
                int jointIndex = _dofs[d].jointIndex;
                SolverJoint parent = _joints[jointIndex].ParentIndex >= 0
                    ? _joints[_joints[jointIndex].ParentIndex]
                    : null;
                _dofs[d].axis = _dofs[d].provider.GetDofAxisWorld(
                    _dofs[d].providerDofIndex,
                    new IJacobianDofProvider.Context(_joints[jointIndex], parent));
            }
        }

        private void ApplyConstraints()
        {
            for (int i = 0; i < _jointCount; i++)
            {
                IAngularConstraint angular = _joints[i].Angular;

                // Compute deviation in constraint frame.
                Quaternion parentRotWorld = GetParentSolverRotation(i, out bool hasParent);
                if (!hasParent) continue;

                Quaternion preRotation = _joints[i].SolverRotation;
                Quaternion constraintDeviation = _joints[i].ToConstraintDeviation(parentRotWorld, preRotation);

                Quaternion clamped = angular.ClampDeviation(constraintDeviation);

                Quaternion postRotation = _joints[i].FromConstraintDeviation(parentRotWorld, clamped);
                _joints[i].SolverRotation = postRotation;

                // Inherit any clamp-induced rotation delta onto descendants so their world rotations
                // (and the deviations computed for them later in this same loop) stay consistent.
                if (i < _jointCount - 1 && postRotation != preRotation)
                {
                    Quaternion appliedDelta = postRotation * Quaternion.Inverse(preRotation);
                    PropagateRotationDeltaDownstream(i, appliedDelta);
                }
            }
        }

        /// <summary>
        /// Applies a world-space rotation delta to every joint downstream of <paramref name="jointIndex"/>.
        /// Position propagation is handled separately by <see cref="RebuildDownstreamPositions"/>.
        /// </summary>
        private void PropagateRotationDeltaDownstream(int jointIndex, Quaternion delta)
        {
            for (int i = jointIndex + 1; i < _jointCount; i++)
                _joints[i].SolverRotation = delta * _joints[i].SolverRotation;
        }

        /// <summary>
        /// Gets the world rotation of the IK parent frame for a joint, reading from solver state.
        /// </summary>
        /// <param name="jointIndex">Index of the joint whose parent rotation is requested.</param>
        /// <param name="hasParent">
        /// True when a parent frame exists for the joint; false when the root joint has no IK parent
        /// (the caller should skip clamping in that case to preserve prior behavior).
        /// </param>
        private Quaternion GetParentSolverRotation(int jointIndex, out bool hasParent)
        {
            int parentIdx = _joints[jointIndex].ParentIndex;
            if (parentIdx >= 0)
            {
                hasParent = true;
                return _joints[parentIdx].SolverRotation;
            }

            Transform rootParent = _chain.RootParentTransform;
            if (rootParent != null)
            {
                hasParent = true;
                return rootParent.rotation;
            }

            hasParent = false;
            return Quaternion.identity;
        }

        [Header("Gizmos")]
        [SerializeField] private GizmoDrawMode boneGizmoMode = GizmoDrawMode.SelectedOnly;
        /// <summary> Gizmo rendering mode for joint bones in scene view. </summary>
        public GizmoDrawMode BoneGizmoMode => boneGizmoMode;

        /// <summary> Joint transforms resolved by the solver from first to end effector. </summary>
        public IReadOnlyList<Transform> ChainJoints => chainJoints;
    }
}
