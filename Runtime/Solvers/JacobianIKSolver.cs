using System.Collections.Generic;
using UnityEngine;

namespace OpenIK
{
    /// <summary>
    /// Jacobian Damped Least Squares (DLS) solver. Moves the end effector to the target position,
    /// optionally matching the target's orientation as well.
    /// </summary>
    /// <remarks>
    /// Each iteration builds a 6xN Jacobian from every joint's rotational and translational degrees
    /// of freedom, then solves a damped linear system for the step that best reduces the end-effector
    /// error. Damping keeps the step stable near singularities at the cost of slower convergence.
    /// Slider segments are solved as translational degrees of freedom.
    /// </remarks>
    public class JacobianIKSolver : OpenIKSolverBase
    {
        [Header("Damped Least Squares")]
        [Tooltip("Stabilizes solver steps near singularities. Higher values can slow convergence.")]
        [SerializeField] private float damping = 0.1f;

        [Tooltip("Scales each iteration's joint rotation and slider movement.")]
        [Range(0.01f, 1f)]
        [SerializeField] private float stepSize = 1f;

        [Header("Orientation")]
        [SerializeField] private OrientationMode orientationMode = OrientationMode.FullRotation;

        [Tooltip("Controls the importance of matching orientation relative to position. Higher values favour orientation and also affect the tolerance check. " +
                 "Zero removes the orientation objective; unused when Orientation Mode is None.")]
        [Range(0f, 3f)]
        [SerializeField] private float orientationWeight = 1f;

        private enum OrientationMode
        {
            // Ignore the target's rotation. The Jacobian carries only position error.
            None,

            // Match the final bone's rotation to the full target rotation.
            FullRotation,

            // Aim the final bone along the target's forward axis (+Z) without constraining twist.
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

        private SolverJoint[] _joints;
        private int _jointCount;
        private DOF[] _dofs;
        private int _dofCount;
        private Matrix6xN _jacobian;
        private float[] _deltaTheta;

        protected override string SolverName => "Jacobian Solver";

        protected override bool WritesEndRotation => true;

        protected override void OnInitialized()
        {
            _joints = Chain.Joints;
            _jointCount = _joints.Length;
            // Pull slider positions into the runtime segment state before the DOFs are built.
            Chain.SyncSolverStateFromTransforms();
            BuildDOFs();
        }

        protected override void OnConstraintsRebound()
        {
            BuildDOFs();
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

        protected override SolveResult SolveChain(in IKGoal goal)
        {
            int endIdx = _jointCount - 1;
            int boneIdx = endIdx - 1; // last visible bone (the one we orient)
            bool useOri = orientationMode != OrientationMode.None;

            int iter = 0;
            float totalError = ComputeError(goal, endIdx, boneIdx, out Vector3 posError, out Vector3 oriError);
            for (; iter < MaxIterations; iter++)
            {
                if (totalError < Tolerance)
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
                totalError = ComputeError(goal, endIdx, boneIdx, out posError, out oriError);
            }

            return new SolveResult(iter, totalError);
        }

        // Position and orientation error of the solver state against the goal, summed into one
        // scalar. Position error is in metres; orientation error is in radians, scaled by
        // orientationWeight. The sum is what the solver compares against Tolerance, so a higher
        // orientation weight biases convergence toward orientation. With OrientationMode.None the
        // orientation error is zero and the scalar is a plain distance.
        private float ComputeError(in IKGoal goal, int endIdx, int boneIdx, out Vector3 posError, out Vector3 oriError)
        {
            Vector3 endEffectorPos = _joints[endIdx].SolverPosition;
            posError = goal.Position - endEffectorPos;
            oriError = Vector3.zero;

            if (orientationMode == OrientationMode.FullRotation)
            {
                Quaternion oriDelta = goal.Rotation * Quaternion.Inverse(_joints[boneIdx].SolverRotation);
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
                Vector3 desiredDir = goal.Forward;
                // Cross product gives axis and sin(angle), and a smooth error signal for the Jacobian to drive.
                oriError = Vector3.Cross(boneDir, desiredDir) * orientationWeight;
            }

            return posError.magnitude + oriError.magnitude;
        }

        // Reads the actual orientation error from the transforms, matching ComputeError's objective:
        // the last bone's rotation for FullRotation, its direction for BoneDirection.
        protected override float ComputeActualOrientationError()
        {
            Transform target = Target;
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

        // Recomputes each joint's solver position, root to end, from its parent's solver pose and the
        // segment's current local offset.
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

                // A root without a parent transform has no frame to clamp against.
                if (i == 0 && Chain.RootParentTransform == null)
                    continue;

                Quaternion parentRotWorld = Chain.GetParentSolverRotation(i);
                Quaternion preRotation = _joints[i].SolverRotation;
                Quaternion constraintDeviation = _joints[i].ToConstraintDeviation(parentRotWorld, preRotation);

                Quaternion clamped = angular.ClampDeviation(constraintDeviation);

                // Clamping converts the rotation into the joint's limit space and back with
                // Quaternion.Inverse, which only works on unit-length quaternions, so normalize.
                Quaternion postRotation = Quaternion.Normalize(_joints[i].FromConstraintDeviation(parentRotWorld, clamped));
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

        // Applies a world-space rotation delta to every joint below jointIndex. Positions are rebuilt
        // separately by RebuildDownstreamPositions.
        private void PropagateRotationDeltaDownstream(int jointIndex, Quaternion delta)
        {
            for (int i = jointIndex + 1; i < _jointCount; i++)
                _joints[i].SolverRotation = delta * _joints[i].SolverRotation;
        }
    }
}
