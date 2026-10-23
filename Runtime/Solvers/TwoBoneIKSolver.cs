using UnityEngine;

namespace OpenIK
{
    /// <summary>Selects how the TwoBoneIKSolver turns the end joint.</summary>
    public enum TwoBoneEndRotation
    {
        /// <summary>The end joint keeps its rotation relative to the lower bone.</summary>
        None,

        /// <summary>The end joint takes the target's rotation, keeping the offset it had to the target at initialization.</summary>
        MatchTarget,

        /// <summary>The end joint turns so its local up axis matches the target's up axis.</summary>
        AlignUp
    }

    /// <summary>Solves a two-bone limb, such as an arm or leg, directly with the law of cosines.</summary>
    /// <remarks>
    /// Joints holds exactly three joints: upper (shoulder or hip), middle (elbow or knee), and end
    /// (wrist or ankle). A pole on the middle joint sets the bend side. Without one, a bent limb keeps
    /// its current bend side and a nearly straight limb keeps the rest pose's. A hinge on the middle
    /// joint, or on the upper joint with a free middle joint, is solved exactly within its limits. With a
    /// hinge on the upper joint, the pole only picks between the two bend sides the hinge allows. Other
    /// joint limits are applied after the solve and can keep the end joint from reaching its target. The
    /// solver does not solve slider translation; slider segments keep their current length.
    /// </remarks>
    [AddComponentMenu("OpenIK/Two Bone Solver", 3)]
    public class TwoBoneIKSolver : SingleChainSolverBase
    {
        // Squared lengths below this count as zero.
        private const float MinLengthSqr = 1e-10f;

        // Below this squared fraction of their full length, a unit vector projected onto a plane, or a
        // cross product of two bones, counts as zero: the vectors are parallel.
        private const float ParallelSqr = 1e-8f;
        
        // The rest pose's middle joint must sit at least this squared fraction of the upper bone's length
        // off the reach line for the rest pose to have a bend side.
        private const float MinRestBendSqr = 1e-6f;

        // How far the middle joint sits off the reach line, as a fraction of the upper bone's length,
        // before the current bend fully decides the bend side. Closer to straight, the rest pose's bend
        // side takes over, so a straight or overextended limb does not flip.
        private const float BendBlendRange = 0.15f;

        // Two hinge angles whose clamps differ by less than this many degrees count as equally limited.
        private const float AngleTie = 1e-3f;

        [Space(6)]
        [Tooltip("How the end joint turns.\n" +
                 "• None: keeps its rotation relative to the lower bone.\n" +
                 "• Match Target: follows the target's rotation with the captured offset.\n" +
                 "• Align Up: aligns the end joint's local up axis with the target's up axis.")]
        [SerializeField] private TwoBoneEndRotation endRotation = TwoBoneEndRotation.None;

        [Tooltip("Eases the last part of the reach so the limb straightens smoothly instead of snapping. The easing starts this " +
                 "fraction of the limb's length before full extension. The end joint then stops slightly short of distant targets. " +
                 "At 0 it reaches targets exactly.")]
        [Range(0f, 0.5f)]
        [SerializeField] private float softness;

        // The end joint's rotation relative to the target, captured at initialization.
        private Quaternion _endRotationOffset = Quaternion.identity;

        // The rest pose's bend direction in the upper joint's frame; zero for a straight rest pose.
        private Vector3 _bendLocal;

        /// <summary>How the end joint turns.</summary>
        public TwoBoneEndRotation EndRotation
        {
            get => endRotation;
            set => endRotation = value;
        }

        /// <summary>Eases the last part of the reach so the limb straightens smoothly instead of snapping.</summary>
        /// <remarks>
        /// The easing starts this fraction of the limb's length before full extension, from 0 (off) to 0.5.
        /// The end joint then stops short of distant targets. Use 0 when it must reach its target exactly.
        /// </remarks>
        public float Softness
        {
            get => softness;
            set => softness = Mathf.Clamp(value, 0f, 0.5f);
        }

        /// <summary>
        /// The end joint's rotation relative to the target, captured at initialization. With Match Target,
        /// the end joint turns to the target's rotation times this offset.
        /// </summary>
        public Quaternion EndRotationOffset => _endRotationOffset;

        protected override string SolverName => "Two Bone Solver";

        protected override int RequiredJointCount => 3;

        protected override bool SupportsSliderTranslation => false;

        protected override bool WritesEndRotation => endRotation != TwoBoneEndRotation.None;

        // SolveChain places the bend toward the pole directly.
        protected override bool AppliesPolesBeforeSolve => false;

        protected override void OnInitialized()
        {
            Quaternion endJointRotation = Chain.Joints[2].Transform.rotation;
            _endRotationOffset = Quaternion.Inverse(Target.rotation) * endJointRotation;
            _bendLocal = GetRestBendLocal();
        }

        // The rest pose's bend direction in the upper joint's frame. It counts only when the middle
        // joint visibly leaves the reach line; a straight rest pose gives zero.
        private Vector3 GetRestBendLocal()
        {
            SolverJoint[] joints = Chain.Joints;
            Vector3 middle = joints[1].RestLocalOffset;
            Vector3 end = middle + joints[1].RestLocalRotation * joints[2].RestLocalOffset;
            if (end.sqrMagnitude < MinLengthSqr)
                return Vector3.zero;

            Vector3 restBend = Vector3.ProjectOnPlane(middle, end.normalized);
            return restBend.sqrMagnitude > MinRestBendSqr * middle.sqrMagnitude ? restBend.normalized : Vector3.zero;
        }

        protected override SolveResult SolveChain(in IKGoal goal)
        {
            SolverJoint[] joints = Chain.Joints;
            Vector3 root = joints[0].SolverPosition;
            float upperLength = joints[1].Segment.GetCurrentLocalOffset().magnitude;
            float lowerLength = joints[2].Segment.GetCurrentLocalOffset().magnitude;

            Vector3 toTarget = goal.Position - root;
            float distance = toTarget.magnitude;
            if (distance * distance < MinLengthSqr || upperLength * lowerLength <= 0f)
                return new SolveResult(1, Vector3.Distance(joints[2].SolverPosition, goal.Position));

            Vector3 reachDirection = toTarget / distance;
            float maxReach = upperLength + lowerLength;
            // The lower bound covers targets closer to the root than the folded limb can reach.
            float reach = Mathf.Clamp(SoftenReach(distance, maxReach), Mathf.Abs(upperLength - lowerLength), maxReach);

            // Law of cosines: the angle at the upper joint between the reach line and the upper bone.
            float cos = Mathf.Clamp((upperLength * upperLength + reach * reach - lowerLength * lowerLength) / (2f * upperLength * reach), -1f, 1f);
            float sin = Mathf.Sqrt(1f - cos * cos);

            Vector3 bend = GetBendDirection(root, reachDirection, upperLength);
            Vector3 middleTarget = root + upperLength * (cos * reachDirection + sin * bend);
            Vector3 endTarget = root + reachDirection * reach;

            bool solvedHinge = joints[1].Angular is HingeAngularConstraint middleHinge
                ? TrySolveMiddleHinge(middleHinge, root, endTarget, reach, bend)
                : Chain.RootParentTransform != null && joints[0].Angular is HingeAngularConstraint upperHinge
                  && TrySolveUpperHinge(upperHinge, root, endTarget, bend);
            if (!solvedHinge)
                TurnUpperAndMiddle(root, middleTarget, endTarget, bend);
            ClampUpperAndMiddle();
            TurnEnd(goal);

            return new SolveResult(1, Vector3.Distance(joints[2].SolverPosition, goal.Position));
        }

        protected override float ComputeActualOrientationError()
        {
            Transform target = Target;
            if (target == null || Chain.Count != 3)
                return float.NaN;

            Quaternion endJointRotation = Chain.Joints[2].Transform.rotation;
            return endRotation switch
            {
                TwoBoneEndRotation.MatchTarget => Quaternion.Angle(endJointRotation, target.rotation * _endRotationOffset),
                TwoBoneEndRotation.AlignUp => Vector3.Angle(endJointRotation * Vector3.up, target.up),
                _ => float.NaN
            };
        }

        // Soft IK: past the hard reach, the reach approaches full extension exponentially, with a
        // continuous slope, so the middle joint straightens smoothly instead of snapping.
        private float SoftenReach(float distance, float maxReach)
        {
            if (softness <= 0f)
                return distance;

            float softLength = softness * maxReach;
            float hardReach = maxReach - softLength;
            if (distance <= hardReach)
                return distance;

            return hardReach + softLength * (1f - Mathf.Exp(-(distance - hardReach) / softLength));
        }

        // The side the middle joint bends toward, perpendicular to the reach line. A pole on the middle
        // joint decides it. Without one, a clearly bent limb keeps its current bend side. Near straight
        // that bend is too small to trust and flips when the limb overextends, so the rest pose's bend
        // side, carried by the upper joint, takes over. With no rest bend, the limb uses the middle
        // hinge's bending side, then any perpendicular direction.
        private Vector3 GetBendDirection(Vector3 root, Vector3 reachDirection, float upperLength)
        {
            if (TryGetPoleDirection(1, root, reachDirection, out Vector3 poleDirection))
                return poleDirection;

            SolverJoint[] joints = Chain.Joints;
            Vector3 current = Vector3.ProjectOnPlane(joints[1].SolverPosition - root, reachDirection);
            Vector3 stored = Vector3.ProjectOnPlane(joints[0].SolverRotation * _bendLocal, reachDirection);
            if (stored.sqrMagnitude > ParallelSqr)
            {
                stored.Normalize();
                float offset = Vector3.Dot(current, stored) / upperLength;
                float weight = Mathf.SmoothStep(0f, 1f, offset / BendBlendRange);
                return weight > 0f ? Vector3.Lerp(stored, current.normalized, weight).normalized : stored;
            }

            if (current.sqrMagnitude > MinLengthSqr)
                return current.normalized;

            Vector3 bend;

            // A positive hinge angle folds the lower bone toward cross(axis, reach), which moves the
            // middle joint the opposite way.
            if (joints[1].Angular is HingeAngularConstraint hinge)
            {
                bend = Vector3.Cross(reachDirection, joints[1].SolverRotation * hinge.HingeAxis);
                if (joints[1].ConstrainedJoint is HingeIKJoint limits && limits.hingeMaxAngle < -limits.hingeMinAngle)
                    bend = -bend;
                if (bend.sqrMagnitude > MinLengthSqr)
                    return bend.normalized;
            }

            Vector3 reference = Mathf.Abs(reachDirection.y) < 0.99f ? Vector3.up : Vector3.right;
            return Vector3.Cross(reachDirection, reference).normalized;
        }

        // Turns the upper and middle joints so the bones run through the target positions. The upper
        // joint's turn maps the old bend plane onto the new one, so a hinge on the middle joint keeps
        // its axis and the middle joint only bends around it.
        private void TurnUpperAndMiddle(Vector3 root, Vector3 middleTarget, Vector3 endTarget, Vector3 bend)
        {
            SolverJoint[] joints = Chain.Joints;
            Vector3 oldUpper = joints[1].SolverPosition - root;
            Vector3 oldLower = joints[2].SolverPosition - joints[1].SolverPosition;
            Vector3 newUpper = middleTarget - root;
            Vector3 newLower = endTarget - middleTarget;
            Vector3 oldNormal = Vector3.Cross(oldUpper, oldLower);
            Vector3 newNormal = Vector3.Cross(newUpper, newLower);

            // An overextended limb puts its middle joint just past the reach line, opposite its bend side.
            // Treat that as a small negative bend in the same plane. Read as a plane turned half a turn, it
            // would twist the upper joint by 180 degrees as the limb bends back.
            Vector3 bendSide = _bendLocal.sqrMagnitude > 0f ? joints[0].SolverRotation * _bendLocal : bend;
            if (Vector3.Dot(Vector3.ProjectOnPlane(oldUpper, oldUpper + oldLower), bendSide) < 0f)
                oldNormal = -oldNormal;

            float scale = oldUpper.sqrMagnitude * oldLower.sqrMagnitude;
            Quaternion upperTurn = oldNormal.sqrMagnitude > ParallelSqr * scale && newNormal.sqrMagnitude > ParallelSqr * scale
                ? Quaternion.LookRotation(newUpper, newNormal) * Quaternion.Inverse(Quaternion.LookRotation(oldUpper, oldNormal))
                : Quaternion.FromToRotation(oldUpper, newUpper);

            TurnJoint(0, upperTurn);
            AimMiddleAt(endTarget);
        }

        // A hinge on the middle joint can only bend around its axis. Solving the bend in the bend plane
        // and clamping afterwards would swing the end joint off the target whenever the hinge axis is not
        // that plane's normal, or the starting pose rotates the middle joint off its hinge. Instead, the
        // hinge angle that gives the reach is solved directly, then the upper joint aims the limb at the
        // target and swivels it around the reach line toward the bend side.
        private bool TrySolveMiddleHinge(HingeAngularConstraint hinge, Vector3 root, Vector3 endTarget, float reach, Vector3 bend)
        {
            SolverJoint[] joints = Chain.Joints;
            Quaternion zeroRotation = joints[0].SolverRotation * joints[1].RestLocalRotation;
            Vector3 axis = zeroRotation * hinge.HingeAxis;
            Vector3 upper = joints[1].SolverPosition - root;
            Vector3 lowerAtZero = zeroRotation * (Quaternion.Inverse(joints[1].SolverRotation) * (joints[2].SolverPosition - joints[1].SolverPosition));
            if (!TrySolveHingeAngles(upper, lowerAtZero, axis, reach, out float straight, out float offset))
                return false;

            // The two angles bend to either side of straight. Like the bend plane solve, keep the rest
            // pose's side, or the current side when the rest pose is straight, so the limb does not flip.
            float current = HingeIKJoint.ExtractHingeAngle(Quaternion.Inverse(zeroRotation) * joints[1].SolverRotation, hinge.HingeAxis);
            float side = Mathf.DeltaAngle(straight, _bendLocal.sqrMagnitude > 0f ? 0f : current);
            float angle = PickHingeAngle(hinge, straight - offset, straight + offset, -side, side);

            TurnJoint(1, zeroRotation * Quaternion.AngleAxis(angle, hinge.HingeAxis) * Quaternion.Inverse(joints[1].SolverRotation));
            TurnJoint(0, Quaternion.FromToRotation(joints[2].SolverPosition - root, endTarget - root));

            Vector3 reachDirection = (endTarget - root).normalized;
            // A straight limb has no bend side to swivel.
            Vector3 middleSide = Vector3.ProjectOnPlane(joints[1].SolverPosition - root, reachDirection);
            if (middleSide.sqrMagnitude > ParallelSqr * upper.sqrMagnitude)
                TurnJoint(0, Quaternion.AngleAxis(Vector3.SignedAngle(middleSide, bend, reachDirection), reachDirection));
            return true;
        }

        // A hinge on the upper joint keeps the middle joint on a circle around the hinge axis. The upper
        // joint turns to the point on that circle one lower-bone length from the target, the one on the bend
        // side when there are two, and the middle joint then aims the end joint at the target. The bend side
        // only picks between those two points; the hinge leaves no other choice.
        private bool TrySolveUpperHinge(HingeAngularConstraint hinge, Vector3 root, Vector3 endTarget, Vector3 bend)
        {
            SolverJoint[] joints = Chain.Joints;
            Quaternion zeroRotation = Chain.RootParentRotation * joints[0].RestLocalRotation;
            Vector3 axis = zeroRotation * hinge.HingeAxis;
            Vector3 upperAtZero = zeroRotation * (Quaternion.Inverse(joints[0].SolverRotation) * (joints[1].SolverPosition - root));
            float lowerLength = Vector3.Distance(joints[2].SolverPosition, joints[1].SolverPosition);
            if (!TrySolveHingeAngles(root - endTarget, upperAtZero, axis, lowerLength, out float straight, out float offset))
                return false;

            float first = straight - offset;
            float second = straight + offset;
            float angle = PickHingeAngle(hinge, first, second,
                Vector3.Dot(Quaternion.AngleAxis(first, axis) * upperAtZero, bend),
                Vector3.Dot(Quaternion.AngleAxis(second, axis) * upperAtZero, bend));

            TurnJoint(0, zeroRotation * Quaternion.AngleAxis(angle, hinge.HingeAxis) * Quaternion.Inverse(joints[0].SolverRotation));
            AimMiddleAt(endTarget);
            return true;
        }

        // Turning q around axis by straight ± offset degrees makes |p + q| equal length. At straight, |p + q|
        // is longest. Out of range, the angles give the nearest length possible. False when p or q runs
        // along the axis, so turning q cannot change the length.
        private static bool TrySolveHingeAngles(Vector3 p, Vector3 q, Vector3 axis, float length, out float straight, out float offset)
        {
            straight = offset = 0f;
            Vector3 pAcross = Vector3.ProjectOnPlane(p, axis);
            Vector3 qAcross = Vector3.ProjectOnPlane(q, axis);
            if (pAcross.sqrMagnitude * qAcross.sqrMagnitude <= ParallelSqr * p.sqrMagnitude * q.sqrMagnitude)
                return false;

            // |p + q|² = p² + q² + 2(p·q along the axis) + 2|pAcross||qAcross|cos(angle between them).
            float along = Vector3.Dot(p, axis) * Vector3.Dot(q, axis);
            float cos = (length * length - p.sqrMagnitude - q.sqrMagnitude - 2f * along) / (2f * pAcross.magnitude * qAcross.magnitude);
            offset = Mathf.Acos(Mathf.Clamp(cos, -1f, 1f)) * Mathf.Rad2Deg;
            straight = Vector3.SignedAngle(qAcross, pAcross, axis);
            return true;
        }

        // Clamps both angles to the hinge limits and keeps the one the limits change least. On a tie, such
        // as both inside the limits, it keeps the one with the higher preference.
        private static float PickHingeAngle(HingeAngularConstraint hinge, float first, float second, float firstPreference, float secondPreference)
        {
            first = Mathf.DeltaAngle(0f, first);
            second = Mathf.DeltaAngle(0f, second);
            float clampedFirst = Mathf.Clamp(first, hinge.MinAngle, hinge.MaxAngle);
            float clampedSecond = Mathf.Clamp(second, hinge.MinAngle, hinge.MaxAngle);
            float firstChange = Mathf.Abs(clampedFirst - first);
            float secondChange = Mathf.Abs(clampedSecond - second);
            bool pickFirst = Mathf.Abs(firstChange - secondChange) > AngleTie ? firstChange < secondChange : firstPreference >= secondPreference;
            return pickFirst ? clampedFirst : clampedSecond;
        }

        // Turns the middle joint so the lower bone points at endTarget.
        private void AimMiddleAt(Vector3 endTarget)
        {
            SolverJoint[] joints = Chain.Joints;
            TurnJoint(1, Quaternion.FromToRotation(
                joints[2].SolverPosition - joints[1].SolverPosition,
                endTarget - joints[1].SolverPosition));
        }

        private void TurnJoint(int index, Quaternion turn)
        {
            SolverJoint joint = Chain.Joints[index];
            joint.SolverRotation = Quaternion.Normalize(turn * joint.SolverRotation);
            Chain.RotateAndRepositionDownstream(index, turn);
        }

        // Clamps the upper and middle joints to their constraints. A root without a parent transform
        // has no frame to clamp against.
        private void ClampUpperAndMiddle()
        {
            SolverJoint[] joints = Chain.Joints;
            for (int i = 0; i < 2; i++)
            {
                if (i == 0 && Chain.RootParentTransform == null)
                    continue;

                Quaternion desired = joints[i].SolverRotation;
                Quaternion clamped = joints[i].ClampRotation(Chain.GetParentSolverRotation(i), desired, desired);
                if (clamped == desired)
                    continue;

                clamped = Quaternion.Normalize(clamped);
                joints[i].SolverRotation = clamped;
                Chain.RotateAndRepositionDownstream(i, clamped * Quaternion.Inverse(desired));
            }
        }

        // Turns the end joint as End Rotation selects, within the end joint's limits.
        private void TurnEnd(in IKGoal goal)
        {
            SolverJoint end = Chain.Joints[2];
            Quaternion desired;
            switch (endRotation)
            {
                case TwoBoneEndRotation.MatchTarget:
                    desired = goal.Rotation * _endRotationOffset;
                    break;
                case TwoBoneEndRotation.AlignUp:
                    Vector3 up = end.SolverRotation * Vector3.up;
                    desired = Quaternion.FromToRotation(up, goal.Rotation * Vector3.up) * end.SolverRotation;
                    break;
                default:
                    return;
            }

            desired = Quaternion.Normalize(desired);
            end.SolverRotation = end.ClampRotation(Chain.Joints[1].SolverRotation, desired, desired);
        }
    }
}
