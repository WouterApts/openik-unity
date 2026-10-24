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
    /// 1. The chain has exactly three joints: the upper joint (shoulder or hip), the middle joint (elbow or knee),
    ///    and the end joint (wrist or ankle).
    ///
    /// 2. A pole sets which way the limb bends. Without a pole, a bent limb keeps bending the way it
    ///    already does, and a nearly straight limb bends the way it does in the rest pose.
    ///
    /// 3. Hinge constraints: A hinge on the middle joint is solved exactly within its limits, and the pole turns the whole limb around
    ///    the line from the upper joint to the target. A hinge on the upper joint is solved exactly when the middle
    ///    joint is not a hinge. The middle joint then has at most two places it can go, and the pole picks one of
    ///    them. When both joints are hinges, only the middle hinge is solved this way.
    ///
    /// 4. Other constraints: Ball-and-socket limits are applied after the solve,
    ///    so they can stop the end joint short of its target. The solver does not solve slider translation,
    ///    so slider segments keep their current length.
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

            // A hinge on the middle joint, or else on the upper joint, is solved directly around its axis.
            HingeAngularConstraint middleHinge = joints[1].Angular as HingeAngularConstraint;
            HingeAngularConstraint upperHinge = joints[0].Angular as HingeAngularConstraint;

            bool solvedHinge = false;
            if (middleHinge != null)
            {
                solvedHinge = TrySolveMiddleHinge(middleHinge, root, endTarget, reach, bend);
            }
            else if (upperHinge != null)
            {
                solvedHinge = TrySolveUpperHinge(upperHinge, root, endTarget, bend);
            }

            // Without a hinge, or when the hinge solve has no answer, the bend is solved in the bend plane.
            if (!solvedHinge)
            {
                TurnUpperAndMiddle(root, middleTarget, endTarget, bend);
            }

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

        // A hinge on the middle joint can only bend around its axis. The hinge angle that gives
        // the reach is solved directly, then the upper joint aims the limb at the target and
        // swivels it around the reach line toward the bend side.
        private bool TrySolveMiddleHinge(HingeAngularConstraint hinge, Vector3 root, Vector3 endTarget, float reach, Vector3 bend)
        {
            SolverJoint[] joints = Chain.Joints;

            // The middle joint's rotation at hinge angle 0 is its rest rotation under the current upper joint.
            Quaternion zeroRotation = joints[0].SolverRotation * joints[1].RestLocalRotation;
            Vector3 axis = zeroRotation * hinge.HingeAxis;

            // The upper bone, and the lower bone as it would point at hinge angle 0.
            Vector3 upper = joints[1].SolverPosition - root;
            Vector3 lowerLocal = Quaternion.Inverse(joints[1].SolverRotation) * (joints[2].SolverPosition - joints[1].SolverPosition);
            Vector3 lowerAtZero = zeroRotation * lowerLocal;

            if (!TrySolveHingeAngles(upper, lowerAtZero, axis, reach, out float straight, out float offset))
            {
                return false;
            }

            // The two angles bend to either side of straight. Like the bend plane solve, keep the rest
            // pose's side, or the current side when the rest pose is straight, so the limb does not flip.
            float sideAngle;
            if (_bendLocal.sqrMagnitude > 0f)
            {
                sideAngle = 0f;
            }
            else
            {
                Quaternion currentDeviation = Quaternion.Inverse(zeroRotation) * joints[1].SolverRotation;
                sideAngle = HingeIKJoint.ExtractHingeAngle(currentDeviation, hinge.HingeAxis);
            }

            bool preferSecond = Mathf.DeltaAngle(straight, sideAngle) > 0f;
            float angle = PickHingeAngle(hinge, straight - offset, straight + offset, preferSecond);

            // Set the hinge angle, then turn the upper joint so the end joint lies on the reach line.
            Quaternion middleRotation = zeroRotation * Quaternion.AngleAxis(angle, hinge.HingeAxis);
            TurnJoint(1, middleRotation * Quaternion.Inverse(joints[1].SolverRotation));
            TurnJoint(0, Quaternion.FromToRotation(joints[2].SolverPosition - root, endTarget - root));

            // Swivel the limb around the reach line so the middle joint moves to the bend side.
            // A straight limb has no bend side to swivel.
            Vector3 reachDirection = (endTarget - root).normalized;
            Vector3 middleSide = Vector3.ProjectOnPlane(joints[1].SolverPosition - root, reachDirection);
            if (middleSide.sqrMagnitude > ParallelSqr * upper.sqrMagnitude)
            {
                float swivel = Vector3.SignedAngle(middleSide, bend, reachDirection);
                TurnJoint(0, Quaternion.AngleAxis(swivel, reachDirection));
            }

            return true;
        }
        
        private bool TrySolveUpperHinge(HingeAngularConstraint hinge, Vector3 root, Vector3 endTarget, Vector3 bend)
        {
            SolverJoint[] joints = Chain.Joints;

            // The upper joint's rotation at hinge angle 0 is its rest rotation under the root's parent.
            Quaternion zeroRotation = Chain.RootParentRotation * joints[0].RestLocalRotation;
            Vector3 axis = zeroRotation * hinge.HingeAxis;

            // The upper bone as it would point at hinge angle 0, and the lower bone's length.
            Vector3 upperLocal = Quaternion.Inverse(joints[0].SolverRotation) * (joints[1].SolverPosition - root);
            Vector3 upperAtZero = zeroRotation * upperLocal;
            float lowerLength = Vector3.Distance(joints[2].SolverPosition, joints[1].SolverPosition);

            // The middle joint sits at root + upper, so it is one lower-bone length from the target when
            // |(root - endTarget) + upper| equals lowerLength.
            if (!TrySolveHingeAngles(root - endTarget, upperAtZero, axis, lowerLength, out float straight, out float offset))
            {
                return false;
            }

            // Prefer the angle that moves the middle joint further toward the bend side.
            float first = straight - offset;
            float second = straight + offset;
            Vector3 firstUpper = Quaternion.AngleAxis(first, axis) * upperAtZero;
            Vector3 secondUpper = Quaternion.AngleAxis(second, axis) * upperAtZero;
            bool preferSecond = Vector3.Dot(secondUpper, bend) > Vector3.Dot(firstUpper, bend);
            float angle = PickHingeAngle(hinge, first, second, preferSecond);

            // Set the hinge angle, then aim the lower bone at the target.
            Quaternion upperRotation = zeroRotation * Quaternion.AngleAxis(angle, hinge.HingeAxis);
            TurnJoint(0, upperRotation * Quaternion.Inverse(joints[0].SolverRotation));
            AimMiddleAt(endTarget);
            return true;
        }
        
        private static bool TrySolveHingeAngles(Vector3 p, Vector3 q, Vector3 axis, float length, out float straight, out float offset)
        {
            straight = 0f;
            offset = 0f;

            // Only the parts across the axis change as q turns.
            Vector3 pAcross = Vector3.ProjectOnPlane(p, axis);
            Vector3 qAcross = Vector3.ProjectOnPlane(q, axis);
            if (pAcross.sqrMagnitude * qAcross.sqrMagnitude <= ParallelSqr * p.sqrMagnitude * q.sqrMagnitude)
            {
                return false;
            }
            
            // Solved for the cosine that gives length. The clamp gives the nearest length possible.
            float along = Vector3.Dot(p, axis) * Vector3.Dot(q, axis);
            float cos = (length * length - p.sqrMagnitude - q.sqrMagnitude - 2f * along) / (2f * pAcross.magnitude * qAcross.magnitude);
            offset = Mathf.Acos(Mathf.Clamp(cos, -1f, 1f)) * Mathf.Rad2Deg;

            // Turning q by straight lines qAcross up with pAcross, which makes |p + q| longest.
            straight = Vector3.SignedAngle(qAcross, pAcross, axis);
            return true;
        }

        // Clamps both angles to the hinge limits and keeps the one the limits change least. When the limits
        // change both about equally, such as when both lie inside them, preferSecond decides.
        private static float PickHingeAngle(HingeAngularConstraint hinge, float first, float second, bool preferSecond)
        {
            // The limits lie within -180 to 180 degrees.
            first = Mathf.DeltaAngle(0f, first);
            second = Mathf.DeltaAngle(0f, second);

            float clampedFirst = Mathf.Clamp(first, hinge.MinAngle, hinge.MaxAngle);
            float clampedSecond = Mathf.Clamp(second, hinge.MinAngle, hinge.MaxAngle);
            float firstChange = Mathf.Abs(clampedFirst - first);
            float secondChange = Mathf.Abs(clampedSecond - second);

            bool pickSecond;
            if (Mathf.Abs(firstChange - secondChange) > AngleTie)
                pickSecond = secondChange < firstChange;
            else
                pickSecond = preferSecond;

            return pickSecond ? clampedSecond : clampedFirst;
        }

        // Turns the middle joint so the lower bone points at endTarget.
        private void AimMiddleAt(Vector3 endTarget)
        {
            SolverJoint[] joints = Chain.Joints;
            Vector3 lower = joints[2].SolverPosition - joints[1].SolverPosition;
            Vector3 toTarget = endTarget - joints[1].SolverPosition;
            TurnJoint(1, Quaternion.FromToRotation(lower, toTarget));
        }

        // Turns a joint by a world-space rotation and carries the joints below it along.
        private void TurnJoint(int index, Quaternion turn)
        {
            SolverJoint joint = Chain.Joints[index];
            joint.SolverRotation = Quaternion.Normalize(turn * joint.SolverRotation);
            Chain.RotateAndRepositionDownstream(index, turn);
        }

        // Clamps the upper and middle joints to their constraints.
        private void ClampUpperAndMiddle()
        {
            SolverJoint[] joints = Chain.Joints;
            for (int i = 0; i < 2; i++)
            {
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
