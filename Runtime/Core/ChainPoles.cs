using System;
using System.Collections.Generic;
using UnityEngine;
using Object = UnityEngine.Object;

namespace OpenIK
{
    // Runtime state for a solver's poles: the resolved sections, the goal data captured at the start
    // of each solve, and the swivel that turns each section toward its pole. All per-solve work is
    // skipped when the solver has no active pole.
    internal sealed class ChainPoles
    {
        private const float MinLengthSqr = 1e-10f;
        private const float MinTurnDegrees = 1e-4f;
        private const float SettledDegrees = 0.01f;
        private const float LimitedDegrees = 1f;
        private const int MaxGroupPasses = 4;

        // Bisection steps when a joint limit shortens a turn: 6 steps resolve it to under 2 percent.
        // Solvers apply poles every iteration, so the remainder is refined on the next one.
        private const int LimitSearchSteps = 6;
        private const float MinSearchFraction = 0.01f;

        private struct Section
        {
            public IKPole Pole;
            public int Start;
            public int Bend;
            public int End;

            // Index of the last section in this section's group. Sections in a group touch or overlap,
            // so each one can move the line or the reference frame of another.
            public int GroupEnd;

            // Whether the start and end joints have limits that can shorten a turn.
            public bool CheckStart;
            public bool CheckEnd;

            // True when every joint between start and end is free, so the turn re-aims bones with the
            // smallest rotation instead of turning them rigidly, and adds no spin around the bones.
            public bool SmallestTurn;

            // Captured at the start of each solve.
            public bool Active;
            public Vector3 PolePosition;
            public float Weight;
            public bool HasIncoming;

            // The incoming bend, relative to the section start's IK parent so it follows the body.
            public Vector3 IncomingLocal;
        }

        private readonly List<PoleSection> _resolved = new();
        private readonly List<string> _problems = new();
        private Section[] _sections = Array.Empty<Section>();
        private int _count;
        private int _activeCount;

        // The entries the sections were resolved from, for the structural change check.
        private IKPole[] _entries = Array.Empty<IKPole>();
        private Transform[] _entryBendJoints = Array.Empty<Transform>();
        private int _entryCount = -1;

        public int Count => _count;

        // The joint indices of a resolved section, in sorted order. For tests.
        public void GetSection(int index, out int start, out int bend, out int end)
        {
            start = _sections[index].Start;
            bend = _sections[index].Bend;
            end = _sections[index].End;
        }

        // True when the entries or their bend joints changed since the last Resolve. Compares
        // references only, so it is cheap enough to run every solve.
        public bool NeedsResolve(List<IKPole> poles)
        {
            int count = poles.Count;
            if (count != _entryCount)
                return true;

            for (int i = 0; i < count; i++)
            {
                IKPole pole = poles[i];
                if (!ReferenceEquals(pole, _entries[i]) || !ReferenceEquals(pole?.BendJoint, _entryBendJoints[i]))
                    return true;
            }

            return false;
        }

        public void Resolve(
            List<IKPole> poles,
            IReadOnlyList<Transform> chainJoints,
            SolverChain chain,
            Object context,
            string solverName)
        {
            Resolve(poles, chainJoints, null, chain.Count - 1, null, chain, context, solverName);
        }

        // Resolves pole entries given on one chain of a tree. treeIndex maps each joint of chainJoints
        // to its index in chain, or is null when they match. tipIndex is the chain's last joint in
        // chain. blocked marks the joints of chainJoints that a section may not start at or pass,
        // such as branch points; it may be null.
        public void Resolve(
            List<IKPole> poles,
            IReadOnlyList<Transform> chainJoints,
            int[] treeIndex,
            int tipIndex,
            bool[] blocked,
            SolverChain chain,
            Object context,
            string solverName)
        {
            int entryCount = poles.Count;
            EnsureCapacity(ref _entries, entryCount);
            EnsureCapacity(ref _entryBendJoints, entryCount);
            for (int i = 0; i < entryCount; i++)
            {
                _entries[i] = poles[i];
                _entryBendJoints[i] = poles[i]?.BendJoint;
            }

            _entryCount = entryCount;

            // Only setup mistakes are logged, once per setup change. Joint limits that shorten a turn
            // are expected and reported through IKPole.IsLimited instead.
            PoleSectionResolver.Resolve(chainJoints, poles, _resolved, _problems);
            PoleSectionResolver.RemoveBlockedSections(chainJoints, blocked, _resolved, _problems);
            foreach (string problem in _problems)
                Debug.LogWarning($"[OpenIK] {solverName}: {problem}", context);

            _count = _resolved.Count;
            _activeCount = 0;
            EnsureCapacity(ref _sections, _count);
            for (int i = 0; i < _count; i++)
            {
                PoleSection resolved = _resolved[i];
                int start = MapIndex(treeIndex, resolved.Start);
                int end = MapIndex(treeIndex, resolved.End);
                _sections[i] = new Section
                {
                    Pole = poles[resolved.EntryIndex],
                    Start = start,
                    Bend = MapIndex(treeIndex, resolved.Bend),
                    End = end,
                    CheckStart = HasLimits(chain, start),
                    CheckEnd = (end != tipIndex || !chain.IsLeaf(end)) && HasLimits(chain, end),
                    SmallestTurn = InnerJointsAreFree(chain, start, end)
                };
            }

            AssignGroups();
        }

        private static int MapIndex(int[] treeIndex, int index) => treeIndex != null ? treeIndex[index] : index;

        // Reads the pole targets and weights, and records each section's incoming bend.
        public void Capture(SolverChain chain)
        {
            _activeCount = 0;
            for (int i = 0; i < _count; i++)
            {
                ref Section section = ref _sections[i];
                IKPole pole = section.Pole;
                pole.IsLimited = false;

                Transform target = pole.Target;
                section.Weight = pole.Weight;
                section.Active = section.Weight > 0f && target != null;
                if (!section.Active)
                    continue;

                _activeCount++;
                section.PolePosition = target.position;

                // A full-weight pole ignores the incoming bend, so skip recording it.
                section.HasIncoming = false;
                if (section.Weight < 1f && TryGetBend(chain, section, out Vector3 bend, out _))
                {
                    section.IncomingLocal = Quaternion.Inverse(chain.GetParentSolverRotation(section.Start)) * bend;
                    section.HasIncoming = true;
                }
            }
        }

        // Turns every active section toward its pole. Sections in a group take passes in chain order,
        // then in reverse, until the turns settle or the pass limit is reached.
        public void Apply(SolverChain chain)
        {
            if (_activeCount == 0)
                return;

            int first = 0;
            while (first < _count)
            {
                int last = _sections[first].GroupEnd;
                if (last == first)
                {
                    if (_sections[first].Active)
                        ApplySection(chain, ref _sections[first]);
                }
                else
                {
                    for (int pass = 0; pass < MaxGroupPasses; pass++)
                    {
                        bool forward = (pass & 1) == 0;
                        float largestTurn = 0f;
                        for (int offset = 0; offset <= last - first; offset++)
                        {
                            int index = forward ? first + offset : last - offset;
                            if (_sections[index].Active)
                                largestTurn = Mathf.Max(largestTurn, ApplySection(chain, ref _sections[index]));
                        }

                        if (largestTurn < SettledDegrees)
                            break;
                    }
                }

                first = last + 1;
            }
        }

        // Swivels one section toward its pole, as far as its boundary joints' limits allow. Returns
        // the applied turn in degrees.
        private float ApplySection(SolverChain chain, ref Section section)
        {
            if (!TryGetBend(chain, section, out Vector3 bend, out Vector3 axis))
                return 0f;

            SolverJoint[] joints = chain.Joints;
            if (!TryGetTargetDirection(chain, ref section, joints[section.Start].SolverPosition, axis, bend, out Vector3 target))
                return 0f;

            float turn = SignedAngle(bend, target, axis);
            if (Mathf.Abs(turn) < MinTurnDegrees)
                return 0f;

            // Joint limits take priority: shorten the turn instead of clamping afterward, because a
            // clamp correction would move the end effector and fight the solver's main goal.
            float applied = section.CheckStart || section.CheckEnd
                ? turn * GetAllowedFraction(chain, section, axis, turn)
                : turn;

            section.Pole.IsLimited = Mathf.Abs(turn - applied) > LimitedDegrees;

            if (Mathf.Abs(applied) < MinTurnDegrees)
                return 0f;

            chain.SwivelSection(section.Start, section.End, Quaternion.AngleAxis(applied, axis), section.SmallestTurn);
            return Mathf.Abs(applied);
        }

        // The direction the bend joint should face for the active pole on bendIndex: perpendicular to
        // axis, the line through origin, with Weight applied. False when no active pole is on that joint
        // or the pole lies on the line. Analytic solvers use it to place the bend directly.
        public bool TryGetBendDirection(SolverChain chain, int bendIndex, Vector3 origin, Vector3 axis, out Vector3 direction)
        {
            direction = Vector3.zero;
            if (_activeCount == 0)
                return false;

            for (int i = 0; i < _count; i++)
            {
                ref Section section = ref _sections[i];
                if (section.Bend != bendIndex || !section.Active)
                    continue;

                Vector3 bend = Vector3.ProjectOnPlane(chain.Joints[bendIndex].SolverPosition - origin, axis);
                return TryGetTargetDirection(chain, ref section, origin, axis, bend, out direction);
            }

            return false;
        }

        // The normalized direction Weight of the way from the incoming bend to the pole, perpendicular
        // to axis. False when the pole lies on the line.
        private static bool TryGetTargetDirection(SolverChain chain, ref Section section, Vector3 origin, Vector3 axis, Vector3 bend, out Vector3 target)
        {
            target = Vector3.ProjectOnPlane(section.PolePosition - origin, axis);
            if (target.sqrMagnitude < MinLengthSqr)
                return false;

            target.Normalize();
            if (section.Weight >= 1f)
                return true;

            Quaternion parentRotation = chain.GetParentSolverRotation(section.Start);
            Vector3 incoming = section.HasIncoming
                ? Vector3.ProjectOnPlane(parentRotation * section.IncomingLocal, axis)
                : Vector3.zero;

            // Without a usable incoming bend, the current one becomes the reference for the rest of
            // this solve, so repeated calls do not move the blend.
            if (incoming.sqrMagnitude < MinLengthSqr)
            {
                if (bend.sqrMagnitude < MinLengthSqr)
                    return true;

                incoming = bend;
                section.IncomingLocal = Quaternion.Inverse(parentRotation) * bend;
                section.HasIncoming = true;
            }

            target = Quaternion.AngleAxis(SignedAngle(incoming, target, axis) * section.Weight, axis) * incoming.normalized;
            return true;
        }

        // The largest fraction of the turn that keeps the section's start and end joints within their
        // limits. Only searches when the full turn breaks a limit.
        private static float GetAllowedFraction(SolverChain chain, in Section section, Vector3 axis, float turn)
        {
            if (IsWithinLimits(chain, section, axis, turn))
                return 1f;

            // A blocked section, such as one starting at a hinge, rejects even a small turn. Stop
            // there instead of searching every iteration.
            if (!IsWithinLimits(chain, section, axis, turn * MinSearchFraction))
                return 0f;

            float allowed = MinSearchFraction, rejected = 1f;
            for (int step = 0; step < LimitSearchSteps; step++)
            {
                float middle = 0.5f * (allowed + rejected);
                if (IsWithinLimits(chain, section, axis, turn * middle))
                    allowed = middle;
                else
                    rejected = middle;
            }

            return allowed;
        }

        // Whether turning the section by the given angle keeps its start joint, relative to its IK
        // parent, and its end joint, relative to the turned joint before it, within their constraints.
        // The turned rotation is also passed as the current one, so a ball socket checks the turn's
        // twist instead of keeping the current twist.
        private static bool IsWithinLimits(SolverChain chain, in Section section, Vector3 axis, float degrees)
        {
            SolverJoint[] joints = chain.Joints;
            Quaternion swivel = Quaternion.AngleAxis(degrees, axis);

            if (section.CheckStart)
            {
                SolverJoint start = joints[section.Start];
                Quaternion desired = chain.GetSwivelledRotation(section.Start, section.End, swivel, section.SmallestTurn, section.Start);
                if (start.ClampRotation(chain.GetParentSolverRotation(section.Start), desired, desired) != desired)
                    return false;
            }

            if (section.CheckEnd)
            {
                SolverJoint end = joints[section.End];
                Quaternion parentRotation = chain.GetSwivelledRotation(section.Start, section.End, swivel, section.SmallestTurn, section.End - 1);
                if (end.ClampRotation(parentRotation, end.SolverRotation, end.SolverRotation) != end.SolverRotation)
                    return false;
            }

            return true;
        }

        // The bend joint's offset from the section line, and the line's direction. False for a
        // collapsed line or a bend joint on the line, where the section has no bend to turn.
        private static bool TryGetBend(SolverChain chain, in Section section, out Vector3 bend, out Vector3 axis)
        {
            SolverJoint[] joints = chain.Joints;
            Vector3 origin = joints[section.Start].SolverPosition;
            Vector3 line = joints[section.End].SolverPosition - origin;
            float lineLengthSqr = line.sqrMagnitude;
            if (lineLengthSqr < MinLengthSqr)
            {
                bend = axis = Vector3.zero;
                return false;
            }

            axis = line / Mathf.Sqrt(lineLengthSqr);
            bend = Vector3.ProjectOnPlane(joints[section.Bend].SolverPosition - origin, axis);
            return bend.sqrMagnitude >= MinLengthSqr;
        }

        // Signed angle from one vector to another around an axis both are perpendicular to.
        private static float SignedAngle(Vector3 from, Vector3 to, Vector3 axis)
        {
            return Mathf.Atan2(Vector3.Dot(Vector3.Cross(from, to), axis), Vector3.Dot(from, to)) * Mathf.Rad2Deg;
        }

        // Free joints have no limits, and solvers leave a root without a parent transform unclamped.
        private static bool HasLimits(SolverChain chain, int index)
        {
            if (index == 0 && chain.RootParentTransform == null)
                return false;
            return chain.Joints[index].Angular is not FreeAngularConstraint;
        }

        private static bool InnerJointsAreFree(SolverChain chain, int start, int end)
        {
            for (int i = start + 1; i < end; i++)
            {
                if (chain.Joints[i].Angular is not FreeAngularConstraint)
                    return false;
            }

            return true;
        }

        private void AssignGroups()
        {
            int first = 0;
            while (first < _count)
            {
                int last = first;
                int reach = _sections[first].End;
                while (last + 1 < _count && _sections[last + 1].Start <= reach)
                {
                    last++;
                    reach = Mathf.Max(reach, _sections[last].End);
                }

                for (int i = first; i <= last; i++)
                    _sections[i].GroupEnd = last;

                first = last + 1;
            }
        }

        private static void EnsureCapacity<T>(ref T[] array, int count)
        {
            if (array.Length < count)
                array = new T[count];
        }
    }
}
