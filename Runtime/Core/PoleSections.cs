using System.Collections.Generic;
using UnityEngine;

namespace OpenIK
{
    // A pole entry resolved to chain indices. The swivel turns the joints between Start and End around
    // the line through them, so that Bend faces the pole.
    internal readonly struct PoleSection
    {
        public readonly int EntryIndex;
        public readonly int Start;
        public readonly int Bend;
        public readonly int End;

        public PoleSection(int entryIndex, int start, int bend, int end)
        {
            EntryIndex = entryIndex;
            Start = start;
            Bend = bend;
            End = end;
        }
    }

    // Resolves pole entries into sections. Shared by the runtime and the editor, so it reads the joint
    // components instead of runtime constraints and also works in Edit mode.
    internal static class PoleSectionResolver
    {
        private enum TwistSupport
        {
            Free,
            Twists,
            Blocks,
            Unknown
        }

        // Fills sections in chain order. Entries without a bend joint, with a bend joint that is not a
        // middle joint of the chain, or with a bend joint an earlier entry already uses are skipped and
        // reported in problems.
        public static void Resolve(
            IReadOnlyList<Transform> chainJoints,
            IReadOnlyList<IKPole> poles,
            List<PoleSection> sections,
            List<string> problems)
        {
            sections.Clear();
            problems?.Clear();
            if (chainJoints == null || poles == null)
                return;

            for (int entry = 0; entry < poles.Count; entry++)
            {
                string problem = GetProblem(chainJoints, poles, entry, sections, out int bend);
                if (problem != null)
                {
                    problems?.Add(problem);
                    continue;
                }

                InsertInChainOrder(sections, new PoleSection(entry, ResolveStart(chainJoints, bend), bend, ResolveEnd(chainJoints, bend)));
            }
        }

        // Why an entry cannot be used, or null. Earlier entries must already be in sections.
        public static string GetProblem(
            IReadOnlyList<Transform> chainJoints,
            IReadOnlyList<IKPole> poles,
            int entry,
            List<PoleSection> sections,
            out int bend)
        {
            bend = -1;
            Transform bendJoint = poles[entry]?.BendJoint;
            if (bendJoint == null)
                return "A pole has no Bend Joint.";

            bend = IndexOf(chainJoints, bendJoint);
            if (bend <= 0 || bend >= chainJoints.Count - 1)
                return $"\"{bendJoint.name}\" is not a middle joint of this chain, so its pole is not used.";

            if (ContainsBend(sections, bend))
                return $"Two poles use \"{bendJoint.name}\". Only the first is used.";

            return null;
        }

        // Removes the sections that start at or pass a joint marked in blocked, and reports each in
        // problems. A section may end at a blocked joint, because the end joint does not move. Does
        // nothing when blocked is null.
        public static void RemoveBlockedSections(
            IReadOnlyList<Transform> chainJoints,
            bool[] blocked,
            List<PoleSection> sections,
            List<string> problems)
        {
            if (blocked == null)
                return;

            for (int i = sections.Count - 1; i >= 0; i--)
            {
                string problem = GetBlockedProblem(chainJoints, sections[i], blocked);
                if (problem == null)
                    continue;

                problems?.Add(problem);
                sections.RemoveAt(i);
            }
        }

        // Why a section cannot turn because it starts at or passes a joint marked in blocked, or null.
        public static string GetBlockedProblem(IReadOnlyList<Transform> chainJoints, PoleSection section, bool[] blocked)
        {
            if (blocked == null)
                return null;

            for (int i = section.Start; i < section.End; i++)
            {
                if (!blocked[i])
                    continue;

                string bend = chainJoints[section.Bend] != null ? chainJoints[section.Bend].name : "None";
                string branch = chainJoints[i] != null ? chainJoints[i].name : "None";
                return i == section.Start
                    ? $"The pole on \"{bend}\" is not used: its section starts at \"{branch}\", where other chains branch off, and turning it would turn those chains too."
                    : $"The pole on \"{bend}\" is not used: its section passes \"{branch}\", where other chains branch off, and those chains would not turn with it.";
            }

            return null;
        }

        // True when any joint of the chain has an enabled joint component. Poles on such a chain are
        // best effort, because joint limits take priority over them.
        public static bool HasConstrainedJoint(IReadOnlyList<Transform> chainJoints)
        {
            if (chainJoints == null)
                return false;

            foreach (Transform joint in chainJoints)
            {
                if (joint != null && joint.TryGetComponent(out ConstrainedJoint constrained) && constrained.jointIsEnabled)
                    return true;
            }

            return false;
        }

        // The section start is the bend joint's parent. When that joint cannot twist, the section
        // extends up to the nearest joint that can, and the joints in between turn as one piece.
        private static int ResolveStart(IReadOnlyList<Transform> joints, int bend)
        {
            int neighbour = bend - 1;
            if (GetTwistSupport(joints, neighbour) != TwistSupport.Blocks)
                return neighbour;

            for (int j = neighbour - 1; j >= 0; j--)
            {
                TwistSupport support = GetTwistSupport(joints, j);
                if (support == TwistSupport.Free || support == TwistSupport.Twists)
                    return j;
                if (support == TwistSupport.Unknown)
                    break;
            }

            return neighbour;
        }

        // The section end is the bend joint's child. The chain end never blocks, because no joint after
        // it is constrained. Otherwise a joint that cannot twist extends the section down.
        private static int ResolveEnd(IReadOnlyList<Transform> joints, int bend)
        {
            int neighbour = bend + 1;
            int last = joints.Count - 1;
            if (neighbour == last || GetTwistSupport(joints, neighbour) != TwistSupport.Blocks)
                return neighbour;

            for (int j = neighbour + 1; j <= last; j++)
            {
                if (j == last)
                    return j;

                TwistSupport support = GetTwistSupport(joints, j);
                if (support == TwistSupport.Free || support == TwistSupport.Twists)
                    return j;
                if (support == TwistSupport.Unknown)
                    break;
            }

            return neighbour;
        }

        // Matches the runtime bindings: a joint without an enabled joint component is free. Solvers
        // do not clamp a root without a parent transform, so such a root is free as well. A custom
        // joint type is unknown, so the search does not pass it.
        private static TwistSupport GetTwistSupport(IReadOnlyList<Transform> joints, int index)
        {
            Transform joint = joints[index];
            if (joint == null)
                return TwistSupport.Unknown;

            if (!joint.TryGetComponent(out ConstrainedJoint constrained) || !constrained.jointIsEnabled)
                return TwistSupport.Free;

            if (index == 0 && joint.parent == null)
                return TwistSupport.Free;

            return constrained switch
            {
                BallSocketIKJoint => TwistSupport.Twists,
                HingeIKJoint or SliderIKJoint => TwistSupport.Blocks,
                _ => TwistSupport.Unknown
            };
        }

        private static int IndexOf(IReadOnlyList<Transform> joints, Transform joint)
        {
            for (int i = 0; i < joints.Count; i++)
            {
                if (joints[i] == joint)
                    return i;
            }

            return -1;
        }

        private static bool ContainsBend(List<PoleSection> sections, int bend)
        {
            foreach (PoleSection section in sections)
            {
                if (section.Bend == bend)
                    return true;
            }

            return false;
        }

        private static void InsertInChainOrder(List<PoleSection> sections, PoleSection section)
        {
            int index = sections.Count;
            while (index > 0 && sections[index - 1].Bend > section.Bend)
                index--;
            sections.Insert(index, section);
        }
    }
}
