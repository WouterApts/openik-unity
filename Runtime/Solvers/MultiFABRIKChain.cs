using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace OpenIK
{
    /// <summary>How the chains starting at a shared joint connect to it.</summary>
    public enum MultiFABRIKJunctionMode
    {
        /// <summary>The branches keep fixed offsets in the joint's frame and turn it together.</summary>
        Rigid,

        /// <summary>Each branch starts at its own pivot, and the pivots rotate independently.</summary>
        Point
    }

    /// <summary>
    /// One chain of a Multi FABRIK Solver, placed on the joint the chain starts from. That joint is the
    /// junction where the chain connects to the rest of the tree. Add the chain to the solver's Chains list.
    /// </summary>
    [AddComponentMenu("OpenIK/FABRIK Chain", 5)]
    public sealed class MultiFABRIKChain : MonoBehaviour
    {
        [Tooltip("The chain's name in the inspectors and setup messages.")]
        [SerializeField] private string chainName;

        [Tooltip("How the chains on this joint connect to it.\n" +
                 "• Rigid: the joint turns as one piece and its chains move with it.\n" +
                 "• Point: the joint does not turn for its branches; each chain turns on its own pivot, a direct child at a shared position.")]
        [SerializeField] private MultiFABRIKJunctionMode junction;

        [Tooltip("The joints after this one, down to the tip. Only this joint may belong to another chain.")]
        [SerializeField] private List<Transform> joints = new();

        [Tooltip("Optional position target for the tip. An untargeted chain can still move through its targeted branches.")]
        [SerializeField] private Transform target;

        [Tooltip("Maximum tip distance to the real target for this chain to report convergence.")]
        [SerializeField, Min(0f)] private float tolerance = 0.001f;

        [Tooltip("Help a nearly straight chain bend toward a target aligned with it.")]
        [SerializeField] private bool useSingularityHandling = true;

        [SerializeField] private List<IKPole> poles = new();

        private JointList _joints;

        /// <summary>The chain's name in the inspectors and setup messages.</summary>
        public string ChainName { get => chainName; set => chainName = value; }

        /// <summary>
        /// False holds this chain's incoming local pose. Chains branching from it can still solve.
        /// The same as the component's enabled checkbox.
        /// </summary>
        public bool Enabled { get => enabled; set => enabled = value; }

        /// <summary>
        /// The joints from this component's joint to the tip. Change the topology with the solver's
        /// AddChain and RemoveChain.
        /// </summary>
        public IReadOnlyList<Transform> Joints => _joints ??= new JointList(this);

        /// <summary>
        /// How this chain connects at its first joint. Setting it sets every chain on that joint, which
        /// must all use the same mode. The tree reads it when it is built, so set it before initialization.
        /// </summary>
        public MultiFABRIKJunctionMode Junction
        {
            get => junction;
            set
            {
                foreach (MultiFABRIKChain chain in GetComponents<MultiFABRIKChain>())
                    chain.junction = value;
            }
        }

        /// <summary>The optional position target for this chain's tip.</summary>
        public Transform Target { get => target; set => target = value; }

        /// <summary>Maximum distance to the real target for this chain to report convergence.</summary>
        public float Tolerance { get => Mathf.Max(0f, tolerance); set => tolerance = Mathf.Max(0f, value); }

        /// <summary>Helps a straight chain fold toward a target on its line.</summary>
        public bool UseSingularityHandling { get => useSingularityHandling; set => useSingularityHandling = value; }

        /// <summary>The poles on this chain. Sections cannot start at or cross a shared branch point.</summary>
        public IReadOnlyList<IKPole> Poles => poles;
        internal List<IKPole> PoleEntries => poles;

        /// <summary>
        /// This chain's result from the last completed solve. Inactive before the first
        /// solve and after a tree rebuild. Unchanged when Solve skips solving.
        /// </summary>
        public MultiFABRIKChainResult LastResult { get; internal set; } = MultiFABRIKChainResult.Inactive;

        // The solver that built a tree with this chain, so OnDestroy can remove it from that solver.
        internal MultiFABRIKSolver Owner { get; set; }

        /// <summary>Adds a pole on a middle joint of this chain.</summary>
        /// <param name="bendJoint">The joint that bends toward the pole.</param>
        /// <param name="poleTarget">The pole's target transform.</param>
        /// <param name="weight">The fraction of the turn to apply, from 0 to 1.</param>
        /// <returns>The new pole entry.</returns>
        public IKPole AddPole(Transform bendJoint, Transform poleTarget, float weight = 1f)
        {
            var pole = new IKPole(bendJoint, poleTarget, weight);
            poles.Add(pole);
            return pole;
        }

        /// <summary>Removes a pole from this chain.</summary>
        /// <param name="pole">The entry to remove.</param>
        /// <returns>False if the entry did not belong to this chain.</returns>
        public bool RemovePole(IKPole pole) => poles.Remove(pole);

        // chainJoints[0] is this component's own transform; only the remaining joints are stored.
        internal void Configure(string name, IList<Transform> chainJoints, Transform chainTarget)
        {
            chainName = name;
            joints = new List<Transform>(chainJoints.Count);
            for (int i = 1; i < chainJoints.Count; i++) joints.Add(chainJoints[i]);
            target = chainTarget;
        }

        private void Reset()
        {
            chainName = name;
            MultiFABRIKChain other = null;
            foreach (MultiFABRIKChain chain in GetComponents<MultiFABRIKChain>())
                if (chain != this) other = chain;
            if (other != null) junction = other.junction;
        }

        // Unity only shows the enabled checkbox for components with OnEnable.
        private void OnEnable() { }

        private void OnDestroy()
        {
            // Skip the rebuild while the scene unloads or the application quits.
            if (Owner != null && gameObject.scene.isLoaded) Owner.OnChainDestroyed(this);
        }

        // The chain's joints with the component's own transform first, without copying the list.
        private sealed class JointList : IReadOnlyList<Transform>
        {
            private readonly MultiFABRIKChain _chain;
            private Transform _first;

            internal JointList(MultiFABRIKChain chain) => _chain = chain;

            public int Count => _chain.joints.Count + 1;

            public Transform this[int index] => index == 0 ? _first ??= _chain.transform : _chain.joints[index - 1];

            public IEnumerator<Transform> GetEnumerator()
            {
                for (int i = 0; i < Count; i++) yield return this[i];
            }

            IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
        }
    }

    /// <summary>The result for one chain in the most recent Multi FABRIK solve.</summary>
    public readonly struct MultiFABRIKChainResult
    {
        internal static readonly MultiFABRIKChainResult Inactive = new(float.NaN, false, false);

        /// <summary>The distance to the real target, or NaN when HasTarget is false.</summary>
        public float Error { get; }

        /// <summary>True when this enabled chain reached its target within its own tolerance.</summary>
        public bool Converged { get; }

        /// <summary>True when the chain was enabled and had a target during the solve.</summary>
        public bool HasTarget { get; }

        internal MultiFABRIKChainResult(float error, bool converged, bool hasTarget)
        {
            Error = error;
            Converged = converged;
            HasTarget = hasTarget;
        }
    }
}
