using System;
using System.Collections.Generic;
using UnityEngine;

namespace OpenIK
{
    /// <summary>
    /// Solves position targets for a tree of FABRIK chains joined by rigid or point junctions. Each
    /// chain is a <see cref="MultiFABRIKChain"/> component on the joint it starts from, listed in
    /// Chains with its Pull and Pin.
    /// </summary>
    [AddComponentMenu("OpenIK/Multi FABRIK Solver", 4)]
    public class MultiFABRIKSolver : OpenIKSolverBase
    {
        [Serializable]
        private sealed class ChainEntry
        {
            [SerializeField] internal MultiFABRIKChain chain;

            [Tooltip("How strongly this chain pulls the joint it starts from, from 0 to 1.")]
            [SerializeField, Range(0f, 1f)] internal float pull = 1f;

            [Tooltip("How strongly this chain resists the chains branching from its joints, from 0 to 1.")]
            [SerializeField, Range(0f, 1f)] internal float pin;

            internal float Pull => Mathf.Clamp01(pull);
            internal float Pin => Mathf.Clamp01(pin);
        }

        [Tooltip("The chains this solver runs, each with its Pull and Pin. The list order does not matter; the solver sorts the chains into a tree.")]
        [SerializeField] private List<ChainEntry> chains = new();

        [Tooltip("How strongly the root keeps its incoming position, from 0 to 1. A rigid root with several chains also keeps its rotation. A point root always keeps its incoming local reference frame.")]
        [SerializeField, Range(0f, 1f)] private float rootPin = 1f;

        // The listed chains in tree order, and the index of each one's item in the Chains list. Unity
        // can replace the items when the Inspector applies an edit, so the solver stores indices, not
        // item references.
        private readonly List<MultiFABRIKChain> _chains = new();
        private int[] _entryIndices = Array.Empty<int>();
        private MultiFABRIKTree _tree;
        private readonly List<string> _problems = new();

        // One chain's settings and target, read once per solve. Active means enabled with a target.
        // When Root Pin is 1, Reachable is the target moved within the tip's reach from the root.
        // Singularity is a sideways offset for the first iteration, so a straight chain can bend.
        private struct Goal
        {
            internal bool Enabled, Active;
            internal Vector3 Target, Reachable, Singularity;
            internal float Pull, Pin, Tolerance;
        }

        private Goal[] _goals;

        // Per tree joint. Driven: an active target at or below it pulls on it. Held: it keeps its
        // incoming pose relative to its parent, because its chain is disabled or nothing drives it.
        // Moved is scratch for TranslateSubtree and is all false between calls.
        private bool[] _driven, _held, _moved;

        // Each joint's rotation and offset relative to its parent when the solve starts, for held joints.
        private Quaternion[] _incomingLocal;
        private Vector3[] _incomingOffset;

        // Buffers for fitting a joint to its children, sized for the joint with the most children.
        private Vector3[] _fitRest, _fitGoals;
        private float[] _fitWeights;
        private Vector3 _rootPosition;
        private Quaternion _rootRotation;
        private float _rootPin;
        private bool _converged;

        protected override string SolverName => "Multi FABRIK Solver";
        private SolverJoint[] Joints => Chain.Joints;

        /// <summary>The listed chains in tree order: each chain after the chain that owns its first joint.</summary>
        /// <remarks>Before initialization, each read sorts the chains again.</remarks>
        public IReadOnlyList<MultiFABRIKChain> Chains
        {
            get
            {
                if (!IsInitialized)
                {
                    CollectChains(_chains, null);
                    MultiFABRIKTree.Sort(_chains, _problems);
                }
                return _chains;
            }
        }

        /// <summary>How strongly the root keeps its incoming position, from 0 to 1.</summary>
        /// <remarks>
        /// When several chains start at a rigid root, Root Pin also holds the root's rotation. A rigid
        /// root with one chain turns like the first joint of a FABRIK chain. A point root always keeps
        /// its incoming local reference orientation, whatever the Root Pin.
        /// </remarks>
        public float RootPin { get => Mathf.Clamp01(rootPin); set => rootPin = Mathf.Clamp01(value); }
        
        public float GetPull(MultiFABRIKChain chain) => Entry(chain).Pull;
        
        public void SetPull(MultiFABRIKChain chain, float pull) => Entry(chain).pull = Mathf.Clamp01(pull);
        
        public float GetPin(MultiFABRIKChain chain) => Entry(chain).Pin;
        
        public void SetPin(MultiFABRIKChain chain, float pin) => Entry(chain).pin = Mathf.Clamp01(pin);

        /// <summary>Adds a FABRIK Chain component to joints[0] and lists it in Chains.</summary>
        /// <param name="name">The chain's name.</param>
        /// <param name="joints">The chain's joints, from its first joint to the tip.</param>
        /// <param name="target">The position target, or null.</param>
        /// <param name="pull">How strongly the chain pulls its first joint, from 0 to 1.</param>
        /// <param name="pin">How strongly the chain resists chains that branch from it, from 0 to 1.</param>
        /// <param name="junction">The mode for every chain on joints[0]. Null keeps the current mode, or uses Rigid.</param>
        /// <returns>The new chain.</returns>
        public MultiFABRIKChain AddChain(string name, IList<Transform> joints, Transform target = null,
            float pull = 1f, float pin = 0f, MultiFABRIKJunctionMode? junction = null)
        {
            if (joints == null) throw new ArgumentNullException(nameof(joints));
            if (joints.Count == 0 || joints[0] == null) throw new ArgumentException("The first joint is missing.", nameof(joints));
            Transform first = joints[0];
            MultiFABRIKChain existing = first.GetComponent<MultiFABRIKChain>();
            MultiFABRIKJunctionMode previous = existing != null ? existing.Junction : MultiFABRIKJunctionMode.Rigid;
            MultiFABRIKChain chain = first.gameObject.AddComponent<MultiFABRIKChain>();
            chain.Configure(name, joints, target);
            chain.Junction = junction ?? previous;
            try
            {
                AddChain(chain, pull, pin);
            }
            catch (ArgumentException)
            {
                if (Application.isPlaying) Destroy(chain);
                else DestroyImmediate(chain);
                if (existing != null) existing.Junction = previous;
                throw;
            }
            return chain;
        }

        /// <summary>Lists an existing chain component in Chains.</summary>
        /// <param name="chain">The chain to list.</param>
        /// <param name="pull">How strongly the chain pulls its first joint, from 0 to 1.</param>
        /// <param name="pin">How strongly the chain resists chains that branch from it, from 0 to 1.</param>
        /// <exception cref="ArgumentNullException">Thrown when chain is null.</exception>
        /// <exception cref="ArgumentException">
        /// Thrown, without listing the chain, when it is already listed or would make an initialized
        /// tree invalid.
        /// </exception>
        /// <remarks>On an initialized solver, this resets the joints to their rest pose and rebuilds the tree.</remarks>
        public void AddChain(MultiFABRIKChain chain, float pull = 1f, float pin = 0f)
        {
            if (chain == null) throw new ArgumentNullException(nameof(chain));
            if (IndexOf(chain) >= 0) throw new ArgumentException($"{MultiFABRIKTree.Label(chain)} is already in Chains.", nameof(chain));
            chains.Add(new ChainEntry { chain = chain, pull = Mathf.Clamp01(pull), pin = Mathf.Clamp01(pin) });
            if (!IsInitialized) return;
            var listed = new List<MultiFABRIKChain>();
            if (!BuildTree(listed, new MultiFABRIKTree(), _problems))
            {
                chains.RemoveAt(chains.Count - 1);
                throw new ArgumentException(string.Join("\n", _problems), nameof(chain));
            }
            RestoreRestPose();
            Rebuild();
        }

        /// <summary>Removes a chain from Chains. The component stays on its joint.</summary>
        /// <param name="chain">The chain to remove.</param>
        /// <returns>False if the chain is not listed.</returns>
        /// <exception cref="InvalidOperationException">
        /// Thrown, without removing the chain, when other chains branch from it or removing it would make an
        /// initialized tree invalid.
        /// </exception>
        /// <remarks>On an initialized solver, this resets the joints to their rest pose and rebuilds the tree.</remarks>
        public bool RemoveChain(MultiFABRIKChain chain)
        {
            int index = IndexOf(chain);
            if (chain == null || index < 0) return false;
            foreach (ChainEntry other in chains)
            {
                if (other.chain == null || other.chain == chain) continue;
                for (int j = 1; j < chain.Joints.Count; j++)
                    if (other.chain.transform == chain.Joints[j])
                        throw new InvalidOperationException($"Remove {MultiFABRIKTree.Label(other.chain)} before {MultiFABRIKTree.Label(chain)}, which it branches from.");
            }
            ChainEntry removed = chains[index];
            chains.RemoveAt(index);
            if (IsInitialized)
            {
                if (!BuildTree(new List<MultiFABRIKChain>(), new MultiFABRIKTree(), _problems))
                {
                    chains.Insert(index, removed);
                    throw new InvalidOperationException(string.Join("\n", _problems));
                }
                RestoreRestPose();
                Rebuild();
            }
            chain.Owner = null;
            return true;
        }

        // A chain calls this from OnDestroy, except when its scene is unloading. A disabled solver,
        // including one destroyed together with its rig, rebuilds on its next solve.
        internal void OnChainDestroyed(MultiFABRIKChain chain)
        {
            int index = IndexOf(chain);
            if (index < 0) return;
            chains.RemoveAt(index);
            if (!IsInitialized) return;
            RestoreRestPose();
            if (enabled && gameObject.activeInHierarchy) Rebuild();
            else ResetInitialization();
        }

        // True when the chain is in the Chains list.
        internal bool Lists(MultiFABRIKChain chain) => chain != null && IndexOf(chain) >= 0;

        // Collects the listed chains, sorts them into tree order, and builds the tree. Reports empty
        // items and chains listed twice before the tree's own problems.
        internal bool BuildTree(List<MultiFABRIKChain> listed, MultiFABRIKTree tree, List<string> problems)
        {
            var listProblems = new List<string>();
            CollectChains(listed, listProblems);
            bool valid = tree.Build(listed, problems);
            problems.InsertRange(0, listProblems);
            return valid && listProblems.Count == 0;
        }

        // The listed chains in list order, without empty items or repeats.
        private void CollectChains(List<MultiFABRIKChain> result, List<string> problems)
        {
            result.Clear();
            for (int i = 0; i < chains.Count; i++)
            {
                MultiFABRIKChain chain = chains[i].chain;
                if (chain == null) problems?.Add($"Chains item {i} has no chain. Assign one or remove the item.");
                else if (result.Contains(chain)) problems?.Add($"{MultiFABRIKTree.Label(chain)}: it is listed more than once.");
                else result.Add(chain);
            }
        }

        private int IndexOf(MultiFABRIKChain chain)
        {
            for (int i = 0; i < chains.Count; i++)
                if (ReferenceEquals(chains[i].chain, chain)) return i;
            return -1;
        }

        // The list item of a chain in the tree, found again if the list changed since initialization.
        private ChainEntry EntryAt(int c)
        {
            int index = _entryIndices[c];
            if (index >= chains.Count || !ReferenceEquals(chains[index].chain, _chains[c]))
                _entryIndices[c] = index = IndexOf(_chains[c]);
            return index >= 0 ? chains[index] : Default;
        }

        private static readonly ChainEntry Default = new();

        private ChainEntry Entry(MultiFABRIKChain chain)
        {
            int index = IndexOf(chain);
            if (index < 0) throw new ArgumentException($"{(chain != null ? MultiFABRIKTree.Label(chain) : "The chain")} is not in Chains.", nameof(chain));
            return chains[index];
        }

        protected override bool ValidateSetup()
        {
            var tree = new MultiFABRIKTree();
            if (!BuildTree(_chains, tree, _problems))
            {
                foreach (string problem in _problems) Debug.LogError($"[OpenIK] {SolverName}: {problem}", this);
                enabled = false;
                return false;
            }
            _tree = tree;
            _entryIndices = new int[_chains.Count];
            for (int c = 0; c < _chains.Count; c++)
            {
                _entryIndices[c] = IndexOf(_chains[c]);
                _chains[c].Owner = this;
            }
            return true;
        }

        protected override void BuildChain(SolverChain chain)
        {
            chain.Initialize(_tree.Joints, _tree.ParentIndices);
            int count = chain.Count;
            _goals = new Goal[_chains.Count];
            foreach (MultiFABRIKChain entry in _chains) entry.LastResult = MultiFABRIKChainResult.Inactive;
            _driven = new bool[count];
            _held = new bool[count];
            _moved = new bool[count];
            _incomingLocal = new Quaternion[count];
            _incomingOffset = new Vector3[count];
            int mostChildren = 1;
            for (int i = 0; i < count; i++) mostChildren = Mathf.Max(mostChildren, chain.GetChildCount(i));
            _fitRest = new Vector3[mostChildren];
            _fitGoals = new Vector3[mostChildren];
            _fitWeights = new float[mostChildren];
            ValidatePointBindings();
            ResolvePoles(true);
        }

        protected override bool HasGoal
        {
            get
            {
                for (int c = 0; c < _chains.Count; c++)
                    if (_chains[c] != null && _chains[c].Enabled && _chains[c].Target != null) return true;
                return false;
            }
        }

        private protected override void OnConfigurationRefreshed(bool rebound)
        {
            if (rebound) ValidatePointBindings();
            ResolvePoles(rebound);
        }

        private void ValidatePointBindings()
        {
            if (!_tree.HasPointJunctions) return;
            for (int i = 0; i < Joints.Length; i++)
            {
                if (_tree.PointPivots[i] && Joints[i].Segment is not RigidSegmentConstraint)
                    InvalidPointBinding(i, "a branch pivot must keep its rigid attachment offset");
                if (_tree.PointJunctions[i] && Joints[i].Angular is not FreeAngularConstraint
                    && Joints[i].Angular is not FixedAngularConstraint)
                    InvalidPointBinding(i, "put angular limits on the independent branch pivots");
            }
        }

        private void InvalidPointBinding(int index, string problem)
        {
            enabled = false;
            throw new InvalidOperationException($"Point junction at {_tree.Joints[index].name}: {problem}.");
        }

        private void ResolvePoles(bool force)
        {
            for (int c = 0; c < _chains.Count; c++)
            {
                MultiFABRIKTree.ChainData data = _tree.Chains[c];
                if (force || data.Poles.NeedsResolve(_chains[c].PoleEntries))
                    data.Poles.Resolve(_chains[c].PoleEntries, _chains[c].Joints, data.TreeIndex,
                        data.TipIndex, data.Blocked, Chain, this, $"{SolverName}, {MultiFABRIKTree.Label(_chains[c])}");
            }
        }

        // Runs forward and backward passes until every active tip is within tolerance of its
        // reachable goal, the solver reaches MaxIterations, or the error stops improving. Final
        // errors and convergence use the actual targets.
        protected override SolveResult SolveChain()
        {
            Capture();
            ApplyPoles(true);
            FinalizeTreePose();
            for (int c = 0; c < _chains.Count; c++)
                if (_goals[c].Active && _chains[c].UseSingularityHandling)
                    _goals[c].Singularity = GetSingularityOffset(c);

            int iteration = 0, stalled = 0;
            float previous = MeasureReachable(out bool reached);
            float minimumTolerance = float.PositiveInfinity;
            for (int c = 0; c < _chains.Count; c++)
                if (_goals[c].Active) minimumTolerance = Mathf.Min(minimumTolerance, _goals[c].Tolerance);
            float stallThreshold = Mathf.Max(1e-7f, minimumTolerance * 0.01f);
            while (!reached && iteration < MaxIterations)
            {
                ForwardReach(iteration);
                BackwardReach();
                ApplyPoles(false);
                Chain.ReconstructRotations();
                FinalizeTreePose();
                iteration++;
                float error = MeasureReachable(out reached);
                stalled = previous - error < stallThreshold ? stalled + 1 : 0;
                previous = error;
                if (stalled >= 2) break;
            }

            float largest = 0f;
            _converged = true;
            for (int c = 0; c < _chains.Count; c++)
            {
                Goal goal = _goals[c];
                float error = goal.Active ? Vector3.Distance(Joints[_tree.Chains[c].TipIndex].SolverPosition, goal.Target) : float.NaN;
                bool converged = goal.Active && error <= goal.Tolerance;
                _chains[c].LastResult = new MultiFABRIKChainResult(error, converged, goal.Active);
                if (!goal.Active) continue;
                _converged &= converged;
                largest = Mathf.Max(largest, error);
            }
            return new SolveResult(iteration, largest);
        }

        // Reads this solve's targets and settings, marks the driven and held joints, and stores the
        // incoming local poses.
        private void Capture()
        {
            _rootPosition = Joints[0].SolverPosition;
            _rootRotation = Joints[0].SolverRotation;
            _rootPin = RootPin;
            Array.Clear(_driven, 0, _driven.Length);
            for (int c = 0; c < _chains.Count; c++)
            {
                MultiFABRIKChain entry = _chains[c];
                bool active = entry.Enabled && entry.Target != null;
                Vector3 target = active ? entry.Target.position : Vector3.zero;
                int tip = _tree.Chains[c].TipIndex;
                Vector3 delta = target - _rootPosition;
                float reach = Chain.GetPathLength(tip);
                // Only a fully pinned root limits the goal to the tip's reach. A root that can move
                // may bring a farther target within reach.
                Vector3 reachable = _rootPin == 1f && reach > 0f && delta.magnitude > reach
                    ? _rootPosition + delta.normalized * reach : target;
                _goals[c] = new Goal { Enabled = entry.Enabled, Active = active, Target = target,
                    Reachable = reachable, Pull = EntryAt(c).Pull, Pin = EntryAt(c).Pin, Tolerance = entry.Tolerance };
                if (active) _driven[tip] = true;
            }
            for (int i = Joints.Length - 1; i >= 0; i--)
            {
                int owner = _tree.Owners[i], parent = Joints[i].ParentIndex;
                bool enabledChain = owner < 0 || _goals[owner].Enabled;
                _held[i] = !enabledChain || !_driven[i];
                if (parent >= 0 && !_held[i]) _driven[parent] = true;
                Quaternion parentRotation = Chain.GetParentSolverRotation(i);
                _incomingLocal[i] = Quaternion.Inverse(parentRotation) * Joints[i].SolverRotation;
                _incomingOffset[i] = parent >= 0
                    ? Quaternion.Inverse(parentRotation) * (Joints[i].SolverPosition - Joints[parent].SolverPosition)
                    : Vector3.zero;
            }
        }

        // The tip-to-root pass. Moves each targeted tip to its goal, then fits each joint that is
        // not held to its children.
        private void ForwardReach(int iteration)
        {
            for (int i = Joints.Length - 1; i >= 0; i--)
            {
                if (_held[i]) continue;
                int tipChain = _tree.TipChains[i];
                Vector3 ownPosition = Joints[i].SolverPosition;
                Quaternion ownRotation = Joints[i].SolverRotation;
                if (tipChain >= 0 && _goals[tipChain].Active)
                    ownPosition = _goals[tipChain].Reachable + (iteration == 0 ? _goals[tipChain].Singularity : Vector3.zero);
                FitJoint(i, true, ref ownPosition, ref ownRotation);
                Joints[i].SolverPosition = ownPosition;
                Joints[i].SolverRotation = ownRotation;

                // Children stay at their forward proposals until the backward pass. This pass moves
                // a subtree only to correct a joint limit.
                for (int k = 0; k < Chain.GetChildCount(i); k++)
                {
                    int child = Chain.GetChild(i, k);
                    if (!_held[child]) CorrectChildLimit(i, child);
                }
            }
        }

        // Fits a joint's pose to its children's current positions. It sets only this joint's pose and
        // does not move the children. When fitPosition is false, only the rotation is fitted.
        private void FitJoint(int index, bool fitPosition, ref Vector3 position, ref Quaternion rotation)
        {
            if (_tree.PointJunctions[index])
            {
                FitPointJoint(index, fitPosition, ref position, ref rotation);
            }
            else FitRigidJoint(index, fitPosition, ref position, ref rotation);
        }

        // Fits a rigid junction or an ordinary joint. Its children turn with it as one piece, so it takes
        // one rotation that suits all their offsets. The own-chain child aims the joint as in regular FABRIK,
        // and the branches pull it toward a weighted fit by min(1, summed branch Pull) * (1 - Pin).
        private void FitRigidJoint(int index, bool fitPosition, ref Vector3 position, ref Quaternion rotation)
        {
            int count = Chain.GetChildCount(index);
            if (count == 0) return;
            int owner = _tree.Owners[index];
            Vector3 ownPosition = position;
            Quaternion ownRotation = rotation;
            float branchPull = 0f;
            for (int k = 0; k < count; k++)
            {
                int child = Chain.GetChild(index, k);
                int childOwner = _tree.Owners[child];
                bool continuation = owner >= 0 && childOwner == owner;
                float weight = _held[child] ? 0f : continuation ? 1f : _goals[childOwner].Pull;
                _fitRest[k] = Joints[child].Segment.GetCurrentLocalOffset();
                _fitGoals[k] = Joints[child].SolverPosition;
                _fitWeights[k] = weight;
                if (!continuation) branchPull += weight;
                if (continuation && weight > 0f)
                {
                    if (fitPosition) ownPosition = RepositionParent(index, child);
                    ownRotation = Aim(ownRotation, _fitRest[k], Joints[child].SolverPosition - ownPosition);
                }
            }
            if (index == 0)
            {
                ownPosition = _rootPosition;
                ownRotation = _rootRotation;
            }
            float pin = owner < 0 ? (fitPosition ? 0f : _rootPin) : _goals[owner].Pin;
            float share = Mathf.Min(1f, branchPull) * (1f - pin);
            float rotationShare = index == 0 && count == 1 ? Mathf.Min(1f, branchPull) : share;
            Vector3 fitPos = ownPosition;
            Quaternion fitRot = ownRotation;
            
            // One child is a regular FABRIK step, so only junctions with several children use RigidFit.
            if (count == 1)
            {
                int child = Chain.GetChild(index, 0);
                if (_fitWeights[0] > 0f)
                {
                    if (fitPosition) fitPos = RepositionParent(index, child);
                    fitRot = Aim(rotation, _fitRest[0], Joints[child].SolverPosition - fitPos);
                }
            }
            else if (fitPosition)
            {
                RigidFit.FitPose(_fitRest, _fitGoals, _fitWeights, count, ownPosition, ownRotation,
                    out fitPos, out fitRot);
            }
            else
            {
                for (int k = 0; k < count; k++) _fitGoals[k] -= position;
                fitRot = RigidFit.FitRotation(_fitRest, _fitGoals, _fitWeights, count, ownRotation);
            }
            
            
            if (fitPosition) position = Vector3.Lerp(ownPosition, fitPos, share);
            rotation = Quaternion.Slerp(ownRotation, fitRot, rotationShare);
        }

        // Fits a point junction. Its rotation is the point frame, so its children turn independently,
        // each within its own limits. Each child proposes a position for this joint from its rest offset, so all proposals describe
        // the same point, even when a child is offset from the joint. Only the position is fitted.
        private void FitPointJoint(int index, bool fitPosition, ref Vector3 position, ref Quaternion rotation)
        {
            rotation = PointFrame(index);
            if (!fitPosition) return;
            int owner = _tree.Owners[index];
            Vector3 ownPosition = index == 0 ? _rootPosition : position;
            Vector3 sum = Vector3.zero;
            float total = 0f, branchPull = 0f;
            for (int k = 0; k < Chain.GetChildCount(index); k++)
            {
                int child = Chain.GetChild(index, k);
                int childOwner = _tree.Owners[child];
                bool continuation = owner >= 0 && childOwner == owner;
                float weight = _held[child] ? 0f : continuation ? 1f : _goals[childOwner].Pull;
                Vector3 proposal = Joints[child].SolverPosition - rotation * Joints[child].RestLocalOffset;
                sum += weight * proposal;
                total += weight;
                if (!continuation) branchPull += weight;
                else if (weight > 0f) ownPosition = proposal;
            }
            float pin = owner < 0 ? 0f : _goals[owner].Pin;
            position = total > 0f
                ? Vector3.Lerp(ownPosition, sum / total, Mathf.Min(1f, branchPull) * (1f - pin))
                : ownPosition;
        }

        private Quaternion PointFrame(int index) => Chain.GetParentSolverRotation(index)
            * (!_held[index] && Joints[index].Angular is FixedAngularConstraint
                ? Joints[index].RestLocalRotation : _incomingLocal[index]);

        // The root-to-tip pass. Pins the root, then puts each joint at its segment length from its
        // parent and turns it toward its children within its limits. Held joints get their incoming
        // local pose back.
        private void BackwardReach()
        {
            // Pin the root after the forward pass's limit corrections. Move only the root, so its
            // children stay on their goals.
            Joints[0].SolverPosition = Vector3.Lerp(Joints[0].SolverPosition, _rootPosition, _rootPin);
            if (_held[0])
            {
                Joints[0].SolverPosition = _rootPosition;
                Joints[0].SolverRotation = _rootRotation;
            }
            else
            {
                Quaternion rootRotation = Joints[0].SolverRotation;
                if (Chain.GetChildCount(0) > 1 || _tree.PointJunctions[0])
                {
                    Vector3 position = Joints[0].SolverPosition;
                    FitJoint(0, false, ref position, ref rootRotation);
                    Joints[0].SolverRotation = rootRotation;
                }
                // Without a parent transform, the root is constrained in world space.
                Quaternion clamped = ClampTreeRotation(0, Chain.RootParentRotation, rootRotation, rootRotation);
                if (!SameRotation(clamped, rootRotation))
                {
                    Joints[0].SolverRotation = clamped;
                    Chain.RotateAndRepositionDownstream(0, clamped * Quaternion.Inverse(rootRotation));
                }
            }
            for (int i = 1; i < Joints.Length; i++)
            {
                int parent = Joints[i].ParentIndex;
                Quaternion parentRotation = Joints[parent].SolverRotation;
                Quaternion before = Joints[i].SolverRotation;
                Quaternion desired;
                if (_held[i])
                {
                    Joints[i].SolverPosition = Joints[parent].SolverPosition + parentRotation * _incomingOffset[i];
                    Joints[i].Segment.SyncFromLocalOffset(_incomingOffset[i]);
                    desired = parentRotation * _incomingLocal[i];
                }
                else
                {
                    Joints[i].SolverPosition = Joints[i].Segment.ConstrainChildPosition(new ISegmentConstraint.SolveContext(
                        Joints[parent].SolverPosition, parentRotation, Joints[i].SolverPosition));
                    desired = before;
                    Vector3 position = Joints[i].SolverPosition;
                    FitJoint(i, false, ref position, ref desired);
                    desired = ClampTreeRotation(i, parentRotation, before, desired);
                }
                Joints[i].SolverRotation = desired;
                Chain.RotateAndRepositionDownstream(i, desired * Quaternion.Inverse(before));
            }
        }

        private void FinalizeTreePose()
        {
            if (!Chain.IsTree && !_tree.HasPointJunctions) return;
            // Quaternion == accepts small angular differences in the shared clamp and pole code.
            // Project the tree once more without that comparison, then rebuild its rigid offsets.
            for (int i = 0; i < Joints.Length; i++)
            {
                SolverJoint joint = Joints[i];
                Quaternion parentRotation = Chain.GetParentSolverRotation(i);
                if (_tree.PointJunctions[i])
                {
                    Quaternion before = joint.SolverRotation;
                    joint.SolverRotation = PointFrame(i).normalized;
                    if (!before.Equals(joint.SolverRotation))
                        Chain.RotateAndRepositionDownstream(i, joint.SolverRotation * Quaternion.Inverse(before));
                }
                else if (_held[i])
                    joint.SolverRotation = parentRotation * _incomingLocal[i];
                else if (joint.Angular is not FreeAngularConstraint)
                {
                    Quaternion deviation = joint.ToConstraintDeviation(parentRotation, joint.SolverRotation);
                    joint.SolverRotation = joint.FromConstraintDeviation(parentRotation,
                        joint.Angular.ClampDeviation(deviation, deviation)).normalized;
                }
                if (i > 0)
                    joint.SolverPosition = Joints[joint.ParentIndex].SolverPosition + parentRotation
                        * (_held[i] ? _incomingOffset[i] : joint.Segment.GetCurrentLocalOffset());
            }
        }

        private Vector3 RepositionParent(int parent, int child) =>
            Joints[child].Segment.ConstrainParentPosition(new ISegmentConstraint.SolveContext(
                Joints[parent].SolverPosition, Joints[parent].SolverRotation, Joints[child].SolverPosition));

        private Quaternion Aim(Quaternion rotation, Vector3 offset, Vector3 desired)
        {
            Vector3 current = rotation * offset;
            return current.sqrMagnitude > 0f && desired.sqrMagnitude > 0f
                ? DirectionTurn(current, desired) * rotation : rotation;
        }

        private Quaternion DirectionTurn(Vector3 from, Vector3 to)
        {
            if (!_tree.HasPointJunctions) return Quaternion.FromToRotation(from, to);
            // FromToRotation drops small turns, which can stall a point branch above its tolerance.
            from.Normalize();
            to.Normalize();
            Vector3 cross = Vector3.Cross(from, to);
            float sine = cross.magnitude;
            float cosine = Vector3.Dot(from, to);
            return sine > 1e-10f ? Quaternion.AngleAxis(Mathf.Atan2(sine, cosine) * Mathf.Rad2Deg, cross / sine)
                : Quaternion.FromToRotation(from, to);
        }

        // FinalizeTreePose clamps tree poses strictly. Clamp the same way during the iterations, so
        // the solver cannot accept a small violation on every iteration and then remove it at the end.
        private Quaternion ClampTreeRotation(int index, Quaternion parent, Quaternion current, Quaternion desired)
        {
            SolverJoint joint = Joints[index];
            
            if (!Chain.IsTree && !_tree.HasPointJunctions) return joint.ClampRotation(parent, current, desired);
            if (joint.Angular is FreeAngularConstraint) return desired;
            
            Quaternion deviation = joint.ToConstraintDeviation(parent, desired);
            Quaternion? incoming = joint.Angular.NeedsCurrentDeviation ? joint.ToConstraintDeviation(parent, current) : null;
            Quaternion clamped = joint.Angular.ClampDeviation(deviation, incoming);
            
            return clamped.Equals(deviation) ? desired : joint.FromConstraintDeviation(parent, clamped);
        }

        private bool SameRotation(Quaternion a, Quaternion b) => Chain.IsTree || _tree.HasPointJunctions ? a.Equals(b) : a == b;
        
        private void CorrectChildLimit(int parent, int child)
        {
            Quaternion before = Joints[child].SolverRotation;
            Quaternion clamped = ClampTreeRotation(child, Joints[parent].SolverRotation, before, before);
            
            if (SameRotation(clamped, before)) return;
            bool hasCentroid = TryTipCentroid(parent, out Vector3 centroid);
            Joints[child].SolverRotation = clamped;
            Chain.RotateAndRepositionDownstream(child, clamped * Quaternion.Inverse(before));
            if (!hasCentroid || !TryTipCentroid(parent, out Vector3 after)) return;
            Vector3 from = after - Joints[parent].SolverPosition, to = centroid - Joints[parent].SolverPosition;
            if (!_tree.PointJunctions[parent] && from.sqrMagnitude > 0f && to.sqrMagnitude > 0f)
            {
                Quaternion compensation = DirectionTurn(from, to);
                Joints[parent].SolverRotation = compensation * Joints[parent].SolverRotation;
                Chain.RotateAndRepositionDownstream(parent, compensation);
            }
            TryTipCentroid(parent, out after);
            TranslateSubtree(parent, centroid - after);
        }
        
        private bool TryTipCentroid(int root, out Vector3 centroid)
        {
            centroid = Vector3.zero;
            float total = 0f;
            foreach (int c in _tree.SubtreeTips[root])
            {
                if (!_goals[c].Active) continue;
                int tip = _tree.Chains[c].TipIndex;
                float weight = 1f;
                for (int i = tip; i != root; i = Joints[i].ParentIndex)
                {
                    if (_held[i]) { weight = 0f; break; }
                    int parent = Joints[i].ParentIndex;
                    int owner = _tree.Owners[parent];
                    if (_tree.Owners[i] != owner)
                        weight *= _goals[_tree.Owners[i]].Pull * (owner < 0 ? 1f : 1f - _goals[owner].Pin);
                }
                centroid += weight * Joints[tip].SolverPosition;
                total += weight;
            }
            if (total <= 0f) return false;
            centroid /= total;
            return true;
        }

        private void TranslateSubtree(int root, Vector3 offset)
        {
            _moved[root] = true;
            Joints[root].SolverPosition += offset;
            for (int i = root + 1; i < Joints.Length; i++)
            {
                _moved[i] = _moved[Joints[i].ParentIndex];
                if (_moved[i]) Joints[i].SolverPosition += offset;
            }
            Array.Clear(_moved, root, _moved.Length - root);
        }

        private void ApplyPoles(bool capture)
        {
            for (int c = 0; c < _chains.Count; c++)
            {
                MultiFABRIKTree.ChainData data = _tree.Chains[c];
                if (!_goals[c].Enabled || !_driven[data.TipIndex]) continue;
                if (capture) data.Poles.Capture(Chain);
                data.Poles.Apply(Chain);
            }
        }

        // FABRIK keeps a straight chain straight when the target lies on its line. This offset nudges
        // the tip's first goal sideways so the chain starts to bend.
        private Vector3 GetSingularityOffset(int c)
        {
            MultiFABRIKTree.ChainData data = _tree.Chains[c];
            Vector3 root = Joints[data.BranchIndex].SolverPosition;
            Vector3 toEnd = Joints[data.TipIndex].SolverPosition - root;
            Vector3 toTarget = _goals[c].Reachable - root;
            float length = Chain.GetPathLength(data.TipIndex) - Chain.GetPathLength(data.BranchIndex);
            float lastLength = Joints[data.TipIndex].Segment.MaxReach;
            float endDistance = toEnd.magnitude, targetDistance = toTarget.magnitude;
            if (endDistance < length - lastLength * 0.1f || endDistance == 0f || targetDistance == 0f
                || targetDistance > endDistance || Vector3.Dot(toEnd / endDistance, toTarget / targetDistance) < 0.999f)
                return Vector3.zero;
            Vector3 direction = toTarget / targetDistance;
            Vector3 secondary = new(direction.y, direction.z, direction.x);
            int lastBone = Joints[data.TipIndex].ParentIndex;
            if (Joints[lastBone].Angular is HingeAngularConstraint hinge)
                secondary = Joints[lastBone].SolverRotation * hinge.HingeAxis;
            Vector3 offset = Vector3.Cross(direction, secondary);
            if (offset.sqrMagnitude < 1e-10f)
                offset = Vector3.Cross(direction, Mathf.Abs(direction.x) < 0.9f ? Vector3.right : Vector3.up);
            return offset * lastLength * 0.5f;
        }

        private float MeasureReachable(out bool reached)
        {
            reached = true;
            float largest = 0f;
            for (int c = 0; c < _chains.Count; c++)
            {
                if (!_goals[c].Active) continue;
                float error = Vector3.Distance(Joints[_tree.Chains[c].TipIndex].SolverPosition, _goals[c].Reachable);
                reached &= error <= _goals[c].Tolerance;
                largest = Mathf.Max(largest, error);
            }
            return largest;
        }

        protected override bool IsConverged(in SolveResult result) => _converged;

        protected override float ComputeActualPositionError()
        {
            float largest = 0f;
            for (int c = 0; c < _chains.Count; c++)
                if (_chains[c].Enabled && _chains[c].Target != null)
                    largest = Mathf.Max(largest, Vector3.Distance(_tree.Joints[_tree.Chains[c].TipIndex].position, _chains[c].Target.position));
            return largest;
        }

        private void RestoreRestPose()
        {
            // Topology edits write transforms directly, outside Solve. Restore removed joints too,
            // so removing and later adding a branch cannot capture its solved pose as the new rest pose.
            for (int i = 0; i < Joints.Length; i++)
            {
                SolverJoint joint = Joints[i];
                if (joint.Transform == null) continue;
                Transform parent = Chain.GetSetupParentTransform(i);
                Quaternion parentRotation = parent != null ? parent.rotation : Quaternion.identity;
                if (i > 0) joint.Transform.position = parent.position + parentRotation * joint.RestLocalOffset;
                joint.Transform.rotation = parentRotation * joint.RestLocalRotation;
            }
        }
    }
}
