using System;
using System.Collections.Generic;
using UnityEngine;

namespace OpenIK
{
    // The chains merged into one list of joints, with each parent before its children. The solver,
    // the inspector, and the gizmos all build it to check the setup.
    internal sealed class MultiFABRIKTree
    {
        // One chain's place in the tree. TreeIndex maps each chain joint to its tree index, starting
        // with the branch point. Blocked marks the branch points, which a pole section may not start
        // at or pass.
        internal sealed class ChainData
        {
            internal int[] TreeIndex;
            internal bool[] Blocked;
            internal int BranchIndex => TreeIndex[0];
            internal int FirstOwnIndex => TreeIndex[1];
            internal int TipIndex => TreeIndex[TreeIndex.Length - 1];
            internal readonly ChainPoles Poles = new();
        }

        internal Transform[] Joints = Array.Empty<Transform>();
        internal int[] ParentIndices = Array.Empty<int>();

        // The chain that owns each joint, or -1 for the root.
        internal int[] Owners = Array.Empty<int>();

        // The chain whose tip each joint is, or -1.
        internal int[] TipChains = Array.Empty<int>();
        internal ChainData[] Chains = Array.Empty<ChainData>();
        internal bool[] PointJunctions = Array.Empty<bool>();
        internal bool[] PointPivots = Array.Empty<bool>();
        internal bool HasPointJunctions;

        // For each joint, the chains whose tips are at or below it.
        internal int[][] SubtreeTips = Array.Empty<int[]>();

        // Sorts the chains into tree order and builds their joint mappings.
        // Reports setup problems without logging, so inspectors can use it too.
        internal bool Build(List<MultiFABRIKChain> chains, List<string> problems)
        {
            // Put the chains in an order that can be built, if not return early.
            if (!Sort(chains, problems)) return false;

            // Prepare temporary collections.
            var joints = new List<Transform>();
            var parents = new List<int>();
            var owners = new List<int>();
            var jointIndices = new Dictionary<Transform, int>();
            Chains = new ChainData[chains.Count];
            bool hasTarget = false;

            // Loop over all chains in the tree
            for (int chainIndex = 0; chainIndex < chains.Count; chainIndex++)
            {
                MultiFABRIKChain entry = chains[chainIndex];
                string label = Label(entry);
                if (entry.Joints.Count < 2)
                {
                    problems.Add($"{label}: add at least one joint after {entry.name}.");
                    continue;
                }
                hasTarget |= entry.Target != null;
                int[] treeIndices = new int[entry.Joints.Count];
                int parentIndex = -1;

                // Check all the chain's joints to make sure they are set up correctly before adding them.
                for (int chainJointIndex = 0; chainJointIndex < entry.Joints.Count; chainJointIndex++)
                {
                    Transform joint = entry.Joints[chainJointIndex];
                    if (joint == null)
                    {
                        problems.Add($"{label}: joint {chainJointIndex} is missing.");
                        break;
                    }
                    // Sorting put the chain that owns this first joint earlier, unless that chain is invalid.
                    if (chainJointIndex == 0 && jointIndices.TryGetValue(joint, out int sharedIndex))
                    {
                        treeIndices[chainJointIndex] = sharedIndex;
                        parentIndex = sharedIndex;
                        continue;
                    }
                    if (chainJointIndex == 0 && chainIndex > 0)
                    {
                        problems.Add($"{label}: {joint.name} is not in the tree; fix the chain it belongs to.");
                        break;
                    }
                    if (jointIndices.ContainsKey(joint))
                    {
                        problems.Add($"{label}: {joint.name} is already in another chain; only the joint a chain starts from may be shared.");
                        break;
                    }
                    if (chainJointIndex > 0 && !joint.IsChildOf(entry.Joints[chainJointIndex - 1]))
                    {
                        problems.Add($"{label}: {joint.name} must be below the preceding joint in the hierarchy.");
                        break;
                        
                        
                    }
                    int treeIndex = joints.Count;
                    treeIndices[chainJointIndex] = treeIndex;
                    jointIndices.Add(joint, treeIndex);
                    joints.Add(joint);
                    parents.Add(parentIndex);
                    owners.Add(treeIndex == 0 ? -1 : chainIndex);
                    parentIndex = treeIndex;
                }
                Chains[chainIndex] = new ChainData { TreeIndex = treeIndices, Blocked = new bool[treeIndices.Length] };
            }
            if (!hasTarget) problems.Add("At least one chain must have a target.");
            if (problems.Count > 0) return false;

            // A transform left out of the chains may sit between two joints and move with them, but the
            // path from a joint up to its tree parent may not pass another tree joint.
            for (int jointIndex = 1; jointIndex < joints.Count; jointIndex++)
            {
                Transform ancestor = joints[jointIndex].parent;
                while (ancestor != null && !jointIndices.ContainsKey(ancestor)) ancestor = ancestor.parent;
                if (ancestor != joints[parents[jointIndex]])
                    problems.Add($"{Label(chains[owners[jointIndex]])}: the path to {joints[jointIndex].name} crosses another tree joint.");
            }
            if (problems.Count > 0) return false;

            // Create the tree arrays and count children
            Joints = joints.ToArray();
            ParentIndices = parents.ToArray();
            Owners = owners.ToArray();
            TipChains = new int[joints.Count];
            PointJunctions = new bool[joints.Count];
            PointPivots = new bool[joints.Count];
            HasPointJunctions = false;
            var childCounts = new int[joints.Count];
            var tips = new List<int>[joints.Count];
            for (int jointIndex = 0; jointIndex < joints.Count; jointIndex++)
            {
                TipChains[jointIndex] = -1;
                tips[jointIndex] = new List<int>();
                if (jointIndex > 0) childCounts[parents[jointIndex]]++;
            }

            // Each chain sets the junction mode at its first joint. Chains that start at the same joint must use the same mode.
            var firstChainAtJunction = new int[joints.Count];
            for (int jointIndex = 0; jointIndex < joints.Count; jointIndex++) firstChainAtJunction[jointIndex] = -1;
            for (int chainIndex = 0; chainIndex < chains.Count; chainIndex++)
            {
                int junctionIndex = Chains[chainIndex].BranchIndex;
                int firstChainIndex = firstChainAtJunction[junctionIndex];
                if (firstChainIndex < 0)
                {
                    firstChainAtJunction[junctionIndex] = chainIndex;
                    PointJunctions[junctionIndex] = chains[chainIndex].Junction == MultiFABRIKJunctionMode.Point;
                }
                else if (chains[chainIndex].Junction != chains[firstChainIndex].Junction)
                    problems.Add($"{Label(chains[chainIndex])}: its {chains[chainIndex].Junction} junction at {joints[junctionIndex].name} must match {Label(chains[firstChainIndex])}, which is {chains[firstChainIndex].Junction}.");
            }

            // Validate Point junctions and their pivots
            for (int jointIndex = 0; jointIndex < joints.Count; jointIndex++)
            {
                if (!PointJunctions[jointIndex]) continue;
                HasPointJunctions = true;
                
                if (childCounts[jointIndex] == 0)
                    problems.Add($"Point junction {joints[jointIndex].name}: needs at least one outgoing pivot.");
                
                if (joints[jointIndex].GetComponent<ConstrainedJoint>() is HingeIKJoint or BallSocketIKJoint)
                    problems.Add($"Point junction {joints[jointIndex].name}: put angular limits on its independent branch pivots, not the shared attachment frame.");
                
                Vector3? pivotOffset = null;
                
                for (int childIndex = jointIndex + 1; childIndex < joints.Count; childIndex++)
                {
                    if (parents[childIndex] != jointIndex) continue;
                    PointPivots[childIndex] = true;
                    
                    Transform pivot = joints[childIndex];
                    
                    if (pivot.parent != joints[jointIndex])
                        problems.Add($"Point junction {joints[jointIndex].name}: pivot {pivot.name} must be a direct child.");
                    
                    if (pivotOffset.HasValue && !pivot.localPosition.Equals(pivotOffset.Value))
                        problems.Add($"Point junction {joints[jointIndex].name}: every pivot must have the same local offset.");
                    
                    pivotOffset ??= pivot.localPosition;
                    
                    if (pivot.GetComponent<ConstrainedJoint>() is SliderIKJoint)
                        problems.Add($"Point junction {joints[jointIndex].name}: pivot {pivot.name} cannot be a slider; put the slider before or after the attachment.");
                }
            }

            // Record tips and pole restrictions
            if (problems.Count > 0) return false;
            for (int chainIndex = 0; chainIndex < chains.Count; chainIndex++)
            {
                ChainData data = Chains[chainIndex];
                TipChains[data.TipIndex] = chainIndex;
                for (int jointIndex = data.TipIndex; jointIndex >= 0; jointIndex = parents[jointIndex]) tips[jointIndex].Add(chainIndex);
                for (int chainJointIndex = 0; chainJointIndex < data.TreeIndex.Length; chainJointIndex++)
                {
                    int jointIndex = data.TreeIndex[chainJointIndex];
                    bool startsAttachedChain = chainJointIndex == 0 && chainIndex > 0;
                    bool hasBranches = childCounts[jointIndex] > 1;
                    bool isPointJunction = PointJunctions[jointIndex];

                    data.Blocked[chainJointIndex] = startsAttachedChain || hasBranches || isPointJunction;
                }
            }
            SubtreeTips = new int[joints.Count][];
            for (int jointIndex = 0; jointIndex < joints.Count; jointIndex++) SubtreeTips[jointIndex] = tips[jointIndex].ToArray();
            return true;
        }

        // Every problem message about a chain starts with this label.
        internal static string Label(MultiFABRIKChain chain) =>
            $"Chain {(string.IsNullOrEmpty(chain.ChainName) ? chain.name : chain.ChainName)} on {chain.name}";

        // Sorts chains so each chain comes before all the chains branching from it.
        // Reports disconnected chains and circular dependencies.
        internal static bool Sort(List<MultiFABRIKChain> chains, List<string> problems)
        {
            problems.Clear();
            
            // Reject and empty chains list
            if (chains.Count == 0)
            {
                problems.Add("Add the rig's FABRIK Chains to Chains, starting with the chain on the root joint.");
                return false;
            }

            // record which chain own each joint
            var owners = new Dictionary<Transform, int>();
            for (int c = 0; c < chains.Count; c++)
                for (int j = 1; j < chains[c].Joints.Count; j++)
                    if (chains[c].Joints[j] != null) owners.TryAdd(chains[c].Joints[j], c);

            // Of the joints that start chains without belonging to one, the highest in the hierarchy
            // is the root. Ties keep list order.
            Transform root = null;
            int rootDepth = int.MaxValue;
            foreach (MultiFABRIKChain chain in chains)
            {
                Transform first = chain.transform;
                if (owners.ContainsKey(first)) continue;
                int depth = 0;
                for (Transform t = first.parent; t != null; t = t.parent) depth++;
                if (depth < rootDepth) (root, rootDepth) = (first, depth);
            }
            if (root == null)
            {
                problems.Add("No root joint: every chain starts on a joint of another chain, so the chains form a cycle. "
                    + "Start one chain at the root joint.");
                return false;
            }

            // Build the sorted chains list
            var sorted = new List<MultiFABRIKChain>(chains.Count);
            var placed = new bool[chains.Count];
            bool progress = true;
            while (progress)
            {
                progress = false;
                for (int c = 0; c < chains.Count; c++)
                {
                    if (placed[c]) continue;
                    Transform first = chains[c].transform;
                    if (first != root && (!owners.TryGetValue(first, out int owner) || !placed[owner])) continue;
                    placed[c] = true;
                    sorted.Add(chains[c]);
                    progress = true;
                    break;
                }
            }

            for (int c = 0; c < chains.Count; c++)
            {
                if (placed[c]) continue;
                Transform first = chains[c].transform;
                problems.Add(owners.ContainsKey(first)
                    ? $"{Label(chains[c])}: it starts on a joint of a chain that branches from it, which forms a cycle."
                    : $"{Label(chains[c])}: {first.name} is not a joint of another listed chain. Only the root joint, {root.name}, may start chains without belonging to one; add the chain that contains {first.name}.");
            }
            if (problems.Count > 0) return false;
            chains.Clear();
            chains.AddRange(sorted);
            return true;
        }
    }
}
