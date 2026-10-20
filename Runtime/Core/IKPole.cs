using System;
using UnityEngine;

namespace OpenIK
{
    /// <summary>
    /// Makes one chain joint, such as an elbow or knee, bend toward a pole target while the solver
    /// solves the whole chain.
    /// </summary>
    /// <remarks>
    /// The pole sets a direction, not a position: the bend joint turns around the line through its
    /// section's start and end joints until it faces the pole. Joint limits always take priority.
    /// </remarks>
    [Serializable]
    public sealed class IKPole
    {
        [Tooltip("The chain joint that bends toward the pole, such as an elbow or knee. Must be a middle joint of the chain.")]
        [SerializeField] private Transform bendJoint;

        [Tooltip("The bend joint turns toward this transform. Only its direction matters, not its distance. Leave empty to turn the pole off.")]
        [SerializeField] private Transform target;

        [Tooltip("How far the bend turns from its incoming direction toward the pole. Zero turns the pole off.")]
        [Range(0f, 1f)]
        [SerializeField] private float weight = 1f;

        [NonSerialized] private bool isLimited;

        public IKPole() { }

        /// <summary>Creates a pole for a bend joint.</summary>
        /// <param name="bendJoint">The chain joint that bends toward the pole.</param>
        /// <param name="target">The transform the bend joint turns toward.</param>
        /// <param name="weight">How far the bend turns toward the pole, from 0 to 1.</param>
        public IKPole(Transform bendJoint, Transform target, float weight = 1f)
        {
            this.bendJoint = bendJoint;
            this.target = target;
            this.weight = Mathf.Clamp01(weight);
        }

        /// <summary>The chain joint that bends toward the pole.</summary>
        public Transform BendJoint
        {
            get => bendJoint;
            set => bendJoint = value;
        }

        /// <summary>The transform the bend joint turns toward. The pole is off while it is null.</summary>
        public Transform Target
        {
            get => target;
            set => target = value;
        }

        /// <summary>How far the bend turns from its incoming direction toward the pole, from 0 to 1.</summary>
        public float Weight
        {
            get => weight;
            set => weight = Mathf.Clamp01(value);
        }

        /// <summary>
        /// True when joint limits removed more than one degree of this pole's turn in the latest
        /// solve, so the bend joint may not face the pole.
        /// </summary>
        public bool IsLimited
        {
            get => isLimited;
            internal set => isLimited = value;
        }
    }
}
