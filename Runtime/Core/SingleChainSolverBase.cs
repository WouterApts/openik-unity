using System.Collections.Generic;
using UnityEngine;

namespace OpenIK
{
    /// <summary>
    /// Base class for solvers that move one chain of joints toward one target: the FABRIK, CCD,
    /// Jacobian, and Two Bone solvers. It holds the Target, the Joints, and the Poles.
    /// </summary>
    /// <remarks>
    /// A solver derived from this class only implements <see cref="SolveChain(in IKGoal)"/>. This class
    /// checks and builds the chain, turns it toward its poles, and passes the target's pose to
    /// SolveChain. <see cref="OpenIKSolverBase"/> handles the update, the output, and the application.
    /// </remarks>
    public abstract class SingleChainSolverBase : OpenIKSolverBase
    {
        [Header("Chain")]
        [Tooltip("Transform the end effector moves toward.")]
        [SerializeField] private Transform target;

        [Tooltip("Joint transforms ordered from the root to the end effector.")]
        [SerializeField] private List<Transform> chainJoints = new();

        [Tooltip("Joints that bend toward a pole target, such as an elbow or knee.")]
        [SerializeField] private List<IKPole> poles = new();

        [Header("Solving")]
        [Tooltip("Stop iterating when the end effector is this close to the target, in world units.")]
        [Min(0f)]
        [SerializeField] private float tolerance = 0.001f;

        private readonly ChainPoles _poles = new();

        /// <summary>Transform the end effector moves toward. The solver skips solving while it is null.</summary>
        public Transform Target
        {
            get => target;
            set => target = value;
        }

        /// <summary>The authored joint transforms, in root-to-end order.</summary>
        public IReadOnlyList<Transform> ChainJoints => chainJoints;

        /// <summary>
        /// The pole entries, each making one middle joint bend toward a pole target. Pole targets and
        /// weights are read every solve. Added, removed, or re-pointed entries apply on the next solve,
        /// unless Static Solver Configuration is on.
        /// </summary>
        public IReadOnlyList<IKPole> Poles => poles;

        /// <summary>Adds a pole that makes a middle joint of the chain bend toward a target.</summary>
        /// <param name="bendJoint">The chain joint that bends toward the pole, such as an elbow or knee.</param>
        /// <param name="poleTarget">The transform the bend joint turns toward.</param>
        /// <param name="weight">How far the bend turns toward the pole, from 0 to 1.</param>
        /// <returns>The new pole entry, which scripts can keep to change its target or weight.</returns>
        public IKPole AddPole(Transform bendJoint, Transform poleTarget, float weight = 1f)
        {
            var pole = new IKPole(bendJoint, poleTarget, weight);
            poles.Add(pole);
            return pole;
        }

        /// <summary>Removes a pole entry.</summary>
        /// <returns>False when the entry does not belong to this solver.</returns>
        public bool RemovePole(IKPole pole)
        {
            return poles.Remove(pole);
        }

        /// <summary>Error at which the solver stops iterating and reports convergence.</summary>
        public float Tolerance
        {
            get => tolerance;
            set => tolerance = Mathf.Max(0f, value);
        }

        /// <summary>The exact number of chain joints the solver needs, or zero for any chain of two or more.</summary>
        protected virtual int RequiredJointCount => 0;

        /// <summary>
        /// False when the solver handles poles itself in SolveChain, so the base class does not turn
        /// the starting pose toward them first.
        /// </summary>
        protected virtual bool AppliesPolesBeforeSolve => true;

        /// <summary>
        /// Moves the chain's solver state toward the target.
        /// </summary>
        /// <remarks>
        /// When this is called, Chain holds the pose to start from and up-to-date
        /// constraints. Read and write only the <see cref="SolverJoint"/> solver state; the base class
        /// copies the result into LastSolution and applies it.
        /// </remarks>
        /// <param name="goal">The target's pose at the start of this solve.</param>
        /// <returns>The iteration count and the remaining error against <paramref name="goal"/>.</returns>
        protected abstract SolveResult SolveChain(in IKGoal goal);

        /// <summary>
        /// Turns the chain's solver state toward the active poles, as far as joint limits allow. A
        /// pole never moves the end effector. Call it inside SolveChain where the solver should
        /// correct the bend, for example once per iteration. Does nothing when no pole is active.
        /// </summary>
        protected void ApplyPoles()
        {
            _poles.Apply(Chain);
        }

        /// <summary>
        /// The direction an active pole asks a bend joint to face, for analytic solvers that place the
        /// bend directly instead of calling <see cref="ApplyPoles"/>.
        /// </summary>
        /// <param name="bendIndex">The bend joint's index in Joints.</param>
        /// <param name="origin">A point on the line the bend joint turns around, such as the limb's root.</param>
        /// <param name="axis">The normalized direction of that line.</param>
        /// <param name="direction">The direction perpendicular to the line, with the pole's Weight applied.</param>
        /// <returns>False when no active pole is on the joint, or the pole lies on the line.</returns>
        protected bool TryGetPoleDirection(int bendIndex, Vector3 origin, Vector3 axis, out Vector3 direction)
        {
            return _poles.TryGetBendDirection(Chain, bendIndex, origin, axis, out direction);
        }

        /// <summary>Compares the solve's error with Tolerance.</summary>
        protected override bool IsConverged(in SolveResult result) => result.Error <= tolerance;

        /// <summary>Measures the last chain joint against Target.</summary>
        protected override float ComputeActualPositionError()
        {
            Transform end = Chain.Joints[Chain.Count - 1].Transform;
            return Vector3.Distance(end.position, target.position);
        }

        protected sealed override bool ValidateSetup()
        {
            return SolverSetupValidation.Validate(this, SolverName, target, chainJoints, RequiredJointCount);
        }

        protected sealed override void BuildChain(SolverChain chain)
        {
            chain.Initialize(chainJoints);
            ResolvePoles(chain);
        }

        protected sealed override bool HasGoal => target != null;

        protected sealed override SolveResult SolveChain()
        {
            if (_poles.Count > 0)
            {
                _poles.Capture(Chain);

                // Also covers a moved pole while the target is already reached, when the solver's
                // iteration loop does not run.
                if (AppliesPolesBeforeSolve)
                    _poles.Apply(Chain);
            }

            return SolveChain(new IKGoal(target.position, target.rotation));
        }

        private protected sealed override void OnConfigurationRefreshed(bool rebound)
        {
            if (rebound || _poles.NeedsResolve(poles))
                ResolvePoles(Chain);
        }

        private void ResolvePoles(SolverChain chain)
        {
            _poles.Resolve(poles, chainJoints, chain, this, SolverName);
        }
    }
}
