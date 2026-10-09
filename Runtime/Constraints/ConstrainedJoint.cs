using UnityEngine.Serialization;
using UnityEngine;

namespace OpenIK
{
    public enum GizmoDrawMode
    {
        Always,
        SelectedOnly
    }

    public abstract class ConstrainedJoint : MonoBehaviour
    {
        /// Quickly toggles whether this joint component's constraints are used in IK solving.
        [FormerlySerializedAs("useConstraints")]
        [Tooltip("Use this joint's constraints and speed limits during IK solving. Turning this off keeps the transform in the chain.")]
        public bool jointIsEnabled = true;

        /// The initial Joint's transform rotation in World frame
        public Quaternion InitialWorldRotation { get; private set; }

        /// The initial Joint's transform position in World frame
        public Vector3 InitialWorldPosition { get; private set; }

        /// The joint's initial world rotation expressed in the IK parent's initial rotation frame.
        public Quaternion RestPoseRotation { get; private set; }

        /// The joint's initial offset from its IK parent, in world units, expressed in the IK parent's initial rotation frame.
        public Vector3 RestPoseOffset { get; private set; }

        /// The IK parent transform, set by the solver during initialization.
        private Transform _ikParentTransform;
        public Transform IKParentTransform { get => _ikParentTransform; set => _ikParentTransform = value; }

        public void Initialize()
        {
            InitialWorldRotation = transform.rotation;
            InitialWorldPosition = transform.position;
            UpdateConstraints();
        }

        /// <summary>
        /// Refreshes any cached constraint data needed by this joint type.
        /// </summary>
        /// <remarks>
        /// Called during initialization and before each solve. Derived classes decide which serialized
        /// values must be checked and only recompute cached constraint data when those values changed.
        /// </remarks>
        public abstract void UpdateConstraints();

        public void ComputeRestPose(Quaternion parentInitialRotation)
        {
            RestPoseRotation = Quaternion.Inverse(parentInitialRotation) * InitialWorldRotation;
        }

        /// Saves the rest rotation and the rest offset from the IK parent.
        public void ComputeRestPose(Vector3 parentInitialPosition, Quaternion parentInitialRotation)
        {
            ComputeRestPose(parentInitialRotation);
            RestPoseOffset = Quaternion.Inverse(parentInitialRotation) * (InitialWorldPosition - parentInitialPosition);
        }

        /// The local rotation that maps the transform's local forward vector to the joint's constraint-axis frame.
        /// Each joint type defines and implements its own constraint frame (hinge axis, swing cone center, etc.).
        public virtual Quaternion LocalConstraintAxisRotation => Quaternion.identity;

        /// Returns the world-space rotation representing the constraint rest frame.
        /// During play, this is derived from the parent's current rotation + rest pose.
        /// While editing, falls back to this joint's own rotation.
        public Quaternion GetConstraintBaseRotation()
        {
            if (_ikParentTransform != null && Application.isPlaying)
                return _ikParentTransform.rotation * RestPoseRotation;
            return transform.rotation;
        }

        /// Returns the world-space position of the joint's rest pose.
        /// During play, this is derived from the parent's current pose + rest pose.
        /// While editing, falls back to this joint's own position.
        public Vector3 GetConstraintBasePosition()
        {
            if (_ikParentTransform != null && Application.isPlaying)
                return _ikParentTransform.position + _ikParentTransform.rotation * RestPoseOffset;
            return transform.position;
        }

        [Tooltip("Choose when to draw this joint's constraint gizmos.")]
        [SerializeField] private GizmoDrawMode gizmoMode = GizmoDrawMode.SelectedOnly;
        public GizmoDrawMode GizmoMode => gizmoMode;

        /// Moves this joint toward the solved pose at a capped speed instead of snapping to it.
        [Tooltip("Move this joint toward the solved pose at a capped speed instead of snapping to it. " +
                 "The end effector can then lag behind the IK target. Applies in Solve And Apply, or when scripts call ApplyLastOutput in Solve Only; " +
                 "with Static Solver Configuration enabled, changes made after startup are ignored.")]
        public bool limitSpeed;

        /// Maximum rotation speed relative to the IK parent, in degrees per second.
        [Tooltip("Maximum rotation speed relative to the IK parent, in degrees per second. " +
                 "Zero holds the joint still; its parent can still carry it.")]
        [Min(0f)] public float maxAngularSpeed = 90f;

        /// Maximum travel speed along the slide axis, in metres per second.
        [Tooltip("Maximum travel speed along the slide axis, in metres per second. " +
                 "Zero holds the slide still; its parent can still carry it.")]
        [Min(0f)] public float maxLinearSpeed = 0.5f;

        /// <summary>
        /// Degrees of freedom whose speed this joint type can limit. The Inspector shows only the
        /// matching speed settings. Custom joint types return <see cref="JointMotionSupport.None"/>
        /// unless their runtime constraints implement <see cref="IAngularMotionProvider"/> or
        /// <see cref="ISegmentMotionProvider"/>.
        /// </summary>
        public virtual JointMotionSupport MotionSupport => JointMotionSupport.None;

        /// Packages the current speed-limit settings for the solver's pose applier.
        public JointMotionLimit GetMotionLimit()
        {
            return new JointMotionLimit(limitSpeed, maxAngularSpeed, maxLinearSpeed);
        }

        public virtual ISegmentConstraint CreateSegmentConstraint(in ISegmentConstraint.SetupData setupData)
        {
            return new RigidSegmentConstraint(setupData);
        }

        public virtual IAngularConstraint CreateAngularConstraint(in IAngularConstraint.SetupData setupData)
        {
            return new FreeAngularConstraint();
        }

        /// Copies this joint component's latest serialized segment settings into the bound runtime constraint.
        public virtual void ApplySegmentConfig(ISegmentConstraint segment) {}

        /// Copies this joint component's latest serialized angular settings into the bound runtime constraint.
        public virtual void ApplyAngularConfig(IAngularConstraint angular) {}
    }
}
