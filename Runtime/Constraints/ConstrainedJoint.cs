using UnityEngine.Serialization;
using UnityEngine;

namespace OpenIK
{
    /// <summary>When the Scene view draws a gizmo.</summary>
    public enum GizmoDrawMode
    {
        Always,
        SelectedOnly
    }

    /// <summary>Base class for joint components that add constraints and speed limits to a chain joint.</summary>
    public abstract class ConstrainedJoint : MonoBehaviour
    {
        /// <summary>
        /// Whether the solver uses this joint's constraints and speed limits. When false, the
        /// transform stays in the chain without them.
        /// </summary>
        [FormerlySerializedAs("useConstraints")]
        [Tooltip("Use this joint's constraints and speed limits during IK solving. Turning this off keeps the transform in the chain.")]
        public bool jointIsEnabled = true;

        /// <summary>The joint's world rotation when it was initialized.</summary>
        public Quaternion InitialWorldRotation { get; private set; }

        /// <summary>The joint's world position when it was initialized.</summary>
        public Vector3 InitialWorldPosition { get; private set; }

        /// <summary>The joint's rotation at initialization, relative to its IK parent's rotation at initialization.</summary>
        public Quaternion RestPoseRotation { get; private set; }

        /// <summary>
        /// The joint's offset from its IK parent at initialization, in world units, in the IK parent's
        /// rotation frame at initialization.
        /// </summary>
        public Vector3 RestPoseOffset { get; private set; }

        private Transform _ikParentTransform;

        /// <summary>The joint before this one in the IK chain. The solver sets it during initialization.</summary>
        public Transform IKParentTransform { get => _ikParentTransform; set => _ikParentTransform = value; }

        public void Initialize()
        {
            InitialWorldRotation = transform.rotation;
            InitialWorldPosition = transform.position;
            UpdateConstraints();
        }

        /// <summary>Refreshes the cached constraint data of this joint type.</summary>
        /// <remarks>
        /// The solver calls this during initialization and before each solve. Implementations should
        /// recompute cached data only when the settings it depends on have changed.
        /// </remarks>
        public abstract void UpdateConstraints();

        public void ComputeRestPose(Quaternion parentInitialRotation)
        {
            RestPoseRotation = Quaternion.Inverse(parentInitialRotation) * InitialWorldRotation;
        }

        /// <summary>Records the rest rotation and the rest offset from the IK parent.</summary>
        /// <param name="parentInitialPosition">The IK parent's world position at initialization.</param>
        /// <param name="parentInitialRotation">The IK parent's world rotation at initialization.</param>
        public void ComputeRestPose(Vector3 parentInitialPosition, Quaternion parentInitialRotation)
        {
            ComputeRestPose(parentInitialRotation);
            RestPoseOffset = Quaternion.Inverse(parentInitialRotation) * (InitialWorldPosition - parentInitialPosition);
        }

        /// <summary>
        /// Local rotation from the transform's forward axis to the joint's constraint frame. Each joint
        /// type defines its own frame, for example around the hinge axis or the center of the swing cone.
        /// </summary>
        public virtual Quaternion LocalConstraintAxisRotation => Quaternion.identity;

        /// <summary>
        /// Returns the world rotation of the joint's rest pose. In Play mode it follows the IK parent's
        /// current rotation. In Edit mode it is the joint's own rotation.
        /// </summary>
        public Quaternion GetConstraintBaseRotation()
        {
            if (_ikParentTransform != null && Application.isPlaying)
                return _ikParentTransform.rotation * RestPoseRotation;
            return transform.rotation;
        }

        /// <summary>
        /// Returns the world position of the joint's rest pose. In Play mode it follows the IK parent's
        /// current pose. In Edit mode it is the joint's own position.
        /// </summary>
        public Vector3 GetConstraintBasePosition()
        {
            if (_ikParentTransform != null && Application.isPlaying)
                return _ikParentTransform.position + _ikParentTransform.rotation * RestPoseOffset;
            return transform.position;
        }

        [Tooltip("Choose when to draw this joint's constraint gizmos.")]
        [SerializeField] private GizmoDrawMode gizmoMode = GizmoDrawMode.SelectedOnly;
        public GizmoDrawMode GizmoMode => gizmoMode;

        /// <summary>When true, the joint moves toward the solved pose at a capped speed instead of snapping to it.</summary>
        [Tooltip("Move this joint toward the solved pose at a capped speed instead of snapping to it. " +
                 "The end effector can then lag behind the IK target. Applies whenever the solver applies its solution; " +
                 "with Static Solver Configuration enabled, changes made after startup are ignored.")]
        public bool limitSpeed;

        /// <summary>Maximum rotation speed relative to the IK parent, in degrees per second.</summary>
        [Tooltip("Maximum rotation speed relative to the IK parent, in degrees per second. " +
                 "Zero holds the joint still; its parent can still carry it.")]
        [Min(0f)] public float maxAngularSpeed = 90f;

        /// <summary>Maximum travel speed along the slide axis, in metres per second.</summary>
        [Tooltip("Maximum travel speed along the slide axis, in metres per second. " +
                 "Zero holds the slide still; its parent can still carry it.")]
        [Min(0f)] public float maxLinearSpeed = 0.5f;

        /// <summary>
        /// The kinds of motion whose speed this joint type can limit. The Inspector shows only the
        /// matching speed settings. Custom joint types return None unless their runtime constraints
        /// implement <see cref="IAngularMotionProvider"/> or <see cref="ISegmentMotionProvider"/>.
        /// </summary>
        public virtual JointMotionSupport MotionSupport => JointMotionSupport.None;

        /// <summary>Returns the current speed-limit settings.</summary>
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

        /// <summary>Copies this component's current segment settings into its runtime segment constraint.</summary>
        public virtual void ApplySegmentConfig(ISegmentConstraint segment) {}

        /// <summary>Copies this component's current angular settings into its runtime angular constraint.</summary>
        public virtual void ApplyAngularConfig(IAngularConstraint angular) {}
    }
}
