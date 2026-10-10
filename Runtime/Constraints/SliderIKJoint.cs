using UnityEngine;

namespace OpenIK
{
    /// <summary>
    /// Lets a joint slide along one axis between a minimum and a maximum travel, without rotating
    /// relative to its IK parent.
    /// </summary>
    public class SliderIKJoint : ConstrainedJoint
    {
        [Header("Slider Axis")]
        [Tooltip("The local-space axis along which the joint can translate.")]
        public Vector3 slideAxis = Vector3.forward;

        [Header("Slider Limits")]
        [Tooltip("Minimum signed travel along the slider axis, measured from the authored rest position.")]
        public float minLength = 0f;
        [Tooltip("Maximum signed travel along the slider axis, measured from the authored rest position.")]
        public float maxLength = 1f;

        public Vector3 SlideAxisNormalized { get; private set; } = Vector3.forward;
        public float MinLength { get; private set; }
        public float MaxLength { get; private set; }

        private Vector3 _cachedSlideAxis;
        private float _cachedMinLength;
        private float _cachedMaxLength;

        public override JointMotionSupport MotionSupport => JointMotionSupport.Linear;

        // Recomputes the normalized axis and the ordered limits when the axis or a limit changes.
        public override void UpdateConstraints()
        {
            if (_cachedSlideAxis != slideAxis ||
                !Mathf.Approximately(_cachedMinLength, minLength) ||
                !Mathf.Approximately(_cachedMaxLength, maxLength))
            {
                _cachedSlideAxis = slideAxis;
                _cachedMinLength = minLength;
                _cachedMaxLength = maxLength;

                SlideAxisNormalized = slideAxis.sqrMagnitude > 1e-8f
                    ? slideAxis.normalized
                    : Vector3.forward;

                MinLength = Mathf.Min(minLength, maxLength);
                MaxLength = Mathf.Max(minLength, maxLength);
            }
        }

        public override ISegmentConstraint CreateSegmentConstraint(in ISegmentConstraint.SetupData setupData)
        {
            return new SliderSegmentConstraint(setupData, BuildSegmentConfig());
        }

        public override IAngularConstraint CreateAngularConstraint(in IAngularConstraint.SetupData setupData)
        {
            return new FixedAngularConstraint();
        }

        public override void ApplySegmentConfig(ISegmentConstraint segment)
        {
            if (segment is SliderSegmentConstraint slider)
                slider.ApplyConfig(BuildSegmentConfig());
        }

        private SliderSegmentConstraint.Config BuildSegmentConfig()
        {
            return new SliderSegmentConstraint.Config(SlideAxisNormalized, MinLength, MaxLength);
        }
    }
}
