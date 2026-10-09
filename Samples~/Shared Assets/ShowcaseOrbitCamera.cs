using UnityEngine;

namespace OpenIK.Showcase
{
    /// <summary>
    /// Runtime camera that slowly orbits around a target for showcase recordings.
    /// </summary>
    [ExecuteAlways]
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Camera))]
    public class ShowcaseOrbitCamera : MonoBehaviour
    {
        [SerializeField] private Transform target;
        [SerializeField] private Vector3 targetOffset = new Vector3(0f, 1.5f, 0f);
        [SerializeField] private bool offsetInTargetSpace;

        [Header("Orbit")]
        [SerializeField, Min(0.1f)] private float distance = 8f;
        [SerializeField, Range(-85f, 85f)] private float pitchAngle = 18f;
        [SerializeField] private float initialYaw;
        [SerializeField] private float orbitSpeedDegreesPerSecond = 8f;

        [Header("Smoothing")]
        [SerializeField, Min(0f)] private float positionSmooth = 0f;
        [SerializeField, Min(0f)] private float rotationSmooth = 0f;

        private float yaw;
        private bool hasPlacedCamera;

        public Transform Target
        {
            get => target;
            set => target = value;
        }

        private void Reset()
        {
            Camera cameraComponent = GetComponent<Camera>();
            cameraComponent.fieldOfView = 40f;
            cameraComponent.nearClipPlane = 0.1f;
            cameraComponent.farClipPlane = 100f;
        }

        private void OnEnable()
        {
            yaw = initialYaw;
            hasPlacedCamera = false;
            PlaceCamera(1f);
        }

        private void OnValidate()
        {
            distance = Mathf.Max(0.1f, distance);
            pitchAngle = Mathf.Clamp(pitchAngle, -85f, 85f);
            positionSmooth = Mathf.Max(0f, positionSmooth);
            rotationSmooth = Mathf.Max(0f, rotationSmooth);

            if (!Application.isPlaying)
            {
                yaw = initialYaw;
                hasPlacedCamera = false;
                PlaceCamera(1f);
            }
        }

        private void LateUpdate()
        {
            if (target == null)
                return;

            if (Application.isPlaying)
                yaw += orbitSpeedDegreesPerSecond * Time.deltaTime;
            else
                yaw = initialYaw;

            PlaceCamera(Time.deltaTime);
        }

        public void SnapToInitialPosition()
        {
            yaw = initialYaw;
            hasPlacedCamera = false;
            PlaceCamera(1f);
        }

        private void PlaceCamera(float deltaTime)
        {
            if (target == null)
                return;

            Vector3 pivot = GetPivotPosition();
            Quaternion orbitRotation = Quaternion.AngleAxis(yaw, Vector3.up)
                                       * Quaternion.AngleAxis(pitchAngle, Vector3.right);
            Vector3 desiredPosition = pivot + orbitRotation * (Vector3.back * distance);

            Vector3 lookDirection = pivot - desiredPosition;
            if (lookDirection.sqrMagnitude < 0.0001f)
                return;

            Quaternion desiredRotation = Quaternion.LookRotation(lookDirection.normalized, Vector3.up);

            if (!hasPlacedCamera)
            {
                transform.SetPositionAndRotation(desiredPosition, desiredRotation);
                hasPlacedCamera = true;
                return;
            }

            if (positionSmooth > 0f)
            {
                float positionT = 1f - Mathf.Exp(-positionSmooth * deltaTime);
                transform.position = Vector3.Lerp(transform.position, desiredPosition, positionT);
            }
            else
            {
                transform.position = desiredPosition;
            }

            if (rotationSmooth > 0f)
            {
                Vector3 lookDirectionFromCurrentPosition = pivot - transform.position;
                if (lookDirectionFromCurrentPosition.sqrMagnitude < 0.0001f)
                    return;

                desiredRotation = Quaternion.LookRotation(lookDirectionFromCurrentPosition.normalized, Vector3.up);
                float rotationT = 1f - Mathf.Exp(-rotationSmooth * deltaTime);
                transform.rotation = Quaternion.Slerp(transform.rotation, desiredRotation, rotationT);
            }
            else
            {
                transform.rotation = desiredRotation;
            }
        }

        private Vector3 GetPivotPosition()
        {
            Vector3 offset = offsetInTargetSpace ? target.TransformVector(targetOffset) : targetOffset;
            return target.position + offset;
        }
    }
}
