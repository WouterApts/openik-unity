using UnityEngine;

namespace OpenIK.Showcase
{
    /// <summary>
    /// Orbits around a fixed pivot point. Right-mouse-drag to rotate.
    /// </summary>
    public class OrbitCamera : MonoBehaviour
    {
        [SerializeField] private Transform pivot;
        [SerializeField] private float distance = 8f;

        [Header("Mouse Look")]
        [SerializeField] private float mouseSensitivity = 3f;
        [SerializeField] private float minPitch = -30f;
        [SerializeField] private float maxPitch = 70f;
        [SerializeField] private float initialYaw;
        [SerializeField] private float initialPitch = 20f;

        [Header("Zoom")]
        [SerializeField] private float zoomSensitivity = 2f;
        [SerializeField] private float minDistance = 2f;
        [SerializeField] private float maxDistance = 20f;

        [Header("Smoothing")]
        [SerializeField] private float positionSmooth = 10f;
        [SerializeField] private float rotationSmooth = 12f;

        private float _yaw;
        private float _pitch;

        private void Awake()
        {
            _yaw      = initialYaw;
            _pitch    = initialPitch;
            distance  = Mathf.Clamp(distance, minDistance, maxDistance);
        }

        private void Update()
        {
            if (Input.GetMouseButton(1))
            {
                Vector2 delta = new Vector2(Input.GetAxis("Mouse X"), Input.GetAxis("Mouse Y"));
                _yaw   += delta.x * mouseSensitivity;
                _pitch -= delta.y * mouseSensitivity;
                _pitch  = Mathf.Clamp(_pitch, minPitch, maxPitch);
            }

            float scroll = Input.mouseScrollDelta.y;
            if (scroll != 0f)
                distance = Mathf.Clamp(distance - scroll * zoomSensitivity, minDistance, maxDistance);
        }

        private void LateUpdate()
        {
            if (pivot == null) return;

            Quaternion orbitRot = Quaternion.AngleAxis(_yaw, Vector3.up)
                                * Quaternion.AngleAxis(_pitch, Vector3.right);

            Vector3 desiredPos = pivot.position + orbitRot * (Vector3.back * distance);

            float posT = 1f - Mathf.Exp(-positionSmooth * Time.deltaTime);
            float rotT = 1f - Mathf.Exp(-rotationSmooth * Time.deltaTime);

            transform.position = Vector3.Lerp(transform.position, desiredPos, posT);

            Vector3 lookDir = pivot.position - transform.position;
            if (lookDir.sqrMagnitude < 0.0001f) return;

            Quaternion desiredRot = Quaternion.LookRotation(lookDir.normalized, Vector3.up);
            transform.rotation = Quaternion.Slerp(transform.rotation, desiredRot, rotT);
        }
    }
}
