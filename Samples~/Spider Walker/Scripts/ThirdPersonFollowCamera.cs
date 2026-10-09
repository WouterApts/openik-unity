using UnityEngine;

namespace OpenIK.Showcase
{
    /// <summary>
    /// Simple third-person follow camera for the spider showcase.
    /// </summary>
    public class ThirdPersonFollowCamera : MonoBehaviour
    {
        [SerializeField] private Transform target;
        [SerializeField] private Vector3 pivotOffset = new Vector3(0f, 1.5f, 0f);
        [SerializeField] private float distance = 6f;
        [SerializeField] private float positionSmooth = 10f;
        [SerializeField] private float rotationSmooth = 12f;
        [SerializeField] private bool useTargetUp;

        [Header("Mouse Look")]
        [SerializeField] private float mouseSensitivity = 3f;
        [SerializeField] private float minPitch = -30f;
        [SerializeField] private float maxPitch = 70f;
        [SerializeField] private float initialYaw;
        [SerializeField] private float initialPitch = 20f;
        [Space]
        [SerializeField] private bool requireRightMouseToLook;

        private float yaw;
        private float pitch;

        private void Reset()
        {
            TryAutoAssignTarget();
        }

        private void Awake()
        {
            if (target == null)
                TryAutoAssignTarget();

            yaw = initialYaw;
            pitch = initialPitch;
        }

        private void Update()
        {
            Vector2 lookDelta = ReadLookDelta();
            yaw += lookDelta.x * mouseSensitivity;
            pitch -= lookDelta.y * mouseSensitivity;
            pitch = Mathf.Clamp(pitch, minPitch, maxPitch);
        }

        private Vector2 ReadLookDelta()
        {
            if (requireRightMouseToLook && !Input.GetMouseButton(1))
                return Vector2.zero;
            return new Vector2(Input.GetAxis("Mouse X"), Input.GetAxis("Mouse Y"));
        }

        private void LateUpdate()
        {
            if (target == null)
            {
                TryAutoAssignTarget();
                if (target == null) return;
            }

            float posT = 1f - Mathf.Exp(-positionSmooth * Time.deltaTime);
            float rotT = 1f - Mathf.Exp(-rotationSmooth * Time.deltaTime);

            Vector3 up = useTargetUp ? target.up : Vector3.up;
            Vector3 pivot = target.position + (useTargetUp ? target.TransformVector(pivotOffset) : pivotOffset);

            Quaternion orbitRot = Quaternion.AngleAxis(yaw, up) * Quaternion.AngleAxis(pitch, Vector3.right);
            if (useTargetUp)
            {
                Quaternion targetYaw = Quaternion.LookRotation(Vector3.ProjectOnPlane(target.forward, up), up);
                orbitRot = targetYaw * Quaternion.AngleAxis(yaw, Vector3.up) * Quaternion.AngleAxis(pitch, Vector3.right);
            }

            Vector3 desiredPos = pivot + orbitRot * (Vector3.back * distance);
            transform.position = Vector3.Lerp(transform.position, desiredPos, posT);

            Vector3 lookDir = pivot - transform.position;
            if (lookDir.sqrMagnitude < 0.0001f) return;

            Quaternion desiredRot = Quaternion.LookRotation(lookDir.normalized, up);
            transform.rotation = Quaternion.Slerp(transform.rotation, desiredRot, rotT);
        }

        private void TryAutoAssignTarget()
        {
            SpiderController controller = FindFirstObjectByType<SpiderController>();
            if (controller != null)
                target = controller.transform;
        }
    }
}
