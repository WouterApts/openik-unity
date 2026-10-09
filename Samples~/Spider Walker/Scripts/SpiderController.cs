using UnityEngine;

namespace OpenIK.Showcase
{
    /// <summary>
    /// Moves a spider body along its forward direction and keeps it grounded/oriented to terrain.
    /// Attach to the spider body root. Legs are handled by <see cref="SpiderLegStepper"/>.
    /// </summary>
    public class SpiderController : MonoBehaviour
    {
        [Header("Movement")]
        [SerializeField] private float moveSpeed = 2f;
        [SerializeField] private float turnSpeed = 90f;

        [Header("Ground Alignment")]
        [Tooltip("How quickly the body aligns to the average leg height and ground normal.")]
        [SerializeField] private float bodyAlignSpeed = 8f;
        [SerializeField] private float bodyHeightOffset = 0.5f;
        [SerializeField] private LayerMask groundLayer = ~0;

        [Header("Legs")]
        [SerializeField] private SpiderLegStepper[] legs;

        private bool _warnedMissingLegs;

        private void Awake()
        {
            EnsureLegReferences();
        }

        private void OnValidate()
        {
            EnsureLegReferences();
        }

        private void Update()
        {
            Vector2 moveInput = ReadMoveInput();

            transform.Rotate(Vector3.up, moveInput.x * turnSpeed * Time.deltaTime, Space.World);

            Vector3 forward = transform.forward;
            forward.y = 0f;
            forward.Normalize();
            transform.position += forward * (moveInput.y * moveSpeed * Time.deltaTime);
        }

        private void LateUpdate()
        {
            AlignBodyToLegs();
        }

        private void AlignBodyToLegs()
        {
            if (!TryGetAverageLegPose(out float averageFootY, out Vector3 averageNormal))
            {
                if (!_warnedMissingLegs)
                {
                    Debug.LogWarning($"{nameof(SpiderController)} on '{name}' has no valid leg references. Assign legs in the inspector or place leg steppers under this object.");
                    _warnedMissingLegs = true;
                }
                return;
            }

            Vector3 pos = transform.position;
            float targetY = averageFootY + bodyHeightOffset;
            pos.y = Mathf.Lerp(pos.y, targetY, bodyAlignSpeed * Time.deltaTime);
            transform.position = pos;

            Quaternion targetRot = Quaternion.FromToRotation(transform.up, averageNormal) * transform.rotation;
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, bodyAlignSpeed * Time.deltaTime);
        }

        private bool TryGetAverageLegPose(out float averageFootY, out Vector3 averageNormal)
        {
            averageFootY = 0f;
            averageNormal = Vector3.zero;

            if (legs == null || legs.Length == 0)
                return false;

            int validCount = 0;
            for (int i = 0; i < legs.Length; i++)
            {
                SpiderLegStepper leg = legs[i];
                if (leg == null)
                    continue;

                averageFootY += leg.CurrentFootPosition.y;
                averageNormal += leg.CurrentGroundNormal;
                validCount++;
            }

            if (validCount == 0)
                return false;

            averageFootY /= validCount;
            averageNormal = (averageNormal / validCount).normalized;

            if (averageNormal.sqrMagnitude < 0.01f)
                averageNormal = Vector3.up;

            return true;
        }

        private bool HasUsableLegs()
        {
            if (legs == null || legs.Length == 0)
                return false;

            for (int i = 0; i < legs.Length; i++)
            {
                if (legs[i] != null)
                    return true;
            }

            return false;
        }

        private static Vector2 ReadMoveInput()
        {
            return SampleInput.Move;
        }

        private void OnGUI()
        {
            GUILayout.BeginArea(new Rect(16, 16, 340, 126), GUI.skin.box);
            GUILayout.Label("SPIDER WALKER");
            GUILayout.Label("W/S or Up/Down: move");
            GUILayout.Label("A/D or Left/Right: turn");
            GUILayout.Label("Right drag: look around");
            GUILayout.Space(4f);
            GUILayout.Label("Click Game view to control");
            GUILayout.EndArea();
        }

        private void EnsureLegReferences()
        {
            if (HasUsableLegs())
                return;

            legs = GetComponentsInChildren<SpiderLegStepper>(true);
        }
    }
}
