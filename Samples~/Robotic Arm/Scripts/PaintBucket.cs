using UnityEngine;

namespace OpenIK.Showcase
{
    /// <summary>
    /// A paint bucket that previews its color and starts a paint-picker sequence when clicked.
    /// </summary>
    public class PaintBucket : MonoBehaviour
    {
        private static readonly int BaseColorId = Shader.PropertyToID("_BaseColor");
        private static readonly int ColorId = Shader.PropertyToID("_Color");

        [Header("Paint")]
        [Tooltip("The color this bucket applies to the brush.")]
        [SerializeField] private Color paintColor = Color.red;

        [Tooltip("Local-space offset from the bucket origin to the dip point inside the bucket.")]
        [SerializeField] private Vector3 dipLocalOffset = new Vector3(0f, 0.9f, 0f);

        [Tooltip("Local-space height of the bucket rim above its origin.")]
        [SerializeField] private float rimLocalHeight = 1.4f;

        [Tooltip("World-space metres above the rim used for the pre-dip and post-dip waypoint.")]
        [SerializeField, Min(0.01f)] private float approachHeight = 1f;

        [Header("Animation")]
        [Tooltip("Average travel speed in metres per second when approaching and returning to idle.")]
        [SerializeField, Min(0.01f)] private float approachSpeed = 4.5f;

        [Tooltip("Average travel speed in metres per second when dipping, with eased starts and stops.")]
        [SerializeField, Min(0.01f)] private float dipSpeed = 3f;

        [Tooltip("Average travel speed in metres per second when lifting out of the paint.")]
        [SerializeField, Min(0.01f)] private float backOffSpeed = 4f;

        [Header("References")]
        [Tooltip("The PaintingTargetController that should run the bucket pickup sequence.")]
        [SerializeField] private PaintingTargetController painter;

        [Tooltip("Camera used to build the click ray. Auto-assigned to Camera.main if null.")]
        [SerializeField] private Camera mainCamera;

        [Header("Raycasting")]
        [Tooltip("LayerMask for click detection. Should include this bucket's layer.")]
        [SerializeField] private LayerMask bucketLayer = ~0;

        [Tooltip("Maximum ray distance for click detection.")]
        [SerializeField] private float clickMaxDistance = 100f;

        public Color PaintColor => paintColor;
        public float ApproachSpeed => approachSpeed;
        public float DipSpeed => dipSpeed;
        public float BackOffSpeed => backOffSpeed;

        private MaterialPropertyBlock _propertyBlock;

        private void Awake()
        {
            if (mainCamera == null)
                mainCamera = Camera.main;

            ApplyPaintColor();
        }

        private void OnValidate()
        {
            approachHeight = Mathf.Max(0.01f, approachHeight);
            approachSpeed = Mathf.Max(0.01f, approachSpeed);
            dipSpeed = Mathf.Max(0.01f, dipSpeed);
            backOffSpeed = Mathf.Max(0.01f, backOffSpeed);
            ApplyPaintColor();
        }

        private void Update()
        {
            if (!Input.GetMouseButtonDown(0))
                return;

            if (painter == null || mainCamera == null || painter.IsDemoControlled)
                return;

            Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);

            if (Physics.Raycast(ray, out RaycastHit hit, clickMaxDistance, bucketLayer) &&
                (hit.collider.transform == transform || hit.collider.transform.IsChildOf(transform)))
            {
                painter.BeginPaintPickup(this);
            }
        }

        public Vector3 GetApproachTarget()
        {
            Vector3 rimPoint = transform.TransformPoint(
                new Vector3(dipLocalOffset.x, rimLocalHeight, dipLocalOffset.z));
            return rimPoint + transform.up * approachHeight;
        }

        public Vector3 GetDipTarget()
        {
            return transform.TransformPoint(dipLocalOffset);
        }

        private void OnDrawGizmosSelected()
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(GetApproachTarget(), 0.08f);
            Gizmos.DrawLine(GetApproachTarget(), GetDipTarget());
            Gizmos.color = paintColor;
            Gizmos.DrawWireSphere(GetDipTarget(), 0.06f);
        }

        private void ApplyPaintColor()
        {
            if (_propertyBlock == null)
                _propertyBlock = new MaterialPropertyBlock();

            foreach (MeshRenderer meshRenderer in GetComponentsInChildren<MeshRenderer>())
            {
                Material[] sharedMaterials = meshRenderer.sharedMaterials;
                if (sharedMaterials == null || sharedMaterials.Length == 0)
                    continue;

                for (int i = 0; i < sharedMaterials.Length; i++)
                {
                    Material sharedMaterial = sharedMaterials[i];
                    if (sharedMaterial == null || !IsBucketTintMaterial(sharedMaterial.name))
                        continue;

                    _propertyBlock.Clear();
                    meshRenderer.GetPropertyBlock(_propertyBlock, i);

                    if (sharedMaterial.HasProperty(BaseColorId))
                        _propertyBlock.SetColor(BaseColorId, paintColor);

                    if (sharedMaterial.HasProperty(ColorId))
                        _propertyBlock.SetColor(ColorId, paintColor);

                    meshRenderer.SetPropertyBlock(_propertyBlock, i);
                }
            }
        }

        private static bool IsBucketTintMaterial(string materialName)
        {
            string baseName = materialName.Replace(" (Instance)", string.Empty);
            return baseName == "PaintMat" || baseName == "StickerMat";
        }

    }
}
