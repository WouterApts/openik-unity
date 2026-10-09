using UnityEngine;

namespace OpenIK.Showcase
{
    /// <summary>
    /// Manages a runtime <see cref="Texture2D"/> that represents the canvas painting surface.
    ///
    /// <para>
    /// On Awake the component creates a white texture, assigns it to the canvas material's
    /// <c>_BaseMap</c> slot (URP Lit), and caches the mesh bounds for UV projection.
    /// Call <see cref="Paint"/> each frame while the brush is in contact with the canvas
    /// to stamp a circular stroke at the given world position.
    /// </para>
    ///
    /// <para>
    /// <b>UV conversion note:</b> The canvas MeshCollider is convex, so
    /// <c>RaycastHit.textureCoord</c> always returns zero. UVs are instead derived by
    /// transforming the hit point into the canvas's local space and normalising against
    /// the mesh's local bounding box extents.
    /// </para>
    ///
    /// <para>
    /// <b>Scene setup:</b><br/>
    /// Add this component to the Canvas GameObject. The <see cref="MeshRenderer"/> and
    /// <see cref="MeshFilter"/> references are resolved automatically in <c>Awake</c> if
    /// left unassigned.
    /// </para>
    /// </summary>
    public class CanvasPainter : MonoBehaviour
    {
        // ── Settings ─────────────────────────────────────────────────────────────

        [Header("Texture")]
        [Tooltip("Width of the canvas render texture in pixels.")]
        [SerializeField] private int textureWidth = 512;

        [Tooltip("Height of the canvas render texture in pixels.")]
        [SerializeField] private int textureHeight = 512;

        [Tooltip("Flip the horizontal (U) axis if paint appears left-right mirrored.")]
        [SerializeField] private bool flipU = false;

        [Tooltip("Flip the vertical (V) axis if paint appears upside-down.")]
        [SerializeField] private bool flipV = true;

        // ── References ────────────────────────────────────────────────────────────

        [Header("References")]
        [Tooltip("MeshRenderer on the canvas. Auto-resolved from this GameObject if null.")]
        [SerializeField] private MeshRenderer canvasRenderer;

        [Tooltip("MeshFilter on the canvas. Auto-resolved from this GameObject if null.")]
        [SerializeField] private MeshFilter canvasMeshFilter;

        // ── Private State ─────────────────────────────────────────────────────────

        private Texture2D _canvasTexture;
        private Color[]   _pixels;
        private Bounds    _localBounds;
        private bool _textureDirty;

        private void LateUpdate()
        {
            if (!_textureDirty) return;
            _canvasTexture.SetPixels(_pixels);
            _canvasTexture.Apply();
            _textureDirty = false;
        }

        /// <summary>Upright canvas coordinates: (0, 0) is the upper-left corner.</summary>
        public Vector3 GetDrawingSurfacePoint(Vector2 point)
        {
            Vector3 up = Vector3.ProjectOnPlane(Vector3.up, transform.forward).normalized;
            Vector3 right = Vector3.Cross(up, transform.forward).normalized;
            float left = float.PositiveInfinity, bottom = float.PositiveInfinity;
            float rightEdge = float.NegativeInfinity, top = float.NegativeInfinity;
            // Imported meshes need not retain CPU-readable vertices in a player.
            Bounds bounds = canvasMeshFilter.sharedMesh.bounds;
            for (int i = 0; i < 8; i++)
            {
                Vector3 vertex = bounds.center + Vector3.Scale(bounds.extents,
                    new Vector3((i & 1) == 0 ? -1f : 1f, (i & 2) == 0 ? -1f : 1f, (i & 4) == 0 ? -1f : 1f));
                Vector3 offset = transform.TransformPoint(vertex) - transform.position;
                float x = Vector3.Dot(offset, right), y = Vector3.Dot(offset, up);
                left = Mathf.Min(left, x); rightEdge = Mathf.Max(rightEdge, x);
                bottom = Mathf.Min(bottom, y); top = Mathf.Max(top, y);
            }
            Vector3 projected = transform.position + right * Mathf.Lerp(left, rightEdge, point.x)
                + up * Mathf.Lerp(top, bottom, point.y);
            Ray ray = new Ray(projected - transform.forward * 5f, transform.forward);
            Collider surface = GetComponent<Collider>();
            return surface != null && surface.Raycast(ray, out RaycastHit hit, 10f) ? hit.point : projected;
        }

        /// <summary>Interpolate actual brush contacts to keep strokes continuous at low frame rates.</summary>
        public void PaintLine(Vector3 from, Vector3 to, Color color, float radius)
        {
            int steps = Mathf.Max(1, Mathf.CeilToInt(Vector3.Distance(from, to) / Mathf.Max(0.001f, radius * 0.5f)));
            for (int i = 0; i <= steps; i++) Paint(Vector3.Lerp(from, to, i / (float)steps), color, radius);
        }

        // ── Unity Messages ────────────────────────────────────────────────────────

        private void Awake()
        {
            if (canvasRenderer    == null) canvasRenderer    = GetComponent<MeshRenderer>();
            if (canvasMeshFilter  == null) canvasMeshFilter  = GetComponent<MeshFilter>();

            if (canvasRenderer == null || canvasMeshFilter == null)
            {
                Debug.LogError($"{nameof(CanvasPainter)} on '{name}': " +
                               "MeshRenderer or MeshFilter not found. Painting disabled.");
                enabled = false;
                return;
            }

            _localBounds = canvasMeshFilter.sharedMesh.bounds;

            // Create the writable texture and fill it white.
            _canvasTexture = new Texture2D(textureWidth, textureHeight, TextureFormat.RGBA32, false);
            _pixels = new Color[textureWidth * textureHeight];
            for (int i = 0; i < _pixels.Length; i++)
                _pixels[i] = Color.white;
            _canvasTexture.SetPixels(_pixels);
            _canvasTexture.Apply();

            // canvasRenderer.material creates a per-instance copy — the asset is not modified.
            canvasRenderer.material.SetTexture("_BaseMap", _canvasTexture);
        }

        private void OnDestroy()
        {
            if (_canvasTexture != null)
                Destroy(_canvasTexture);
        }

        // ── Public API ────────────────────────────────────────────────────────────

        /// <summary>
        /// Stamps a circular brush stroke on the canvas at the given world position.
        /// </summary>
        /// <param name="worldPos">World-space point on the canvas surface (from a raycast hit).</param>
        /// <param name="brushColor">Color to paint.</param>
        /// <param name="brushRadius">Brush radius in world units.</param>
        public void Paint(Vector3 worldPos, Color brushColor, float brushRadius)
        {
            // ── 1. World → local space ───────────────────────────────────────────
            Vector3 localHit = transform.InverseTransformPoint(worldPos);

            // ── 2. Normalise to [0, 1] UV using mesh bounding box ────────────────
            float u = (localHit.x - _localBounds.min.x) / _localBounds.size.x;
            float v = (localHit.y - _localBounds.min.y) / _localBounds.size.y;
            if (flipU) u = 1f - u;
            if (flipV) v = 1f - v;

            // Guard against precision drift near edges or off-surface hits.
            if (u < 0f || u > 1f || v < 0f || v > 1f) return;

            // ── 3. UV → pixel centre ─────────────────────────────────────────────
            int centerX = Mathf.RoundToInt(u * (textureWidth  - 1));
            int centerY = Mathf.RoundToInt(v * (textureHeight - 1));

            // ── 4. World-space radius → pixel radius ─────────────────────────────
            // The canvas local X extent, scaled to world space, maps to textureWidth pixels.
            float worldExtentX = _localBounds.size.x * transform.lossyScale.x;
            int pixelRadius = Mathf.Max(1, Mathf.RoundToInt(brushRadius / worldExtentX * textureWidth));

            // ── 5. Stamp filled circle ───────────────────────────────────────────
            int r2 = pixelRadius * pixelRadius;
            for (int dy = -pixelRadius; dy <= pixelRadius; dy++)
            {
                for (int dx = -pixelRadius; dx <= pixelRadius; dx++)
                {
                    if (dx * dx + dy * dy > r2) continue;

                    int px = centerX + dx;
                    int py = centerY + dy;

                    if (px < 0 || px >= textureWidth || py < 0 || py >= textureHeight) continue;

                    _pixels[py * textureWidth + px] = brushColor;
                }
            }

            // ── 6. Push to GPU ───────────────────────────────────────────────────
            _textureDirty = true;
        }

        /// <summary>Resets the canvas to a blank white state.</summary>
        public void Clear()
        {
            for (int i = 0; i < _pixels.Length; i++)
                _pixels[i] = Color.white;
            _canvasTexture.SetPixels(_pixels);
            _canvasTexture.Apply();
        }
    }
}
