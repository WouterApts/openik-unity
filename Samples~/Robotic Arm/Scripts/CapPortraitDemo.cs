using System.Collections;
using UnityEngine;

namespace OpenIK.Showcase
{
    /// <summary>Paints the reference portrait using the arm's normal IK and bucket pickup motions.</summary>
    public class CapPortraitDemo : MonoBehaviour
    {
        [SerializeField] private PaintingTargetController painter;
        [SerializeField] private CanvasPainter canvas;
        [Tooltip("Red, black, blue, golden yellow, in that order.")]
        [SerializeField] private PaintBucket[] buckets = new PaintBucket[4];
        [SerializeField] private bool playOnStart;
        [SerializeField, Min(0.05f)] private float drawingSpeed = 2.6f;

        public bool IsRunning => _routine != null;
        public string Status { get; private set; } = "Free paint · P to draw portrait";
        private Coroutine _routine;
        private bool _motionFailed;
        private static readonly string[] ColorNames = { "Red", "Black", "Blue", "Golden yellow" };

        private void Start() { if (playOnStart) Play(); }

        private void Update()
        {
            if (Input.GetKeyDown(KeyCode.P) && !IsRunning) Play();
            if (Input.GetKeyDown(KeyCode.Escape)) Stop();
        }

        [ContextMenu("Play portrait demo")]
        public void Play()
        {
            if (!Application.isPlaying || IsRunning || painter == null || canvas == null || painter.IsPaintPickerActive) return;
            if (buckets == null || buckets.Length != 4 || System.Array.Exists(buckets, bucket => bucket == null))
            {
                Debug.LogError("The portrait demo needs red, black, blue, and gold buckets.", this);
                return;
            }
            _motionFailed = false;
            canvas.Clear();
            painter.BeginDemoControl();
            _routine = StartCoroutine(Draw());
        }

        public void Stop()
        {
            if (_routine != null) StopCoroutine(_routine);
            _routine = null;
            if (painter != null) painter.EndDemoControl();
            Status = "Free paint · P to replay";
        }

        private void OnDisable() { Stop(); }

        private IEnumerator Draw()
        {
            // Let Start finish assigning the coroutine before any early exit.
            yield return null;
            var strokes = CapPortraitDrawing.Create();
            int color = -1;
            for (int i = 0; i < strokes.Count; i++)
            {
                var stroke = strokes[i];
                if (stroke.ColorIndex != color)
                {
                    color = stroke.ColorIndex;
                    Status = "Picking up " + ColorNames[color].ToLowerInvariant() + " paint";
                    if (!painter.BeginPaintPickup(buckets[color])) { Fail("The arm is busy."); yield break; }
                    while (painter.IsPaintPickerActive) yield return null;
                    if (painter.BrushColor != buckets[color].PaintColor)
                    {
                        Fail("The brush could not reach the " + ColorNames[color].ToLowerInvariant() + " bucket.");
                        yield break;
                    }
                }

                Status = ColorNames[color] + " · Stroke " + (i + 1) + " / " + strokes.Count;
                var points = new Vector3[stroke.Points.Count];
                for (int j = 0; j < points.Length; j++) points[j] = canvas.GetDrawingSurfacePoint(stroke.Points[j]);
                float radius = painter.BrushRadius;
                yield return Travel(points[0], painter.HoverDistance, 2.5f, radius);
                yield return Travel(points[0], 0f, 0.8f, radius);
                if (_motionFailed) { Fail("The brush could not reach the canvas."); yield break; }

                Vector3 cursor = points[0];
                int next = 1;
                while (next < points.Length)
                {
                    float distance = drawingSpeed * Time.deltaTime;
                    while (next < points.Length && distance > 0f)
                    {
                        float remaining = Vector3.Distance(cursor, points[next]);
                        if (remaining <= distance) { cursor = points[next++]; distance -= remaining; }
                        else { cursor = Vector3.MoveTowards(cursor, points[next], distance); distance = 0f; }
                    }
                    painter.SetDemoTarget(cursor, true, radius);
                    yield return null;
                }
                // Finish at the actual brush, so every closed outline joins cleanly.
                yield return Settle();
                if (_motionFailed) { Fail("The brush lost contact with the drawing."); yield break; }
                yield return Travel(cursor, painter.HoverDistance, 0.8f, radius);
            }
            Status = "Returning to idle";
            if (!painter.ReturnDemoToIdle()) { Fail("Assign an idle target to the painter."); yield break; }
            yield return Settle();
            if (_motionFailed) { Fail("The brush could not reach idle."); yield break; }
            _routine = null;
            Status = "Portrait complete · Waiting at idle";
        }

        private IEnumerator Travel(Vector3 point, float lift, float speed, float radius)
        {
            Vector3 end = point - canvas.transform.forward * lift;
            Vector3 cursor = painter.TargetPosition;
            while (Vector3.Distance(cursor, end) > 0.001f)
            {
                cursor = Vector3.MoveTowards(cursor, end, speed * Time.deltaTime);
                painter.SetDemoTarget(cursor, false, radius);
                yield return null;
            }
            painter.SetDemoTarget(end, false, radius);
            yield return Settle();
        }

        private IEnumerator Settle()
        {
            float elapsed = 0f;
            while (!painter.DemoTargetReached && elapsed < 4f)
            {
                elapsed += Time.deltaTime;
                yield return null;
            }
            _motionFailed |= !painter.DemoTargetReached;
            yield return new WaitForSeconds(0.1f);
        }

        private void Fail(string reason)
        {
            painter.EndDemoControl();
            _routine = null;
            Status = reason + " · P to retry";
            Debug.LogWarning("Portrait demo: " + reason, this);
        }

        private void OnGUI()
        {
            GUILayout.BeginArea(new Rect(16, 16, 300, 118), GUI.skin.box);
            GUILayout.Label("CAP & MUSTACHE · PAINTER DEMO");
            GUILayout.Label(Status);
            if (IsRunning)
            {
                if (GUILayout.Button("Stop demo / free paint  [Esc]")) Stop();
            }
            else
            {
                if (GUILayout.Button("Draw portrait  [P]")) Play();
                if (painter != null && painter.IsDemoControlled && GUILayout.Button("Free paint  [Esc]")) Stop();
            }
            GUILayout.Label("Free paint: drag to draw · R to clear");
            GUILayout.EndArea();
        }
    }
}
