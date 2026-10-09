using UnityEngine;

namespace OpenIK.Showcase
{
    /// <summary>
    /// Drives the IK target for the Jacobian robot arm so it can either paint on a canvas
    /// or run a temporary paint-picker sequence when a bucket is clicked.
    /// </summary>
    /// <remarks>
    /// A pickup is two continuous trips: to the bucket (passing above it into the paint) and back
    /// (out of the paint, above the bucket, to idle). The arm stops only in the paint and at idle;
    /// the corner above the bucket is rounded instead of stopped at. The arm's joint speed limits
    /// apply during the pickup, so the brush can lag behind the IK target: each trip ends only when
    /// the solver reports that the actual brush has arrived, and the color is applied only once the
    /// brush is in the paint. A trip that times out recovers through the cached above-bucket point.
    /// While painting, the limits are switched off by default so the brush follows the mouse directly.
    /// </remarks>
    public class PaintingTargetController : MonoBehaviour
    {
        private static readonly int BaseColorId = Shader.PropertyToID("_BaseColor");
        private static readonly int ColorId = Shader.PropertyToID("_Color");

        private enum PaintPickerPhase
        {
            None,
            /// Start, above the bucket, into the paint.
            ToBucket,
            /// Out of the paint, above the bucket, to idle.
            FromBucket,
            /// Pickup cancelled: above the bucket, to idle, without paint.
            Recover
        }

        [Header("References")]
        [Tooltip("The IK target transform that JacobianIKSolver reads every frame.")]
        [SerializeField] private Transform ikTarget;

        [Tooltip("Solver that drives the arm. Its application status confirms when the actual brush reaches a waypoint. " +
                 "Auto-assigned from this GameObject if null.")]
        [SerializeField] private OpenIKSolverBase solver;

        [Tooltip("The canvas transform. Its forward axis points into the canvas surface.")]
        [SerializeField] private Transform canvas;

        [Tooltip("Camera used to build the mouse ray. Auto-assigned to Camera.main if null.")]
        [SerializeField] private Camera mainCamera;

        [Tooltip("Optional idle pose. When the mouse is not over the canvas the arm returns here.")]
        [SerializeField] private Transform idleTarget;

        [Header("Painting Settings")]
        [Tooltip("Distance in metres the target hovers away from the canvas surface while not painting.")]
        [SerializeField] private float hoverDistance = 0.05f;

        [Tooltip("Distance in metres the target sits from the canvas surface while painting.")]
        [SerializeField] private float paintDistance = 0.005f;

        [Tooltip("Lerp / Slerp speed for smooth position and rotation transitions.")]
        [SerializeField] private float smoothSpeed = 10f;

        [Tooltip("Degrees to tilt the brush toward -canvas.up for a natural painting angle.")]
        [SerializeField] private float tiltAngle = 15f;

        [Header("Canvas Painting")]
        [Tooltip("CanvasPainter component on the Canvas object.")]
        [SerializeField] private CanvasPainter canvasPainter;

        [Tooltip("The brush tip transform. Paint is applied at its projection onto the canvas plane.")]
        [SerializeField] private Transform brushTip;

        [Tooltip("Maximum distance from the brush tip to the canvas plane for paint to be applied.")]
        [SerializeField] private float paintContactDistance = 0.02f;

        [Tooltip("Shared radius of free-paint and portrait strokes, in world units.")]
        [SerializeField] private float brushRadius = 0.033f;

        [Tooltip("Starting brush color.")]
        [SerializeField] private Color brushColor = Color.black;

        [Header("Brush Visual")]
        [Tooltip("Renderer for the wet paint part of the brush tip.")]
        [SerializeField] private Renderer brushPaintRenderer;

        [Tooltip("Material slot on the wet paint renderer that should receive the selected paint color.")]
        [SerializeField] private int brushPaintMaterialIndex;

        [Header("Motion")]
        [Tooltip("Keep the arm's joint speed limits active while painting on the canvas. When off, the brush follows the mouse " +
                 "directly and the limits (joints with Limit Speed enabled) apply only to the paint pickup.")]
        [SerializeField] private bool limitSpeedWhilePainting;

        [Header("Paint Picker")]
        [Tooltip("Distance from the dip point within which the brush counts as in the paint. The color is applied only then.")]
        [SerializeField] private float paintPickerArrivalDistance = 0.05f;

        [Tooltip("Distance within which the settled brush counts as having passed a via point (above the bucket or idle). " +
                 "The arm cannot always match both the position and the brush direction there exactly.")]
        [SerializeField] private float paintPickerWaypointTolerance = 0.5f;

        [Tooltip("Degrees the brush leans away from the arm while dipping (negative leans toward it). A perfectly vertical " +
                 "brush lines the wrist up with the base's rotation axis, where small moves need sudden, large base and wrist rotations.")]
        [SerializeField] private float paintPickerDipTilt = 15f;

        [Tooltip("Pause in seconds in the paint and at idle after the brush arrives.")]
        [SerializeField] private float paintPickerSettleTime = 0.15f;

        [Tooltip("Smoothing time in seconds for the IK target during a pickup. Rounds the corner above the bucket " +
                 "so the arm flows through it instead of stopping.")]
        [SerializeField] private float paintPickerCornerSmoothing = 0.15f;

        [Tooltip("Seconds the brush may take to reach a waypoint after the target gets there before the pickup is cancelled.")]
        [SerializeField] private float paintPickerTimeout = 3f;

        [Header("Raycasting")]
        [Tooltip("LayerMask that includes the canvas collider layer.")]
        [SerializeField] private LayerMask canvasLayer = ~0;

        [Tooltip("Maximum ray distance for canvas intersection test.")]
        [SerializeField] private float raycastMaxDistance = 50f;

        private Vector3 _desiredPosition;
        private Quaternion _desiredRotation;
        private bool _hasValidTarget;
        private PaintBucket _activePaintBucket;
        private PaintPickerPhase _paintPickerPhase;
        private Vector3 _paintPickerApproachPosition;
        private Vector3 _paintPickerDipPosition;
        private Vector3 _paintPickerReturnPosition;
        private Quaternion _paintPickerReturnRotation;
        // The current trip: start, via point above the bucket, end. Times are cumulative seconds.
        private readonly Vector3[] _pathPositions = new Vector3[3];
        private readonly Quaternion[] _pathRotations = new Quaternion[3];
        private readonly float[] _pathTimes = new float[3];
        private float _pathDuration;
        private Vector3 _targetVelocity;
        private float _segmentElapsed;
        private float _arrivedTime;
        private float _returnSpeed;
        private float _dipSpeed;
        private float _backOffSpeed;
        private Color _pickupColor;
        private Quaternion _paintPickerDipRotation;
        private MaterialPropertyBlock _brushPaintPropertyBlock;
        private ConstrainedJoint[] _speedLimitedJoints = System.Array.Empty<ConstrainedJoint>();
        private bool _speedLimitsActive = true;

        public bool IsPaintPickerActive => _paintPickerPhase != PaintPickerPhase.None;
        public bool IsDemoControlled { get; private set; }
        public Color BrushColor => brushColor;
        public float BrushRadius => brushRadius;
        public Vector3 TargetPosition => ikTarget.position;
        public float HoverDistance => hoverDistance;
        public bool DemoTargetReached => brushTip != null &&
            Vector3.Distance(brushTip.position, _desiredPosition) < 0.065f;

        private bool _demoPainting;
        private float _demoBrushRadius;
        private Vector3? _previousDemoPaintPoint;

        public void BeginDemoControl()
        {
            IsDemoControlled = true;
            _demoPainting = false;
            _previousDemoPaintPoint = null;
        }

        public void SetDemoTarget(Vector3 position, bool painting, float radius)
        {
            _desiredPosition = position;
            _desiredRotation = ComputeBrushRotation();
            _demoPainting = painting;
            _demoBrushRadius = radius;
            if (!painting) _previousDemoPaintPoint = null;
        }

        public void EndDemoControl()
        {
            IsDemoControlled = false;
            _demoPainting = false;
            _previousDemoPaintPoint = null;
        }

        /// <summary>Return to the authored idle pose and retain demo control until released.</summary>
        public bool ReturnDemoToIdle()
        {
            if (idleTarget == null) return false;
            _demoPainting = false;
            _previousDemoPaintPoint = null;
            _desiredPosition = idleTarget.position;
            _desiredRotation = idleTarget.rotation;
            return true;
        }

        private void LateUpdate()
        {
            if (!IsDemoControlled || !_demoPainting || IsPaintPickerActive || !IsBrushInContact())
            {
                _previousDemoPaintPoint = null;
                return;
            }

            Vector3 point = ProjectOntoCanvas(brushTip.position);
            canvasPainter.PaintLine(_previousDemoPaintPoint ?? point, point, brushColor, _demoBrushRadius);
            _previousDemoPaintPoint = point;
        }

        private void Awake()
        {
            if (mainCamera == null)
                mainCamera = Camera.main;

            if (mainCamera == null)
            {
                Debug.LogWarning(
                    $"{nameof(PaintingTargetController)} on '{name}': No camera found. Assign mainCamera or tag a camera as MainCamera.");
            }

            if (solver == null)
                solver = GetComponent<OpenIKSolverBase>();

            // Remember which arm joints are authored with a speed limit so they can be toggled per mode.
            var limitedJoints = new System.Collections.Generic.List<ConstrainedJoint>();
            foreach (ConstrainedJoint joint in GetComponentsInChildren<ConstrainedJoint>(true))
            {
                if (joint.limitSpeed)
                    limitedJoints.Add(joint);
            }
            _speedLimitedJoints = limitedJoints.ToArray();

            if (ikTarget != null)
            {
                _desiredPosition = ikTarget.position;
                _desiredRotation = ikTarget.rotation;
            }

            ApplyBrushPaintColor();
        }

        private void OnValidate()
        {
            paintPickerArrivalDistance = Mathf.Max(0.001f, paintPickerArrivalDistance);
            paintPickerWaypointTolerance = Mathf.Max(paintPickerArrivalDistance, paintPickerWaypointTolerance);
            paintPickerSettleTime = Mathf.Max(0f, paintPickerSettleTime);
            paintPickerCornerSmoothing = Mathf.Max(0f, paintPickerCornerSmoothing);
            paintPickerDipTilt = Mathf.Clamp(paintPickerDipTilt, -60f, 60f);
            paintPickerTimeout = Mathf.Max(0.1f, paintPickerTimeout);
            brushPaintMaterialIndex = Mathf.Max(0, brushPaintMaterialIndex);
            ApplyBrushPaintColor();
        }

        private void Update()
        {
            if (ikTarget == null)
                return;

            SetJointSpeedLimitsActive(IsPaintPickerActive || limitSpeedWhilePainting);

            if (IsPaintPickerActive)
            {
                UpdatePaintPickerSequence(Time.deltaTime);
            }
            else if (IsDemoControlled)
            {
                // The demo already advances its stroke target at a controlled speed.
                // Extra target damping cuts corners and leaves gaps at strap/button joins.
                if (_demoPainting)
                    ikTarget.SetPositionAndRotation(_desiredPosition, _desiredRotation);
                else
                    ApplySmoothPose();
            }
            else
            {
                if (canvas == null || mainCamera == null)
                    return;

                ComputeDesiredTargetPose();
                ApplySmoothPose();
            }

            CheckResetInput();
        }

        private void ComputeDesiredTargetPose()
        {
            Ray mouseRay = BuildMouseRay();

            if (Physics.Raycast(mouseRay, out RaycastHit hit, raycastMaxDistance, canvasLayer))
            {
                _hasValidTarget = true;

                bool isPainting = Input.GetMouseButton(0);
                float contactOffset = isPainting ? paintDistance : hoverDistance;

                _desiredPosition = hit.point + (-canvas.forward) * contactOffset;
                _desiredRotation = ComputeBrushRotation();

                if (isPainting && canvasPainter != null && IsBrushInContact())
                {
                    Vector3 paintPoint = brushTip != null
                        ? ProjectOntoCanvas(brushTip.position)
                        : hit.point;
                    canvasPainter.Paint(paintPoint, brushColor, brushRadius);
                }
            }
            else
            {
                _hasValidTarget = false;

                if (idleTarget != null)
                {
                    _desiredPosition = idleTarget.position;
                    _desiredRotation = idleTarget.rotation;
                }
            }
        }

        private void UpdatePaintPickerSequence(float deltaTime)
        {
            // A removed bucket must not leave the arm stranded in the pickup pose.
            if (_activePaintBucket == null && _paintPickerPhase == PaintPickerPhase.ToBucket)
                BeginRecovery("the bucket was removed");

            deltaTime = Mathf.Max(0f, deltaTime);
            _segmentElapsed += deltaTime;
            float progress = Mathf.Clamp01(_segmentElapsed / _pathDuration);
            // Quintic easing gives the trip zero velocity and acceleration at both ends.
            float eased = progress * progress * progress * (progress * (progress * 6f - 15f) + 10f);
            EvaluatePath(eased * _pathTimes[2], out Vector3 pathPosition, out Quaternion pathRotation);

            // The smoothed target rounds the corner above the bucket instead of stopping there.
            if (paintPickerCornerSmoothing > 0f && deltaTime > 0f)
            {
                ikTarget.SetPositionAndRotation(
                    Vector3.SmoothDamp(ikTarget.position, pathPosition, ref _targetVelocity,
                        paintPickerCornerSmoothing, Mathf.Infinity, deltaTime),
                    Quaternion.Slerp(ikTarget.rotation, pathRotation,
                        1f - Mathf.Exp(-deltaTime / paintPickerCornerSmoothing)));
            }
            else
            {
                ikTarget.SetPositionAndRotation(pathPosition, pathRotation);
            }

            if (progress < 1f)
                return;

            // The speed-limited arm may still be catching up with the target.
            float waitTime = _segmentElapsed - _pathDuration;
            if (!HasBrushArrived())
            {
                _arrivedTime = -1f;
                if (waitTime >= paintPickerTimeout)
                    OnWaypointTimedOut();
                return;
            }

            if (_arrivedTime < 0f)
                _arrivedTime = _segmentElapsed;
            if (_segmentElapsed - _arrivedTime < paintPickerSettleTime)
                return;

            switch (_paintPickerPhase)
            {
                case PaintPickerPhase.ToBucket:
                    // The brush is confirmed in the paint.
                    SetBrushColor(_pickupColor);
                    StartPaintPickerPath(PaintPickerPhase.FromBucket,
                        _paintPickerApproachPosition, _paintPickerDipRotation, _backOffSpeed,
                        _paintPickerReturnPosition, _paintPickerReturnRotation, _returnSpeed);
                    break;

                case PaintPickerPhase.FromBucket:
                case PaintPickerPhase.Recover:
                    EndPaintPickerSequence();
                    break;
            }
        }

        /// Samples the current trip at <paramref name="time"/> seconds along its unsmoothed timeline.
        private void EvaluatePath(float time, out Vector3 position, out Quaternion rotation)
        {
            int part = time <= _pathTimes[1] ? 0 : 1;
            float partDuration = _pathTimes[part + 1] - _pathTimes[part];
            float t = partDuration > 1e-5f ? Mathf.Clamp01((time - _pathTimes[part]) / partDuration) : 1f;
            position = Vector3.Lerp(_pathPositions[part], _pathPositions[part + 1], t);
            rotation = Quaternion.Slerp(_pathRotations[part], _pathRotations[part + 1], t);
        }

        /// <summary>
        /// True when the actual brush, not just the IK target, has finished the current trip: the
        /// target has settled at the trip's end, the speed-limited joints have caught up with the
        /// solve, and the brush is in the paint for the dip or near enough to idle.
        /// </summary>
        private bool HasBrushArrived()
        {
            if (Vector3.Distance(ikTarget.position, _desiredPosition) > 0.01f)
                return false;

            float tolerance = _paintPickerPhase == PaintPickerPhase.ToBucket
                ? paintPickerArrivalDistance
                : paintPickerWaypointTolerance;

            if (solver != null && solver.ApplicationStatus.Applied)
            {
                IKApplicationStatus status = solver.ApplicationStatus;
                return status.ReachedSolution && status.PositionError <= tolerance;
            }

            return brushTip != null && Vector3.Distance(brushTip.position, _desiredPosition) <= tolerance;
        }

        private string DescribeArrivalError()
        {
            if (solver == null || !solver.ApplicationStatus.Applied)
                return "no solver status";

            IKApplicationStatus status = solver.ApplicationStatus;
            return status.HasOrientationError
                ? $"{status.PositionError:F2} m and {status.OrientationError:F0}° away"
                : $"{status.PositionError:F2} m away";
        }

        private void OnWaypointTimedOut()
        {
            if (_paintPickerPhase == PaintPickerPhase.ToBucket)
            {
                BeginRecovery($"the brush did not reach the paint ({DescribeArrivalError()})");
                return;
            }

            // Returning to idle failed as well: stop instead of retrying forever.
            Debug.LogWarning($"{nameof(PaintingTargetController)} on '{name}': the paint pickup could not return to idle ({DescribeArrivalError()}) and was stopped.", this);
            HoldCurrentBrushPose();
            EndPaintPickerSequence();
        }

        /// Cancels the pickup without granting paint and returns to idle through the cached above-bucket point.
        private void BeginRecovery(string reason)
        {
            Debug.LogWarning($"{nameof(PaintingTargetController)} on '{name}': paint pickup cancelled because {reason}.", this);
            _activePaintBucket = null;
            StartPaintPickerPath(PaintPickerPhase.Recover,
                _paintPickerApproachPosition, _paintPickerDipRotation, _backOffSpeed,
                _paintPickerReturnPosition, _paintPickerReturnRotation, _returnSpeed);
        }

        private void HoldCurrentBrushPose()
        {
            if (brushTip == null)
                return;

            ikTarget.position = brushTip.position;
            _desiredPosition = brushTip.position;
            _desiredRotation = ikTarget.rotation;
        }

        /// <summary>
        /// Starts a trip from the current target pose through <paramref name="viaPosition"/> to
        /// <paramref name="endPosition"/>. Each leg takes its length divided by its average speed;
        /// the whole trip is eased as one motion.
        /// </summary>
        private void StartPaintPickerPath(PaintPickerPhase phase,
            Vector3 viaPosition, Quaternion viaRotation, float viaSpeed,
            Vector3 endPosition, Quaternion endRotation, float endSpeed)
        {
            _paintPickerPhase = phase;
            _pathPositions[0] = ikTarget.position;
            _pathRotations[0] = ikTarget.rotation;
            _pathPositions[1] = viaPosition;
            _pathRotations[1] = viaRotation;
            _pathPositions[2] = endPosition;
            _pathRotations[2] = endRotation;

            _pathTimes[0] = 0f;
            _pathTimes[1] = Vector3.Distance(_pathPositions[0], viaPosition) / Mathf.Max(0.01f, viaSpeed);
            _pathTimes[2] = _pathTimes[1] + Vector3.Distance(viaPosition, endPosition) / Mathf.Max(0.01f, endSpeed);
            _pathDuration = Mathf.Max(0.25f, _pathTimes[2]);

            _segmentElapsed = 0f;
            _arrivedTime = -1f;
            SetPaintPickerTarget(endPosition, endRotation);
        }

        private void ApplySmoothPose()
        {
            float t = 1f - Mathf.Exp(-smoothSpeed * Time.deltaTime);
            ikTarget.position = Vector3.Lerp(ikTarget.position, _desiredPosition, t);
            ikTarget.rotation = Quaternion.Slerp(ikTarget.rotation, _desiredRotation, t);
        }

        private Quaternion ComputeBrushRotation()
        {
            Quaternion baseRotation = Quaternion.LookRotation(canvas.forward, -canvas.up);
            return baseRotation * Quaternion.AngleAxis(tiltAngle, Vector3.right);
        }

        private void SetPaintPickerTarget(Vector3 position, Quaternion rotation)
        {
            _desiredPosition = position;
            _desiredRotation = rotation;
            _hasValidTarget = false;
        }

        private void OnDisable()
        {
            EndDemoControl();
            EndPaintPickerSequence();
            // Leave the joints with their authored settings.
            SetJointSpeedLimitsActive(true);
        }

        private void SetJointSpeedLimitsActive(bool active)
        {
            if (_speedLimitsActive == active)
                return;

            _speedLimitsActive = active;
            foreach (ConstrainedJoint joint in _speedLimitedJoints)
            {
                if (joint != null)
                    joint.limitSpeed = active;
            }
        }

        private void EndPaintPickerSequence()
        {
            _activePaintBucket = null;
            _paintPickerPhase = PaintPickerPhase.None;
        }

        private Ray BuildMouseRay()
        {
            return mainCamera.ScreenPointToRay(Input.mousePosition);
        }

        private bool IsBrushInContact()
        {
            if (brushTip == null || canvas == null)
                return true;

            Collider canvasCollider = canvas.GetComponent<Collider>();
            if (canvasCollider == null)
                return true;

            Vector3 closestPoint = canvasCollider.ClosestPoint(brushTip.position);
            return Vector3.Distance(brushTip.position, closestPoint) <= paintContactDistance;
        }

        private Vector3 ProjectOntoCanvas(Vector3 worldPoint)
        {
            Plane canvasPlane = new Plane(canvas.forward, canvas.position);
            Ray ray = new Ray(worldPoint, canvas.forward);
            if (canvasPlane.Raycast(ray, out float distance))
                return ray.GetPoint(distance);

            return worldPoint - canvas.forward * canvasPlane.GetDistanceToPoint(worldPoint);
        }

        private void CheckResetInput()
        {
            if (!IsDemoControlled && Input.GetKeyDown(KeyCode.R))
                canvasPainter?.Clear();
        }

        /// <summary>
        /// Starts the temporary bucket pickup sequence. Returns false if the controller is busy.
        /// </summary>
        public bool BeginPaintPickup(PaintBucket bucket)
        {
            if (bucket == null || IsPaintPickerActive || ikTarget == null)
                return false;

            // Cache everything the sequence and its recovery need, so removing the bucket mid-sequence
            // cannot strand the arm.
            _activePaintBucket = bucket;
            _paintPickerApproachPosition = bucket.GetApproachTarget();
            _paintPickerDipPosition = bucket.GetDipTarget();
            _paintPickerReturnPosition = idleTarget != null ? idleTarget.position : ikTarget.position;
            _paintPickerReturnRotation = idleTarget != null ? idleTarget.rotation : ikTarget.rotation;
            _returnSpeed = bucket.ApproachSpeed;
            _dipSpeed = bucket.DipSpeed;
            _backOffSpeed = bucket.BackOffSpeed;
            _pickupColor = bucket.PaintColor;
            _paintPickerDipRotation = ComputeBucketDipRotation(bucket);
            _targetVelocity = Vector3.zero;
            StartPaintPickerPath(PaintPickerPhase.ToBucket,
                _paintPickerApproachPosition, _paintPickerDipRotation, bucket.ApproachSpeed,
                _paintPickerDipPosition, _paintPickerDipRotation, _dipSpeed);
            return true;
        }

        /// <summary>Called by buckets or other systems to set the active brush color.</summary>
        public void SetBrushColor(Color color)
        {
            brushColor = color;
            ApplyBrushPaintColor();
        }

        private void ApplyBrushPaintColor()
        {
            ResolveBrushPaintRenderer();

            if (brushPaintRenderer == null)
                return;

            Material[] sharedMaterials = brushPaintRenderer.sharedMaterials;
            if (sharedMaterials == null ||
                brushPaintMaterialIndex < 0 ||
                brushPaintMaterialIndex >= sharedMaterials.Length)
                return;

            Material sharedMaterial = sharedMaterials[brushPaintMaterialIndex];
            if (sharedMaterial == null)
                return;

            if (_brushPaintPropertyBlock == null)
                _brushPaintPropertyBlock = new MaterialPropertyBlock();

            _brushPaintPropertyBlock.Clear();
            brushPaintRenderer.GetPropertyBlock(_brushPaintPropertyBlock, brushPaintMaterialIndex);

            if (sharedMaterial.HasProperty(BaseColorId))
                _brushPaintPropertyBlock.SetColor(BaseColorId, brushColor);

            if (sharedMaterial.HasProperty(ColorId))
                _brushPaintPropertyBlock.SetColor(ColorId, brushColor);

            brushPaintRenderer.SetPropertyBlock(_brushPaintPropertyBlock, brushPaintMaterialIndex);
        }

        private void ResolveBrushPaintRenderer()
        {
            if (brushPaintRenderer != null)
                return;

            foreach (Renderer childRenderer in GetComponentsInChildren<Renderer>(true))
            {
                Transform rendererTransform = childRenderer.transform;
                if (rendererTransform.name.Trim() != "Paint")
                    continue;

                Transform parent = rendererTransform.parent;
                if (parent == null || parent.name != "PaintBrush")
                    continue;

                brushPaintRenderer = childRenderer;
                return;
            }
        }

        /// <summary>
        /// Brush orientation for the dip: pointing down into the bucket, leaning
        /// <see cref="paintPickerDipTilt"/> degrees away from the arm's base.
        /// </summary>
        private Quaternion ComputeBucketDipRotation(PaintBucket bucket)
        {
            Transform bucketTransform = bucket.transform;
            Vector3 down = -bucketTransform.up;
            Vector3 awayFromArm = Vector3.ProjectOnPlane(bucketTransform.position - transform.position, bucketTransform.up);
            Vector3 brushDirection = awayFromArm.sqrMagnitude > 1e-6f
                ? Vector3.RotateTowards(down, awayFromArm.normalized, paintPickerDipTilt * Mathf.Deg2Rad, 0f)
                : down;
            return Quaternion.LookRotation(brushDirection, bucketTransform.forward);
        }

        private void OnDrawGizmosSelected()
        {
            if (canvas == null)
                return;

            Vector3 surfaceNormal = -canvas.forward;

            Gizmos.color = new Color(0f, 0.8f, 1f, 0.6f);
            Gizmos.DrawWireSphere(canvas.position + surfaceNormal * hoverDistance, 0.05f);

            Gizmos.color = new Color(1f, 0.3f, 0.3f, 0.6f);
            Gizmos.DrawWireSphere(canvas.position + surfaceNormal * paintDistance, 0.03f);

            if (Application.isPlaying && _hasValidTarget)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawSphere(_desiredPosition, 0.03f);
                Gizmos.DrawLine(_desiredPosition, _desiredPosition + surfaceNormal * 0.15f);
            }
        }
    }
}
