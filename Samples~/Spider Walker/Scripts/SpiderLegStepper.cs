using UnityEngine;

namespace OpenIK.Showcase
{
    /// <summary>
    /// Controls a single spider leg's foot placement using ground raycasting.
    /// Triggers a step when the foot drifts too far from the ideal ground position.
    /// <para>
    /// Gait coordination: Assign alternating <see cref="gaitGroup"/> values (0 or 1) to legs.
    /// Diagonal pairs share the same group. A leg only steps when no leg in the OTHER group is mid-step,
    /// producing a natural alternating gait.
    /// </para>
    /// <para>
    /// For a 4-legged spider, assign groups like:
    /// <code>
    ///   Front-Left (0)    Front-Right (1)
    ///   Back-Left  (1)    Back-Right  (0)
    /// </code>
    /// This means diagonal pairs (FL+BR, FR+BL) move together.
    /// </para>
    /// </summary>
    public class SpiderLegStepper : MonoBehaviour
    {
        [Header("References")]
        [Tooltip("The IK target transform that the IK solver follows.")]
        [SerializeField] private Transform ikTarget;

        [Tooltip("The resting position offset relative to the body (where the foot wants to be). " +
                 "Set this in local space of this transform.")]
        [SerializeField] private Vector3 footRestOffset = new Vector3(1f, 0f, 0.5f);

        [Header("Step Settings")]
        [Tooltip("How far the foot can drift from the ideal position before triggering a step.")]
        [SerializeField] private float stepThreshold = 0.5f;

        [Tooltip("How high the foot lifts during a step.")]
        [SerializeField] private float stepHeight = 0.3f;

        [Tooltip("How long a step takes in seconds.")]
        [SerializeField] private float stepDuration = 0.15f;

        [Tooltip("How far ahead of the rest position to overshoot (anticipates movement).")]
        [SerializeField] private float stepOvershoot = 0.2f;

        [Tooltip("Emergency threshold: if the foot drifts this far from ideal, step regardless of gait coordination. " +
                 "Should be larger than stepThreshold.")]
        [SerializeField] private float forceStepThreshold = 1.5f;

        [Header("Ground Detection")]
        [SerializeField] private float raycastHeight = 3f;
        [SerializeField] private float raycastDistance = 6f;
        [SerializeField] private LayerMask groundLayer = ~0;

        [Header("Gait Coordination")]
        [Tooltip("Gait group index (0 or 1). Diagonal leg pairs share the same group. " +
                 "A leg only steps when no leg in the OTHER group is currently stepping.")]
        [SerializeField] private int gaitGroup;

        [Tooltip("All leg steppers on this spider. Needed for gait coordination.")]
        [SerializeField] private SpiderLegStepper[] allLegs;

        // Current foot state
        private Vector3 _currentFootPos;
        private Vector3 _currentGroundNormal = Vector3.up;

        // Step animation state
        private bool _isStepping;
        private float _stepProgress;
        private Vector3 _stepStartPos;
        private Vector3 _stepEndPos;
        private Vector3 _stepEndNormal;

        // How far this leg is from its ideal foot position. 0 means "not waiting to step".
        private float _wantStepDistance;

        /// <summary>Current world-space foot position (used by SpiderController for body alignment).</summary>
        public Vector3 CurrentFootPosition => _currentFootPos;

        /// <summary>Ground normal at the current foot position.</summary>
        public Vector3 CurrentGroundNormal => _currentGroundNormal;

        /// <summary>Whether this leg is currently mid-step.</summary>
        public bool IsStepping => _isStepping;

        /// <summary>This leg's gait group index.</summary>
        public int GaitGroup => gaitGroup;

        private void Start()
        {
            _currentFootPos = ikTarget != null
                ? ikTarget.position
                : transform.TransformPoint(footRestOffset);
            SnapFootToGround();
        }

        private void Update()
        {
            if (ikTarget == null)
                return;

            if (_isStepping)
                AnimateStep();
            else
                TryInitiateStep();

            ikTarget.position = _currentFootPos;
        }

        private void TryInitiateStep()
        {
            Vector3 idealPosition = transform.TransformPoint(footRestOffset);
            if (!TryFindGround(idealPosition, out RaycastHit groundHit))
                return;

            float distanceFromIdeal = Vector3.Distance(_currentFootPos, groundHit.point);
            if (distanceFromIdeal < stepThreshold)
            {
                _wantStepDistance = 0f;
                return;
            }

            _wantStepDistance = distanceFromIdeal;

            if (distanceFromIdeal < forceStepThreshold && !CanStepNow())
                return;

            Vector3 overshootPosition = groundHit.point + GetStepOvershoot(groundHit.point);
            RaycastHit stepTargetHit = TryFindGround(overshootPosition, out RaycastHit overshootHit)
                ? overshootHit
                : groundHit;

            BeginStep(stepTargetHit.point, stepTargetHit.normal);
        }

        private Vector3 GetStepOvershoot(Vector3 targetPosition)
        {
            Vector3 overshootDirection = targetPosition - _currentFootPos;
            overshootDirection.y = 0f;

            if (overshootDirection.sqrMagnitude <= 0.001f)
                return Vector3.zero;

            return overshootDirection.normalized * stepOvershoot;
        }

        private void BeginStep(Vector3 targetPosition, Vector3 targetNormal)
        {
            _stepStartPos = _currentFootPos;
            _stepEndPos = targetPosition;
            _stepEndNormal = targetNormal;
            _stepProgress = 0f;
            _isStepping = true;
            _wantStepDistance = 0f;
        }

        private bool CanStepNow()
        {
            if (allLegs == null)
                return true;

            // If a gait-mate is already stepping, join it. The other gait is already locked out,
            // so there is no point yielding to its drift.
            bool gaitMateStepping = HasSteppingLegInGroup(gaitGroup);
            for (int i = 0; i < allLegs.Length; i++)
            {
                SpiderLegStepper other = allLegs[i];
                if (other == null || other == this || other.gaitGroup == gaitGroup)
                    continue;

                // Block if any leg in the OTHER group is currently stepping
                if (other._isStepping)
                    return false;

                // Yield if any leg in the OTHER group is drifting further than us,
                // but only if no gait-mate is already stepping (otherwise the other gait is blocked anyway).
                if (!gaitMateStepping && other._wantStepDistance > _wantStepDistance)
                    return false;
            }

            return true;
        }

        private bool HasSteppingLegInGroup(int group)
        {
            if (allLegs == null)
                return false;

            for (int i = 0; i < allLegs.Length; i++)
            {
                SpiderLegStepper other = allLegs[i];
                if (other != null && other != this && other.gaitGroup == group && other._isStepping)
                    return true;
            }

            return false;
        }

        private void AnimateStep()
        {
            _stepProgress += Time.deltaTime / stepDuration;

            if (_stepProgress >= 1f)
            {
                _stepProgress = 1f;
                _isStepping = false;
                _currentFootPos = _stepEndPos;
                _currentGroundNormal = _stepEndNormal;
                return;
            }

            float t = SmoothStep(_stepProgress);
            Vector3 position = Vector3.Lerp(_stepStartPos, _stepEndPos, t);
            float arc = 4f * stepHeight * t * (1f - t);
            position.y += arc;

            _currentFootPos = position;
            _currentGroundNormal = Vector3.Slerp(_currentGroundNormal, _stepEndNormal, t);
        }

        private void SnapFootToGround()
        {
            if (TryFindGround(_currentFootPos, out RaycastHit hit))
            {
                _currentFootPos = hit.point;
                _currentGroundNormal = hit.normal;
            }
        }

        private bool TryFindGround(Vector3 position, out RaycastHit hit)
        {
            Vector3 rayOrigin = position + Vector3.up * raycastHeight;
            return Physics.Raycast(rayOrigin, Vector3.down, out hit, raycastDistance, groundLayer);
        }

        private static float SmoothStep(float t)
        {
            return t * t * (3f - 2f * t);
        }

        private void OnDrawGizmosSelected()
        {
            Vector3 restWorld = transform.TransformPoint(footRestOffset);
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(restWorld, 0.1f);

            // Project the threshold zone down onto the ground surface (within raycast limits).
            // Falls back to the rest position if nothing is hit.
            Vector3 thresholdCenter = restWorld;
            if (TryFindGround(restWorld, out RaycastHit hit))
                thresholdCenter = hit.point;

            Gizmos.color = new Color(0f, 1f, 1f, 0.3f);
            Gizmos.DrawWireSphere(thresholdCenter, stepThreshold);

            if (Application.isPlaying)
            {
                Gizmos.color = _isStepping ? Color.red : Color.green;
                Gizmos.DrawSphere(_currentFootPos, 0.08f);

                if (_isStepping)
                {
                    Gizmos.color = Color.yellow;
                    Gizmos.DrawLine(_stepStartPos, _stepEndPos);
                }
            }
        }
    }
}
