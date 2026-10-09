# Core reference

## Solvers

All three solvers inherit `OpenIKSolverBase` and solve in `LateUpdate`.

| Component | Method | Solves | Sliders |
| --- | --- | --- | --- |
| `FABRIKSolver` | Alternates tip-to-root and root-to-tip position passes, applying joint constraints. | Position | Yes |
| `CCDIKSolver` | Rotates joints from tip to root toward the target, clamping each rotation. | Position | Does not solve slider translation |
| `JacobianIKSolver` | Uses a 6xN Jacobian and damped least squares for rotational and translational degrees of freedom. | Position, with optional orientation | Yes |

FABRIK follows "FABRIK: A fast, iterative solver for the Inverse Kinematics problem".
Its passes preserve segment lengths within their constraints and anchor the root.

Max Iterations limits work per frame; Tolerance sets the stopping threshold.
FABRIK and CCD measure position error. Jacobian adds a weighted orientation term:
an angle in radians for Full Rotation, or the direction cross-product magnitude
for Bone Direction. This mixes units. Lower Orientation Weight favors position;
higher values favor orientation.

Jacobian can ignore orientation, match the final bone to the target rotation, or
aim it along the target's forward axis without controlling twist. Higher Damping improves
stability near singularities but can slow convergence.

## Joint constraints

Add constraints to joint transforms. Solvers initialize their runtime constraints
from the authored rest pose.

| Component | Constraint |
| --- | --- |
| `HingeIKJoint` | Rotation around one local axis, between minimum and maximum angles. |
| `BallSocketIKJoint` | Swing and twist limits around the rest pose. |
| `SliderIKJoint` | Signed travel along a local axis, measured from the rest position. |

Joint Is Enabled toggles the joint's constraints and speed limit; the component
checkbox does not. Positive hinge angles follow the right-hand rule around the
hinge axis, matching every solver's gizmo.

## Joint speed limits

Under Motion, enable Limit Speed. Max Angular Speed limits hinges and ball sockets
in degrees per second, including combined swing and twist. Max Linear Speed limits
sliders in metres per second along their axis.

Limits act relative to the IK parent; parent or root movement does not spend the
joint's speed budget. Unlimited joints take their solved local pose immediately.
Limited joints move by at most speed times scaled delta time, independent of
iteration count, and hold at zero delta time.

Synchronize Limited Joints scales their speeds by a common factor for arrival
together. Otherwise, each uses its own limit. Hinges follow their allowed arc;
ball sockets stay within swing and twist limits when starting inside them.
Recovery from outside a limit still respects the speed budget.

While joints catch up, the next solve starts from the previous solution at the
current root to avoid switching solutions. Once they arrive, it reads transforms
again. FABRIK and CCD can override this with Solve From Rest Pose.
Static Solver Configuration skips runtime setting refreshes, including speed limits.

## Solving and applying

| API | Behavior |
| --- | --- |
| `Mode = SolveMode.SolveAndApply` | Solves and applies each frame, including speed limits. Default. |
| `Mode = SolveMode.SolveOnly` | Solves and exposes the result without writing transforms. |
| `LastOutput` (`IKSolverOutput`) | Joint poses, iteration count, `Converged`, and `FinalError` for the solve. |
| `Solved` | Fires after each solve. Output buffers are reused; copy data you need to keep. |
| `ApplyLastOutput(deltaTime)` | Applies the stored result with speed limits in `SolveOnly` mode. |
| `SnapToSolution()` | Applies the stored result immediately in either mode, ignoring speed limits. It does not solve again. |

`ApplicationStatus` (`IKApplicationStatus`) reports the actual pose. `Applied`
means it was written; `ReachedSolution` means the joints reached the solved pose.
That pose can still fall short of the target. Check `PositionError` for distance
and `OrientationError` for degrees (NaN without an orientation objective).

`AnyJointLimited`, `AnyJointBlocked`, and `AnyJointStartedOutsideLimits` report
individual joint outcomes. A converged `LastOutput` can leave the arm catching up.

## Extending OpenIK

`SolverChain` and `SolverJoint` hold chain state. `ISegmentConstraint` and
`IAngularConstraint` define constraints. `IJacobianDofProvider` exposes degrees of
freedom; `IKPoseApplier` applies poses.

Custom speed limits use `IAngularMotionProvider` or `ISegmentMotionProvider` plus
the component's `MotionSupport`. Related types are
`JointMotionLimit`, `JointMotionSupport`, `JointMotionStep`, and `JointMotionStatus`.
The runtime also includes constraint implementations, matrix types, and `float6`.

## Troubleshooting

- Solver disables itself: assign a target and at least two non-null Chain Joints.
  The Console reports the failed setup check.
- Target stays out of reach: check chain length and joint limits, then increase
  iterations. Increase tolerance only if you accept more error. With speed
  limits, inspect `ApplicationStatus.PositionError` as well as `ReachedSolution`.
- Unexpected movement: check joint order, local constraint axes, and the rest pose.
