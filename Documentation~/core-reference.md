# Core reference

## Solvers

All three solvers inherit `OpenIKSolverBase` and solve in `LateUpdate`.

| Component | Method | Solves | Slider Translation |
| --- | --- | --- | --- |
| `FABRIKSolver` | Alternates forward (tip-to-root) and backward (root-to-tip) passes, applying joint constraints. | Position | Yes |
| `CCDIKSolver` | Rotates joints from tip to root toward the target, clamping each rotation. | Position | No |
| `JacobianIKSolver` | Uses damped least squares to adjust joint rotations and slider positions toward the target. | Position, with optional orientation | Yes |

**Max Iterations** limits the number of iterations per solve. Increasing it gives the solver more attempts to reach the target, at a higher CPU cost.
It cannot make an unreachable target reachable or override joint constraints.

**Tolerance** controls how close the solution must be before the solver considers it converged. For FABRIK, CCD, and Jacobian with orientation disabled, this is the end-effector's distance from the target in Unity world units. 
For example, `0.001` corresponds to 1 mm if your project uses one unit per metre. With orientation enabled, Jacobian adds the orientation error, multiplied by **Orientation Weight**, 
to that distance and compares the sum with Tolerance. Smaller values demand greater accuracy and may require more iterations.

Jacobian's **Orientation Mode** determines whether it also tries to match the target's rotation:

- **None:** solve only for the end-effector position.
- **Full Rotation:** match the world rotation of the second-to-last chain transform to the target's rotation. This is the transform controlling the final bone, rather than the end-effector transform.
- **Bone Direction:** align the direction from the second-to-last transform to the end effector with the target's forward axis (`+Z`), without controlling twist.

Increase **Damping** if the solve becomes unstable near difficult configurations, such as a straightened chain. Higher damping generally produces smaller corrections and may require more iterations. 
Reducing **Step Size** also reduces each iteration's correction. These settings control numerical solving; use joint speed limits to control movement over time.

## Joint constraints

Add joint constraint components to the transforms that act as joints in your chain.

When the solver initializes in Awake, it records the joints' starting positions and rotations. This is the *rest pose*, which provides the reference for joint limits.
Arrange the chain in its intended starting pose before entering Play mode. If you create the chain through code, set up its pose before the solver initializes.

| Component | Constraint |
| --- | --- |
| `HingeIKJoint` | Rotation around one local axis, between a minimum and maximum angle. |
| `BallSocketIKJoint` | Swing and twist limits around the rest pose. |
| `SliderIKJoint` | Limited travel along a local axis, measured from the rest position. |

Use **Joint Is Enabled** to turn a joint's constraints and speed limit on or off.
Use the Scene view constraint gizmos to check the axes and allowed movement.

## Joint speed limits

To limit how fast a joint moves, enable **Limit Speed** under **Motion** on the joint component:

- **Max Angular Speed** limits hinges and ball sockets, in degrees per second.
- **Max Linear Speed** limits sliders, in metres per second.

The limit applies to the joint's own movement relative to its IK parent. A parent carries its
children with it, and that does not count against their limits. Joints without a limit go
straight to the solved pose.

Speed limits use scaled delta time, so they do not depend on frame rate or Max Iterations.
Limited joints hold still while `Time.timeScale` is 0.

While limited joints are still moving, each solve continues from the previous solution, so the
chain keeps working toward the same pose instead of switching to another one. FABRIK and CCD
skip this when **Solve From Rest Pose** is enabled.

With **Static Solver Configuration** enabled, the solver reads speed limits once at startup,
like the other joint settings.

### Synchronize Limited Joints

In **Solve and Apply** mode, if motion looks uneven, with some joints snapping into place while others are still moving,
try enabling **Synchronize Limited Joints** on the solver. All limited joints then reach the
target pose together, and no joint exceeds its own limit. 

To use this feature in **Solve Only** mode, call`ApplyLastOutput(deltaTime, synchronizeLimitedJoints = true)` instead.

## Solving and applying

| API | Behavior |
| --- | --- |
| `Mode = SolveMode.SolveAndApply` | Solves and applies each frame, including speed limits. Default. |
| `Mode = SolveMode.SolveOnly` | Solves and exposes the result without writing transforms. |
| `LastOutput` (`IKSolverOutput`) | The result of the most recent solve. See below. |
| `Solved` | Fires after each solve. Output buffers are reused; copy data you need to keep. |
| `SnapToSolution()` | Applies the stored result immediately in either mode, ignoring speed limits. It does not solve again. |

`LastOutput` is the solved state of the IK chain. It is the main way to connect the OpenIK solvers to your own code.
It holds the solved world-space position and rotation of every joint (`WorldPositions` and `WorldRotations`, in root-to-end order), plus the solve details `IterationsUsed`, `Converged`, and `FinalError`.
Read it after each solve or in the `Solved` event, for example to blend it with animation or to apply it yourself with `ApplyTo`.
Each consecutive solve overwrites it, so copy any data you need to keep.

`ApplicationStatus` (`IKApplicationStatus`) reports the actual pose. `Applied`
means it was written; `ReachedSolution` means the joints reached the solved pose.
That pose can still fall short of the target. Check `PositionError` for distance
and `OrientationError` for degrees (NaN without an orientation objective).

`AnyJointLimited`, `AnyJointBlocked`, and `AnyJointStartedOutsideLimits` report
individual joint outcomes. A converged `LastOutput` can leave the arm catching up.

## Extending OpenIK

Custom solvers and joint types can use the same building blocks as the built-in ones.

A custom solver derives from `OpenIKSolverBase`, which applies the solved pose with
`IKPoseApplier`, including speed limits. `SolverChain` and `SolverJoint` hold chain state.

A custom joint derives from `ConstrainedJoint` and creates its constraints from
`ISegmentConstraint` and `IAngularConstraint`. Both extend `IJacobianDofProvider`, which
exposes the joint's degrees of freedom to the Jacobian solver.

Custom speed limits use `IAngularMotionProvider` or `ISegmentMotionProvider` plus
the component's `MotionSupport`. Related types are
`JointMotionLimit`, `JointMotionSupport`, `JointMotionStep`, and `JointMotionStatus`.

The runtime also includes math utilities: `float6`, `Matrix6x6`, and `Matrix6xN`.

## Troubleshooting

- Solver disables itself: assign a target and at least two non-null Chain Joints.
  The Console should report the failed setup check.

- Target stays out of reach: check chain length and joint limits, then increase
  iterations. Increase tolerance only if you accept more error. With speed
  limits, inspect `ApplicationStatus.PositionError` as well as `ReachedSolution`.

- Unexpected movement: check joint order, local constraint axes, and the rest pose.
