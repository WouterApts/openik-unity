# Core reference

## Solvers

All solvers share the lifecycle in `OpenIKSolverBase`. FABRIK, CCD, Jacobian, and Two Bone derive through
`SingleChainSolverBase`; Multi FABRIK derives directly from `OpenIKSolverBase` and builds a tree of chains.
By default they solve and apply the result every frame in `LateUpdate`.
The **Execution** settings change when a solver runs and whether it moves the chain; see [Execution settings](#execution-settings).

| Component | Method | Solves | Slider Translation |
| --- | --- | --- | --- |
| `FABRIKSolver` | Alternates forward (tip-to-root) and backward (root-to-tip) passes, applying joint constraints. | Position | Yes |
| `MultiFABRIKSolver` | Solves several FABRIK chains together with rigid attachments or independently rotating branches at a shared point. | Multiple positions | Yes |
| `CCDIKSolver` | Rotates joints from tip to root toward the target, clamping each rotation. | Position | No |
| `JacobianIKSolver` | Uses damped least squares to adjust joint rotations and slider positions toward the target. | Position, with optional orientation | Yes |
| `TwoBoneIKSolver` | Places a three-joint limb directly with the law of cosines, in one step. | Position, with optional end rotation | No |

**Max Iterations** limits the number of iterations per solve. Increasing it gives the solver more attempts to reach the target, at a higher CPU cost.
It cannot make an unreachable target reachable or override joint constraints.

**Tolerance** controls how close the solution must be before the solver considers it converged. For FABRIK, CCD, and Jacobian with orientation disabled, this is the end-effector's distance from the target in Unity world units. 
For example, `0.001` corresponds to 1 mm if your project uses one unit per metre. With orientation enabled, Jacobian adds the orientation error, multiplied by **Orientation Weight**, 
to that distance and compares the sum with Tolerance. Smaller values demand greater accuracy and may require more iterations.

Jacobian's **Orientation Mode** determines whether it also tries to match the target's rotation:

- **None:** solve only for the end-effector position.
- **Full Rotation:** match the world rotation of the second-to-last chain transform to the target's rotation. This is the transform controlling the final bone, rather than the end-effector transform.
- **Bone Direction:** align the direction from the second-to-last transform to the end effector with the target's forward axis (`+Z`), without controlling twist.

Increase **Damping** if the solver becomes unstable near difficult configurations, such as a straightened chain. Higher damping generally produces smaller corrections and may require more iterations. 
Reducing **Step Size** also reduces each iteration's correction. These settings control numerical solving; use joint speed limits to control movement over time.

FABRIK's **Singularity Handling** helps a fully straightened chain bend when the target lies on the chain's line.
CCD's **Rotation Step** applies only part of each joint's rotation per step; lower values can make tightly constrained chains more stable.

**Solve From Rest Pose** starts every solve from the rest pose captured at initialization instead of from the current pose.
The result then depends on the target and the chain's root, not on the previous frame. The Spider Walker legs use this.

**Static Solver Configuration** reads joint constraints and speed limits once, when the solver initializes, and skips refreshing them every frame.
Turn it on for chains whose joint settings do not change during Play mode.

## Multi FABRIK

`MultiFABRIKSolver` solves several FABRIK chains that share joints, such as two arms on a torso, as
one tree. Joint limits apply as usual and take priority over targets.

Each chain is a **FABRIK Chain** component on the joint it starts from. That joint is labeled joint 0 on the
chain, and the Joints list continues from there down to the tip. Each chain has its own optional
Target, Tolerance, and Poles; at least one chain needs a target. Drag the chains into the solver's
**Chains** list in any order. The solver sorts them into a tree and reports setup problems, such as a
missing root or a joint claimed by two chains.

| Setting | Effect |
| --- | --- |
| **Pull** (per chain, in the Chains list) | How strongly the chain pulls on the joint it branches from, from 0 to 1. |
| **Pin** (per chain, in the Chains list) | How strongly the chain resists the chains that branch from it, from 0 to 1. |
| **Root Pin** (solver) | How strongly the root keeps its position. |

**Junction Mode** sets how the chains on one joint connect:

- **Rigid** (default): the branches are fixed offsets on one shared joint and turn together, as if
  connected to a rigidbody.
- **Point**: each branch gets its own pivot, a direct child of the joint, so the branches turn
  independently. All pivots need the same local position. Put hinge or ball-socket limits on the
  pivots, not on the joint itself.

From scripts, read each chain's `LastResult` for its error and convergence, use `GetPull`, `SetPull`,
`GetPin`, and `SetPin`, and add or remove chains at runtime with `AddChain` and `RemoveChain`. A pole
section cannot cross a branch point.

## Two Bone

`TwoBoneIKSolver` places a limb of exactly three joints (upper, middle, and end) directly, in one
step. A pole connected to the middle joint sets the bend side (see [Poles](#poles)). Without one, the limb keeps
its current bend. A hinge on the middle joint, or on the upper joint when the middle joint is not a
hinge, is solved exactly within its limits. Two Bone IK does not solve slider translation.

- **End Rotation:** **Match Target** copies the target's rotation, keeping the end joint's offset to
  the target from initialization (`EndRotationOffset`). **Align Up** aligns the end joint's up axis
  with the target's, for example to plant a foot on a slope.
- **Softness:** eases the reach near full extension so the limb doesn't snap straight. The end joint
  then stops slightly short of distant targets; use 0 for an exact reach.

## Joint constraints

Add joint components to the transforms that act as constrained joints in your chain.

When the solver initializes in Awake, it records the joints' starting positions and rotations. This is the *rest pose*, which provides the reference for joint limits.
Each joint's limits are measured relative to its IK parent. For the root joint, that is its parent
transform, or world space when it has none.
Arrange the chain in its intended starting pose before entering Play mode. If you create the chain through code, set up its pose before the solver initializes.

| Component | Constraint |
| --- | --- |
| `HingeIKJoint` | Rotation around one local axis, between a minimum and maximum angle. |
| `BallSocketIKJoint` | Swing and twist limits around the rest pose. |
| `SliderIKJoint` | Limited travel along a local axis, measured from the rest position. |

Use **Joint Is Enabled** to turn a joint's constraints and speed limit on or off.
Use the Scene view constraint gizmos to check the axes and allowed movement.

## Poles

A pole makes a middle joint, such as an elbow or knee, bend toward a pole target while the solver still
solves the whole chain. Add one under **Poles** in the solver's Inspector, or with `AddPole` from a
script. The pole only turns the chain around the line from the joint's parent to its child, so it never
moves the end effector. Joint limits take priority, so on a constrained chain the bend may not fully
face the pole. Each entry has:

- **Bend Joint:** the middle joint that bends toward the pole.
- **Pole Target:** the transform it bends toward. Leave it empty to turn the pole off.
- **Weight:** how far the bend turns toward the pole, from 0 to 1.

## Joint speed limits

To limit how fast a joint moves, enable **Limit Speed** under **Motion** on the joint component:

- **Max Angular Speed** limits hinges and ball sockets, in degrees per second.
- **Max Linear Speed** limits sliders, in metres per second.

The limit applies to the joint's own movement relative to its IK parent. A parent carries its
children with it, and that does not count against their limits. Joints without a limit go
straight to the solved pose.

Speed limits use the delta time of each step, so they do not depend on frame rate or Max Iterations.
With **Update Mode** set to **LateUpdate**, that is the scaled `Time.deltaTime`, and limited joints
hold still while `Time.timeScale` is 0. With **FixedUpdate**, it is `Time.fixedDeltaTime`. When you step
the solver yourself, it is the value you pass to `Step` or `Apply`.

While limited joints are still moving, each solve continues from the previous solution, so the
chain keeps working toward the same pose instead of switching to another one. Solvers skip
this when **Solve From Rest Pose** is enabled.

With **Static Solver Configuration** enabled, the solver reads speed limits once at startup,
like the other joint settings.

### Synchronize Limited Joints

If motion looks uneven, with some joints snapping into place while others are still moving,
try enabling **Synchronize Limited Joints** on the solver. All limited joints then reach the
target pose together, and no joint exceeds its own limit. The setting applies whenever the
solver applies a solution, including through `Apply`.

## Solving and applying

Every solver update has two separate steps:

1. **Solve** computes a new pose for the chain and stores it in `LastSolution`. It never moves the transforms.
2. **Apply** moves the transforms toward `LastSolution`, at most as fast as the joint speed limits allow.

### Execution settings

The **Execution** section at the top of every solver's Inspector controls these steps.

**Update Mode** sets when the solver runs:

- **LateUpdate** (default): the solver runs one step every frame in `LateUpdate`, using scaled delta time.
- **FixedUpdate**: the solver runs one step in each `FixedUpdate`, using `Time.fixedDeltaTime`. Use it when
  the chain works with physics, for example a target moved by a Rigidbody or colliders on the joints.
- **Manual**: the solver does nothing on its own. Call `Step(deltaTime)`, or `Solve()` + `Apply(deltaTime)`,
  from your own script. Use this to control the order of several solvers or to run at your own rate.

**Interpolate** appears below Update Mode when it is set to **FixedUpdate**, and is on by default. Fixed
steps do not line up with rendered frames, so without it the chain moves in visible steps whenever the frame
rate differs from the physics rate. With it, the solver works like Rigidbody interpolation: each frame
shows a blend of the last two fixed-step poses, so the visible chain trails the solved pose by up to one fixed
step. Before each fixed step the solver puts the solved pose back, and physics and the next solve start from it.
Interpolation only runs with **Apply Mode** set to **Automatic**.

FixedUpdate is not intended for chains whose bones an Animator also drives. The Animator writes its pose after
`FixedUpdate` and overwrites the IK result. Keep those chains on **LateUpdate**.

**Apply Mode** sets what a step does with the solution:

- **Automatic** (default): the solver applies the solution to the transforms, with joint speed limits.
- **Manual**: the solver only solves. The transforms stay where they are until you call `Apply(deltaTime)` or
  `SnapToSolution()`, or until you use `LastSolution` yourself, for example to blend it with animation.

| Update Mode | Apply Mode | Result                                                         |
| --- | --- |----------------------------------------------------------------|
| LateUpdate | Automatic | Solves and moves the chain every frame. (**default settings**) |
| LateUpdate | Manual | Only Solves. Your script reads `LastSolution` or calls `Apply`.  |
| FixedUpdate | Automatic | Solves and moves the chain every fixed step, with optional interpolation. |
| FixedUpdate | Manual | Only solves, every fixed step. Your script reads `LastSolution` or calls `Apply`. |
| Manual | Automatic | Your script calls `Step(deltaTime)` to solve and apply.        |
| Manual | Manual | Your script calls `Solve()` and `Apply(deltaTime)` separately. |

Scripts can change these settings at runtime through the `UpdateMode`, `Interpolate`, and `ApplyMode` properties.

### Scripting API

| API | Behavior |
| --- | --- |
| `Step(deltaTime)` | Solves, then applies when Apply Mode is Automatic. `LateUpdate` calls this with `Time.deltaTime`, `FixedUpdate` with `Time.fixedDeltaTime`. |
| `Solve()` | Solves and fills `LastSolution` without writing transforms. Returns false when the solver has no valid setup or target. |
| `Apply(deltaTime)` | Moves the transforms toward `LastSolution` within the speed limits. With Automatic apply, `Step` already does this. |
| `SnapToSolution()` | Writes `LastSolution` immediately, ignoring speed limits. It does not solve again. |
| `Solved` | Fires after each solve, before the solution is applied. Output buffers are reused; copy data you need to keep. |
| `Applied` | Fires after a solution is written to the transforms, with the resulting `ApplicationStatus`. |

`LastSolution` is the solved state of the IK chain. It is the main way to connect the OpenIK solvers to your own code.
It holds the solved world-space position and rotation of every joint (`WorldPositions` and `WorldRotations`, in root-to-end order), plus the solve details `IterationsUsed`, `Converged`, and `FinalError`.
Read it after each solve or in the `Solved` event, for example to blend it with animation or to apply it yourself with `ApplyTo`.
Each consecutive solve overwrites it, so copy any data you need to keep.

`LastOutput` is a compatibility alias for `LastSolution` and returns the same result. Use `LastSolution` in new code.

For example, to make one solver always run before another, set both to **Manual** and step them from your own script:

```csharp
void LateUpdate()
{
    armSolver.Step(Time.deltaTime);
    handSolver.Step(Time.deltaTime);
}
```

`ApplicationStatus` (`IKApplicationStatus`) reports the actual pose. `Applied`
means the latest solution was written; it is false between a solve and its apply.
`ReachedSolution` means the joints reached the solved pose.
That pose can still fall short of the target. Check `PositionError` for distance
and `OrientationError` for degrees (NaN without an orientation objective).

`AnyJointLimited`, `AnyJointBlocked`, and `AnyJointStartedOutsideLimits` report
individual joint outcomes. A converged `LastSolution` can leave the arm catching up.

## Extending OpenIK

Custom solvers and joint types can use the same building blocks as the built-in ones.

A custom solver for one chain derives from `SingleChainSolverBase` and overrides one method,
`SolveChain(in IKGoal goal)`. The base classes handle everything around it: setup checks,
constraint updates, choosing the starting pose, filling `LastSolution`, and applying the result
with speed limits through `IKPoseApplier`. Inside `SolveChain`, read and write only the solver
state of `Chain` (`SolverChain` and its `SolverJoint` entries), never the transforms, and return
a `SolveResult` with the iteration count and remaining error. Optional overrides cover the
end-joint rotation (`WritesEndRotation`), slider support (`SupportsSliderTranslation`), and
setup hooks (`OnInitialized`, `OnConstraintsRebound`). To support poles, call `ApplyPoles()` inside
`SolveChain`, for example once per iteration. An analytic solver can instead read the pole's direction with
`TryGetPoleDirection` and place the bend there directly, as the Two Bone Solver does.

A solver that sets up its own joints instead of using Target and Joints derives from
`OpenIKSolverBase` directly. It implements `ValidateSetup`, `BuildChain`, `HasGoal`,
`SolveChain()`, `IsConverged`, and `ComputeActualPositionError`, and keeps the same update,
output, and speed-limited application. `BuildChain` can build a tree of joints by passing parent
indices to `SolverChain.Initialize`.

A custom joint derives from `ConstrainedJoint` and creates its constraints from
`ISegmentConstraint` and `IAngularConstraint`. Both extend `IJacobianDofProvider`, which
exposes the joint's degrees of freedom to the Jacobian solver.

Custom speed limits use `IAngularMotionProvider` or `ISegmentMotionProvider` plus
the component's `MotionSupport`. Related types are
`JointMotionLimit`, `JointMotionSupport`, `JointMotionStep`, and `JointMotionStatus`.

The runtime also includes math utilities: `float6`, `Matrix6x6`, and `Matrix6xN`.

## Troubleshooting

- Solver disables itself: assign a target and at least two non-null entries in **Joints**, or
  exactly three for the Two Bone Solver. The Console should report the failed setup check.

- Pole has no effect: check the pole's entry in the solver's **Poles** section. A hinge or
  slider next to the bend joint blocks the turn unless a joint further along can twist.
  `IsLimited` reports when joint limits shortened the turn.

- Target stays out of reach: check chain length and joint limits, then increase
  iterations. Increase tolerance only if you accept more error. With speed
  limits, inspect `ApplicationStatus.PositionError` as well as `ReachedSolution`.

- Chain does not move: check the solver's **Execution** section. With **Update Mode**
  set to **Manual**, a script must call `Step`. With **Apply Mode** set to **Manual**, a
  script must call `Apply` or `SnapToSolution`.

- Unexpected movement: check joint order, local constraint axes, and the rest pose.
  The Console reports joints listed in the wrong hierarchy order.
