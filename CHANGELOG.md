# Changelog

This file lists the notable changes in each OpenIK release.

## [0.9.1] - 2026-10-25

### Fixed

- The Two Bone Solver now solves a hinge on the middle joint, or on the upper joint when the middle
  joint is not a hinge, exactly within its limits. The end joint no longer swings off the target.
- A root joint without a parent transform now keeps its joint limits, measured in world space.
  FABRIK, Jacobian, Two Bone, Multi FABRIK, and poles used to ignore them.

## [0.9.0] - 2026-10-20

### Breaking changes

- `SingleChainSolverBase` now holds Target, ChainJoints, Poles, and Tolerance, with `AddPole`,
  `RemovePole`, `ApplyPoles`, and `TryGetPoleDirection`. The FABRIK, CCD, Jacobian, and Two Bone
  solvers derive from it. `OpenIKSolverBase` keeps the update, solve, apply, and output code that
  every solver shares. A custom solver for one chain derives from `SingleChainSolverBase` and still
  overrides `SolveChain(in IKGoal goal)`. Scripts that read Target, ChainJoints, Poles, or Tolerance
  through an `OpenIKSolverBase` reference use `SingleChainSolverBase` instead. Saved scenes keep
  their settings.

### Added

- `LastSolution` is the primary property for reading a solver's computed pose and convergence statistics.
  `LastOutput` remains available as a compatibility alias and returns the same result.

- `TwoBoneIKSolver.EndRotationOffset` exposes the end joint's rotation relative to the target, captured at
  initialization, so scripts can turn the end joint to an exact rotation with Match Target.
- `MultiFABRIKSolver`: a tree of position-targeted FABRIK chains with rigid branch offsets, Pull,
  Pin, Root Pin, joint constraints, per-chain poles and results, and runtime chain edits. Each chain
  is a `MultiFABRIKChain` component, shown as FABRIK Chain, on the joint it starts from. Drag the
  chains into the solver's Chains list, where each item also sets that chain's target, junction,
  Pull, and Pin; the solver sorts them into the tree. The chain Inspector shows the solver that
  lists it, its place in the tree, and its junction. The solver warns when another OpenIK solver
  moves the same joints. Scene view gizmos show branches and targets, and highlight the chains on a
  selected joint.
- Sample Chains has a new `Multi FABRIK Chains.unity` scene with a Multi FABRIK rig: two arms on a
  shared sliding lift, with the right arm splitting into an upper and a lower arm. It combines
  slider, hinge, and ball-socket restrictions, with a separate target for each of the three hands.
- Point junctions in the same solver. Each chain's **Junction Mode** setting chooses Rigid or Point at
  its first joint. Point branch pivots share an attachment position and rotate independently, with
  optional per-branch limits. The chain Inspector shows the complete numbered chain used by the
  solver: joint 0 is the component's transform, and joint 1 is the pivot in Point mode. Author the
  hierarchy and assign joints manually; switching modes preserves both. The Scene view draws the
  first bone of a Rigid branch as a dotted line. Multi FABRIK Chains includes a point rig beside the rigid rig.

- A solver that sets up its own joints can derive from `OpenIKSolverBase` directly and implement
  `ValidateSetup`, `BuildChain`, `HasGoal`, `SolveChain()`, `IsConverged`, and
  `ComputeActualPositionError`. `SolverChain.Initialize` accepts parent indices to build a tree
  of joints.

- **FixedUpdate** Update Mode: the solver steps in `FixedUpdate` with `Time.fixedDeltaTime`,
  in sync with physics. **LateUpdate** stays the default.

- Interpolate, shown with FixedUpdate and on by default, smooths the chain between fixed steps
  like Rigidbody interpolation. Scripts can set it through the `Interpolate` property.

- Poles for all solvers: a **Poles** foldout below Joints makes a middle joint, such as an
  elbow or knee, bend toward a pole target while the solver solves the whole chain. Next to a
  hinge or slider, a pole's section extends to the nearest joint that can twist. Joint limits
  take priority, and a pole never moves the end effector. Scripts use `AddPole`, `RemovePole`,
  and `IKPole`.

- `TwoBoneIKSolver`: an analytic solver for three-joint arms and legs. **End Rotation** can match
  the target's rotation or align a foot to a surface, and **Softness** keeps the middle joint
  from snapping straight.

- Basic Chains has two new chains: a two-bone leg with a knee pole and a foot that follows a
  tilted ground plate, and a FABRIK chain with two poles.

- `SolverPoseRegressionTests` covers poles and the two-bone solver. The existing baseline
  entries are unchanged; the new scenarios were recorded and appended.

### Changed

- Multi FABRIK projects tree joint limits consistently during iteration and final pose rebuilding,
  avoiding a small residual error caused by different constraint tolerances in those two steps.

- Sample Chains no longer has the outdated Human Arm scene. Basic Chains has a shoulder and elbow
  chain.

- Moved the Spider Valley terrain and `ThirdPersonFollowCamera` into Shared Assets, preserving
  their GUIDs. The camera no longer searches for a spider when no target is assigned.

- The sample input reads Shift as a run key.

## [0.3.0] - 2026-10-10

### Breaking changes

- `Solve()` computes `LastOutput` without writing transforms. `Apply(deltaTime)`
  moves the transforms toward it, and `Step(deltaTime)` does both. Use `Apply` in
  place of the removed `ApplyLastOutput`.

- `ApplyMode` replaces `SolveMode`, and the `ApplyMode` property replaces `Mode`.
  Solve And Apply is now **Automatic** and Solve Only is now **Manual**. Saved
  scenes keep their setting.

- `Apply` has no `synchronizeLimitedJoints` argument. It uses the solver's
  Synchronize Limited Joints setting in both apply modes.

- `Solved` fires between solving and applying. Read the result in the new `Applied` event.

- Custom solvers override `SolveChain(in IKGoal goal)` and return a `SolveResult`.
  `OpenIKSolverBase` handles setup, configuration refresh, the starting pose, the
  output, and application.

- The `Chain` and `SolveTarget` overrides, `ApplyAndRaiseSolved`, and
  `SolverChain.SyncSolverStateFromLocalPose` no longer exist.

### Added

- `UpdateMode`: **LateUpdate** steps the solver every frame, and **Manual** leaves
  it to your scripts. The solver Inspector shows Update Mode and Apply Mode in a
  new Execution section.

- The `Applied` event, and public properties for the shared solver settings:
  `Target`, `ChainJoints`, `Tolerance`, `MaxIterations`, `SolveFromRestPose`,
  `SynchronizeLimitedJoints`, `StaticSolverConfiguration`, and `Chain`.

- Solve From Rest Pose for the Jacobian Solver.

- `Solve()` initializes the solver when `Awake` has not run, for example in Edit mode.

- `SolverPoseRegressionTests`, which compare solver output against a recorded baseline.

### Changed

- All solvers now report joints listed out of hierarchy order.

- The Inspector shows Synchronize Limited Joints in both apply modes.

- Improved Sample Chains: Basic Chains has six labeled chains: four FABRIK, one CCD, and
  one Jacobian.

### Fixed

- Jacobian Solver: on long chains of ball socket or unconstrained joints, rotations
  could drift from unit length. The arm then missed the target, or the solve
  returned NaN. The solver now normalizes each rotation after it applies joint
  limits.

- Robotic Painter: the arm follows the mouse only over the canvas. Before, hovering
  a paint bucket moved the arm to it, and clicking near the brush made the arm
  jitter. The controller now raycasts against the canvas collider only. The Canvas
  Layer setting is gone, and the sample no longer needs the `PaintCanvas` layer.

## [0.2.0] - 2026-10-09

### Breaking changes

- `ApplyLastOutput` no longer reads the solver's Synchronize Limited Joints
  setting. It takes an optional `synchronizeLimitedJoints` argument, which is
  `false` by default. In Solve Only mode, call `ApplyLastOutput(deltaTime, true)`
  to synchronize limited joints. Solve And Apply mode still uses the setting.

- The Basic Chain sample is now Sample Chains, and its `SampleScene 2.unity` scene
  is now `Basic Chains.unity`. The Robotic Arm sample is now Robotic Painter, and
  its scene is now `Robotic Painter.unity`. The script GUIDs and the other scene
  GUIDs are the same.

- The `FABRIKSolverEditor`, `CCDIKSolverEditor`, `HingeIKJointEditor`,
  `BallSocketIKJointEditor`, and `SliderIKJointEditor` classes are removed. All
  solvers now use `OpenIKSolverEditor`, and all joints use `ConstrainedJointEditor`.

### Added

- `ConstrainedJoint.GetConstraintBasePosition` gives the joint's rest position. In
  Play mode, the IK parent carries it, the same way `GetConstraintBaseRotation`
  works. `RestPoseOffset` stores the rest position relative to the IK parent.

- The solver Inspector has a Motion section for Synchronize Limited Joints. The
  setting is unavailable in Solve Only mode.

- In Play mode, the solver Inspector shows a notice when Static Solver
  Configuration is on, because the solver ignores later setting changes.

- The samples work with Input Manager (Old), Input System Package (New), or Both.
  They do not need named Input Manager axes or a change to Active Input Handling.

- Robotic Painter and Spider Walker show their controls in the Game view.

- Shared Assets has a dark studio floor material, surface texture, and terrain
  layer. Spider Walker has a new terrain, `Spider Valley.asset`, with hills around
  the walking area.

### Changed

- A ball socket's speed limit now measures travel along the path the joint
  follows, with swing and twist moving together.

- In Play mode, the slider gizmo draws the travel range from the rest position,
  carried by the IK parent. Before, the range moved along with the slider.

- The core reference explains synchronized motion, recovery from outside a limit,
  and the Tolerance, Damping, and Step Size settings in more detail.

### Fixed

- A ball socket with a twist range of less than a full turn could jump to its
  target across the ±180° twist angle, through twist outside its range, when the
  speed budget was large. It now turns the long way, inside its range.

### Removed

- The unused `New Terrain.asset` and `New Terrain 1.asset` files in Shared Assets.

## [0.1.0] - 2026-10-09

First release! 

[0.9.0]: https://github.com/WouterApts/openik-unity/compare/v0.3.0...v0.9.0
[0.3.0]: https://github.com/WouterApts/openik-unity/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/WouterApts/openik-unity/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/WouterApts/openik-unity/releases/tag/v0.1.0
