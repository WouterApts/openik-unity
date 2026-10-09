# Changelog

This file lists the notable changes in each OpenIK release.

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

[0.2.0]: https://github.com/WouterApts/openik-unity/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/WouterApts/openik-unity/releases/tag/v0.1.0
