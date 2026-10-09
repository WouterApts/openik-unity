<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="Documentation~/images/openik-logo-white.svg">
    <source media="(prefers-color-scheme: light)" srcset="Documentation~/images/openik-logo-dark.svg">
    <img src="Documentation~/images/openik-logo-dark.svg" alt="OpenIK" width="360">
  </picture>
</p>

**OpenIK** is a free Unity package that provides components for building inverse
kinematics chains with FABRIK, CCD, or Jacobian solvers and hinge, ball socket, and
slider constraints.

## What's included

- **Three solvers:** FABRIK and CCD for end-effector position. Damped least-squares
  Jacobian for position and orientation.
- **Joint constraints:** hinge rotation limits, ball socket swing/twist limits,
  and slider travel limits.
- **Joint speed limits:** cap how fast each joint moves by setting its angular or
  linear speed.
- **Editor tools:** intuitive component inspectors and constraint gizmos.
- **Importable examples:** basic chains, a robotic painting arm, and a walking
  spider, with shared models, materials, and terrain.
- **Documentation and tests:** setup guide, solver/API reference, and optional
  EditMode regression tests.

## Requirements

- Unity 6 (6000.0) or newer.
- Unity Mathematics 1.3.3, installed automatically as a package dependency.

The core package does not require a render pipeline or the Input System package.
The optional samples use URP and work with the legacy Input Manager, the Input
System package, or both.

## Install

In Unity, open **Window > Package Manager**, select **+ > Install package from
git URL** (called **Add package from git URL** in some versions), and paste:

```text
https://github.com/WouterApts/openik-unity.git#v0.2.0
```

The version tag pins your project to a release. To upgrade later, change the tag
in your project's `Packages/manifest.json` to the desired published version.

For local development, clone the repository outside your Unity project, then use
**Install package from disk** and select its `package.json`.

## Quick start

1. Create a chain of at least two transforms, parented from root to end effector.
2. Create a separate target transform.
3. Add `FABRIKSolver`, `CCDIKSolver`, or `JacobianIKSolver` to a controller object.
4. Assign the target and populate **Chain Joints** in root-to-end order.
5. Optionally add `HingeIKJoint`, `BallSocketIKJoint`, or `SliderIKJoint` components to the transforms that act as joints in your chain.
6. Press Play and move the target.

See [Getting started](Documentation~/getting-started.md) and the
[Core reference](Documentation~/core-reference.md) for constraints and speed limits.

## Samples

In a Unity 6 URP project, select OpenIK in Package Manager and open **Samples**.
Import **Shared Assets** first, then **Sample Chains**, **Robotic Painter**, or
**Spider Walker**. Open the scenes under `Assets/Samples/OpenIK/0.2.0/`.

Press Play and click the Game view. The on-screen guide will list the controls.
See the [sample guide](Documentation~/samples.md) for scenes, controls, and setup.

## Tests

Install Unity Test Framework and add `"com.wouterapts.openik"` to the `testables`
array of your project's `Packages/manifest.json`. Run the OpenIK EditMode tests
from **Window > General > Test Runner**. These tests are optional and are excluded
from player builds.

## License

[MIT](LICENSE.md), copyright 2026 Wouter Apts.
