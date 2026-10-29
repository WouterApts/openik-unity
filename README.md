<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="Documentation~/images/openik-logo-white.svg">
    <source media="(prefers-color-scheme: light)" srcset="Documentation~/images/openik-logo-dark.svg">
    <img src="Documentation~/images/openik-logo-dark.svg" alt="OpenIK" width="360">
  </picture>
</p>

**OpenIK** is a free, open-source inverse kinematics package for Unity.
It includes FABRIK, CCD, Jacobian, and Two Bone solvers, with hinge, ball-and-socket, and slider limits.

https://github.com/user-attachments/assets/124f04d5-e9f4-4067-ba43-f49f7a5b3c19

## What's included

- **Solvers:** FABRIK and CCD for position solving. Damped least-squares Jacobian for position and orientation. Multi-FABRIK for branching chains with separate targets.
- **Joint constraints:** Hinge angle limits, ball-and-socket swing/twist limits, and slider travel limits.
- **Joint speed limits:** Cap each joint's angular or linear speed.
- **Pole targets:** Control elbow and knee bend directions with pole targets, supported by all solvers.
- **Execution control:** Automatic per-frame solving or manual control from scripts, including when results are applied.
- **Editor tools:** Intuitive component inspectors and constraint gizmos.
- **Examples:** Ready-to-import scenes featuring basic IK chains, a robotic painting arm, and a walking spider, with shared models, materials, and terrain.
- **Documentation and tests:** Setup guide, solver and API reference, and optional EditMode regression tests.

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
https://github.com/WouterApts/openik-unity.git#v0.9.2
```

The version tag pins your project to a release. To upgrade later, change the tag
in your project's `Packages/manifest.json` to the desired published version.

For local development, clone the repository outside your Unity project, then use
**Install package from disk** and select its `package.json`.

## Quick start

1. Create a chain of at least two transforms, parented from root to end effector.
2. Create a separate target transform.
3. Add `FABRIKSolver`, `CCDIKSolver`, `JacobianIKSolver`, or `TwoBoneIKSolver` (three joints) to a controller object.
4. Assign the target and populate **Joints** in root-to-end order.
5. Optionally add `HingeIKJoint`, `BallSocketIKJoint`, or `SliderIKJoint` components to the transforms that act as joints in your chain.
6. Press Play and move the target.

By default the solver runs every frame and moves the chain. To control when it runs
or applies its result, change **Update Mode** and **Apply Mode** in the solver's
**Execution** section.

See [Getting started](Documentation~/getting-started.md) and the
[Core reference](Documentation~/core-reference.md) for constraints, speed limits,
and execution settings.

## Samples

In a Unity 6 URP project, select OpenIK in Package Manager and open **Samples**.
Import **Shared Assets** first, then **Sample Chains**, **Robotic Painter**, or
**Spider Walker**. Open the scenes under `Assets/Samples/OpenIK/0.9.2/`.

Press Play and click the Game view. The on-screen guide will list the controls.
See the [sample guide](Documentation~/samples.md) for scenes, controls, and setup.

## Tests

Install Unity Test Framework and add `"com.wouterapts.openik"` to the `testables`
array of your project's `Packages/manifest.json`. Run the OpenIK EditMode tests
from **Window > General > Test Runner**. These tests are optional and are excluded
from player builds.

## License

[MIT](LICENSE.md)
