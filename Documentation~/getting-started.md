# Getting started

See the [README](../README.md#install) for installation in Unity 6.

## Create a chain

1. Create a chain of at least two transforms, parented from root to end effector.
2. Create a separate target transform.
3. Add `FABRIKSolver`, `CCDIKSolver`, or `JacobianIKSolver` to a controller object.
4. Assign the target and populate **Chain Joints** in root-to-end order.
5. Press Play and move the target.

Put parent joints before their children. A joint must not be deeper in the
transform hierarchy than the next joint; equal depths are allowed.

Compare solvers in the [core reference](core-reference.md#solvers).

## Add constraints

- `HingeIKJoint`: rotation around one axis.
- `BallSocketIKJoint`: swing and twist limits.
- `SliderIKJoint`: travel along one axis, solved only by FABRIK and Jacobian.

Add these to GameObjects that you want to act like constrained joints and set their local axes and limits.

## Limit speed

When Pose and Apply is enabled, you can enable **Limit Speed** under **Motion**. Set **Max Angular Speed** in degrees per second,
or **Max Linear Speed** in metres per second for sliders. Joints can then lag behind the target.

For smooth motion between two poses, you can try enabling **Synchronize Limited Joints** on
the solver. All limited joints then start and finish their moves together, instead
of the quickest joints snapping into place first. See
[Synchronize Limited Joints](core-reference.md#synchronize-limited-joints).
