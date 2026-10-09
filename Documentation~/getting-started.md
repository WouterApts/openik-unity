# Getting started

See the [README](../README.md#install) for installation in Unity 6.

## Create a chain

1. Place at least two joint transforms in a rest pose.
2. Create a separate target transform.
3. Add `FABRIKSolver`, `CCDIKSolver`, or `JacobianIKSolver` to a controller object.
4. Assign Target and fill Chain Joints (`chainJoints`) from root to end effector.
5. Press Play and move the target.

Put parent joints before their children. A joint must not be deeper in the
transform hierarchy than the next joint; equal depths are allowed.

Compare solvers in the [core reference](core-reference.md#solvers).

## Add constraints

- `HingeIKJoint`: rotation around one axis.
- `BallSocketIKJoint`: swing and twist limits.
- `SliderIKJoint`: travel along one axis, solved by FABRIK or Jacobian.

Add these to joint GameObjects and set their local axes and limits. Toggle
constraints with Joint Is Enabled, rather than the component checkbox.

## Limit speed

Enable Limit Speed under Motion. Set Max Angular Speed in degrees per second,
or Max Linear Speed in metres per second for sliders. Joints can then lag behind
the target.

Enable Synchronize Limited Joints on the solver to coordinate arrival.
