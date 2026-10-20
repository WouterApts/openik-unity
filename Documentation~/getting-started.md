# Getting started

See the [README](../README.md#install) for installation in Unity 6.

## Create a chain

1. Create a chain of at least two transforms, parented from root to end effector.
2. Create a separate target transform.
3. Add `FABRIKSolver`, `CCDIKSolver`, `JacobianIKSolver`, or, for a three-joint arm or leg, `TwoBoneIKSolver`
   to a controller object.
4. Assign the target and populate **Joints** in root-to-end order.
5. Press Play and move the target.

Put parent joints before their children. A joint must not be deeper in the
transform hierarchy than the next joint; equal depths are allowed.

Compare solvers in the [core reference](core-reference.md#solvers).

## Join several chains at one point

Use `MultiFABRIKSolver` for a rig whose chains branch from shared joints, such as two arms on a torso.

1. Add a **FABRIK Chain** to the root joint and to each joint where a branch starts, then fill in
   its Joints and Target.
2. Add a **Multi FABRIK Solver** and drag the chains, or the root joint, into its **Chains** list.
3. Set **Junction Mode** on each branch point: **Rigid** turns the branches together as if connected to a rigidbody,
   **Point** gives each branch its own pivot.

See [Multi FABRIK](core-reference.md#multi-fabrik) for the details.

## Choose when the solver runs

The **Execution** section at the top of the solver's Inspector has two settings:

- **Update Mode:** **LateUpdate** runs the solver every frame. **FixedUpdate** runs it with each physics step.
  **Manual** waits for your script to call it.
- **Apply Mode:** **Automatic** moves the chain to each solution. **Manual** only calculates the solution,
  so your script can read it or apply it later.

The defaults, LateUpdate and Automatic, suit most chains. See
[Execution settings](core-reference.md#execution-settings) for details and scripting.

## Add constraints

- `HingeIKJoint`: rotation around one axis.
- `BallSocketIKJoint`: swing and twist limits.
- `SliderIKJoint`: travel along one axis, solved by FABRIK, Multi FABRIK, and Jacobian.

Add these to GameObjects that you want to act like constrained joints and set their local axes and limits.

## Bend toward a pole

To choose which way an elbow or knee bends, expand **Poles** below **Joints** and click **+**.
Move the pole object it creates to where the joint should point. See [Poles](core-reference.md#poles).

## Limit speed

Enable **Limit Speed** under **Motion** on a joint. Set **Max Angular Speed** in degrees per second,
or **Max Linear Speed** in metres per second for sliders. Joints can then lag behind the target.

For smooth motion between two poses, you can try enabling **Synchronize Limited Joints** on
the solver. All limited joints then start and finish their moves together, instead
of the quickest joints snapping into place first. See
[Synchronize Limited Joints](core-reference.md#synchronize-limited-joints).
