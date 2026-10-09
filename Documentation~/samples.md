# Samples

Use a Unity 6 URP project. The sample materials need URP; the core solvers do not.
Set Color Space to Linear in Project Settings > Player > Other Settings to match
the sample colours and lighting.

## Import

1. Select OpenIK in Package Manager and open Samples.
2. Import Shared Assets first, then the scene samples you want.
3. In Project Settings > Player > Other Settings, set Active Input Handling to
   Input Manager (Old) or Both. Restart Unity if prompted.
4. Open a scene under `Assets/Samples/OpenIK/0.1.0/` and press Play.

Controls use `Horizontal`, `Vertical`, `Mouse X`, and `Mouse Y` from the legacy
Input Manager. Use Display 1 in the Game view; Robotic Arm and Spider Walker have
recording cameras on Display 2.

## Shared Assets

Shared Assets contains models, materials, textures, terrain, a volume profile,
and the shared camera script. The FBX models do not require Blender. Keep `.meta`
files and sample versions together to preserve shared references.

## Basic Chain

`Human Arm.unity` has a FABRIK chain. `SampleScene 2.unity` has FABRIK and Jacobian
constraint examples. Move targets in the Scene view during Play Mode.

## Robotic Arm

`IK Robotic Arm.unity` has a Jacobian painting arm and a CCD arm.

- Hold the left mouse button over the canvas to paint. Click a bucket to pick up paint.
- Hold the right mouse button to orbit; use the wheel to zoom.
- Press R to clear the canvas during free painting.
- Press P to run the portrait demo, or Escape to stop it.

Pickup uses synchronized speed limits: slower base and shoulder, faster wrist.
Tune each hinge under Motion. Free painting disables limits unless you enable
Limit Speed While Painting on `PaintingTargetController`.

The brush travels through a point above the bucket into the paint, then returns
to idle. Each trip is continuous and waits for the brush to arrive. Color changes
after a confirmed dip; failed pickups retreat above the bucket to idle. A slight
brush tilt avoids lining up the wrist and base axes.

## Spider Walker

`IK Spider.unity` uses four FABRIK chains for procedural leg stepping. W/S or
up/down move; A/D or left/right turn. Hold the right mouse button to look around.
