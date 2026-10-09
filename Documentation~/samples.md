# Samples

Use a Unity 6 URP project. The sample materials need URP. (OpenIK itself does not.)
Set Color Space to Linear in Project Settings > Player > Other Settings to match
the sample colors and lighting.

## Import

1. Select OpenIK in Package Manager and open Samples.
2. Import Shared Assets first, then the scene samples you want.
3. Open a scene under `Assets/Samples/OpenIK/0.2.0/` and press Play.
4. Select Display 1 and click the Game view to give keyboard/mouse controls focus.

The samples work with any Active Input Handling setting (**Input Manager (Old)**,
**Input System Package (New)**, or **Both**), so keep your project's setting. They use
keyboard and mouse and need no input actions or Input Manager axes.

Robotic Painter and Spider Walker show their controls in the Game view. Display 2
is reserved for recording cameras.

## Shared Assets

Shared Assets contains models, materials, textures, terrain, a volume profile, and
the shared camera and input scripts. The FBX models do not require Blender.

The scene samples refer to these assets, so import Shared Assets from the same OpenIK
version and keep the `.meta` files. Without them, the scenes lose their references.

## Sample Chains

`Human Arm.unity` has a FABRIK chain. `Basic Chains.unity` has FABRIK and Jacobian
constraint examples. Move targets in the Scene view during Play mode.

## Robotic Painter

`Robotic Painter.unity` has a Jacobian painting arm and a CCD arm.

- Hold the left mouse button over the canvas to paint. Click a bucket to pick up paint.
- Hold the right mouse button to orbit; use the wheel to zoom.
- Press R to clear the canvas during free painting.
- Press P to run the portrait demo, or Escape to stop it.

Picking up paint from the buckets shows off the **synchronized speed limits** feature: slower base and shoulder, faster wrist.

## Spider Walker

`IK Spider.unity` uses four FABRIK chains for procedural leg stepping. Press W/S or
the Up/Down arrows to move, and A/D or the Left/Right arrows to turn. Hold the right
mouse button to look around.

Try changing the leg joints and the spider's step, gait, and movement settings in the Inspector.
