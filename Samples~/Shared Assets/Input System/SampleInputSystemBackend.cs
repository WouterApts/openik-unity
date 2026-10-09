#if ENABLE_INPUT_SYSTEM && OPENIK_SAMPLE_INPUT_SYSTEM
using UnityEngine;
using UnityEngine.InputSystem;

namespace OpenIK.Showcase
{
    /// <summary>Optional adapter. Only compiled when the Input System package and backend are enabled.</summary>
    public sealed class SampleInputSystemBackend : SampleInput.IBackend
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Install()
        {
            // Prefer this single backend in Both mode; never add legacy input a second time.
            SampleInput.RegisterBackend(new SampleInputSystemBackend());
        }

        public SampleInput.State Read()
        {
            var keyboard = Keyboard.current;
            var mouse = Mouse.current;
            var value = new SampleInput.State();
            if (keyboard != null)
            {
                value.Move = new Vector2(
                    (keyboard.dKey.isPressed || keyboard.rightArrowKey.isPressed ? 1f : 0f)
                  - (keyboard.aKey.isPressed || keyboard.leftArrowKey.isPressed ? 1f : 0f),
                    (keyboard.wKey.isPressed || keyboard.upArrowKey.isPressed ? 1f : 0f)
                  - (keyboard.sKey.isPressed || keyboard.downArrowKey.isPressed ? 1f : 0f));
                value.PortraitPressed = keyboard.pKey.wasPressedThisFrame;
                value.CancelPressed = keyboard.escapeKey.wasPressedThisFrame;
                value.ClearPressed = keyboard.rKey.wasPressedThisFrame;
            }
            if (mouse != null)
            {
                value.Pointer = mouse.position.ReadValue();
                // Input System deltas are about twice the Input Manager's Mouse X/Y values at their
                // default 0.1 sensitivity, so halve the scale to give both input modes the same feel.
                value.Look = mouse.delta.ReadValue() * 0.05f;
                value.Scroll = mouse.scroll.ReadValue().y;
#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
#if OPENIK_SAMPLE_NORMALIZED_SCROLL
                if (InputSystem.settings.scrollDeltaBehavior == InputSettings.ScrollDeltaBehavior.KeepPlatformSpecificInputRange)
#endif
                    value.Scroll /= 120f;
#endif
                value.LeftHeld = mouse.leftButton.isPressed;
                value.LeftPressed = mouse.leftButton.wasPressedThisFrame;
                value.RightHeld = mouse.rightButton.isPressed;
            }
            return value;
        }
    }
}
#endif
