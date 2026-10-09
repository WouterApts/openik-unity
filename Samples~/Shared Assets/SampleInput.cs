using UnityEngine;

namespace OpenIK.Showcase
{
    /// <summary>Keyboard/mouse input shared by the optional samples, independent of the core package.</summary>
    public static class SampleInput
    {
        public struct State
        {
            public Vector2 Move, Pointer, Look;
            public float Scroll;
            public bool LeftHeld, LeftPressed, RightHeld, PortraitPressed, CancelPressed, ClearPressed;
        }

        public interface IBackend
        {
            State Read();
        }

        private static IBackend backend;
        private static State state;
        private static int frame = -1;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void Reset()
        {
            // Also reset when entering Play mode with domain reload disabled.
            backend = null;
            frame = -1;
            state = default;
#if ENABLE_LEGACY_INPUT_MANAGER || !ENABLE_INPUT_SYSTEM
            backend = new LegacyBackend();
#endif
        }

        // The optional Input System assembly registers before any scene component's Awake.
        public static void RegisterBackend(IBackend value)
        {
            backend = value;
            frame = -1;
        }

        private static State Current
        {
            get
            {
                if (frame != Time.frameCount)
                {
                    frame = Time.frameCount;
                    state = backend != null ? backend.Read() : default;
                }
                return state;
            }
        }

        public static Vector2 Move => Current.Move;
        public static Vector2 LookDelta => Current.Look;
        public static Vector3 mousePosition => Current.Pointer;
        public static Vector2 mouseScrollDelta => new Vector2(0f, Current.Scroll);
        public static bool GetMouseButton(int button) => button == 0 ? Current.LeftHeld : button == 1 && Current.RightHeld;
        public static bool GetMouseButtonDown(int button) => button == 0 && Current.LeftPressed;
        public static bool GetKeyDown(KeyCode key)
        {
            switch (key)
            {
                case KeyCode.P: return Current.PortraitPressed;
                case KeyCode.Escape: return Current.CancelPressed;
                case KeyCode.R: return Current.ClearPressed;
                default: return false;
            }
        }

#if ENABLE_LEGACY_INPUT_MANAGER || !ENABLE_INPUT_SYSTEM
        private sealed class LegacyBackend : IBackend
        {
            // Cursor movement includes pointer acceleration, so it needs a smaller scale than the axes
            // to feel about the same. Only used when the default Mouse X/Y axes have been removed.
            private const float CursorLookScale = 0.065f;

            private Vector2 previousPointer;
            private bool hasPointer, wasFocused;
            private bool hasMouseAxes = true;

            public State Read()
            {
                Vector2 pointer = Input.mousePosition;
                bool focused = Application.isFocused;
                Vector2 look = Vector2.zero;
                if (hasMouseAxes)
                {
                    // Every new project has these default axes; fall back once if a project removed them.
                    try { look = new Vector2(Input.GetAxis("Mouse X"), Input.GetAxis("Mouse Y")); }
                    catch (System.ArgumentException) { hasMouseAxes = false; }
                }
                if (!hasMouseAxes && hasPointer && focused && wasFocused)
                    look = (pointer - previousPointer) * CursorLookScale;
                previousPointer = pointer;
                hasPointer = true;
                wasFocused = focused;
                // Direct keys avoid requiring named axes in the consumer's Input Manager.
                float horizontal = (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow) ? 1f : 0f)
                                 - (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow) ? 1f : 0f);
                float vertical = (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow) ? 1f : 0f)
                               - (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow) ? 1f : 0f);
                return new State
                {
                    Move = new Vector2(horizontal, vertical), Pointer = pointer, Look = look,
                    Scroll = Input.mouseScrollDelta.y,
                    LeftHeld = Input.GetMouseButton(0), LeftPressed = Input.GetMouseButtonDown(0),
                    RightHeld = Input.GetMouseButton(1), PortraitPressed = Input.GetKeyDown(KeyCode.P),
                    CancelPressed = Input.GetKeyDown(KeyCode.Escape), ClearPressed = Input.GetKeyDown(KeyCode.R)
                };
            }
        }
#endif
    }
}
