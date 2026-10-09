using System.Runtime.CompilerServices;
using Unity.Mathematics;

namespace OpenIK
{
    /// <summary>
    /// A 6-component vector stored as two float3 for SIMD-friendly layout.
    /// Used for Jacobian IK: 3 position values + 3 orientation values.
    /// </summary>
    public struct float6
    {
        /// <summary>Values 0-2 (e.g. linear / position).</summary>
        private float3 lo;
        /// <summary>Values 3-5 (e.g. angular / orientation).</summary>
        private float3 hi;

        // Convenience scalar accessors.
        public float x0 { [MethodImpl(MethodImplOptions.AggressiveInlining)] get => lo.x; [MethodImpl(MethodImplOptions.AggressiveInlining)] set => lo.x = value; }
        public float x1 { [MethodImpl(MethodImplOptions.AggressiveInlining)] get => lo.y; [MethodImpl(MethodImplOptions.AggressiveInlining)] set => lo.y = value; }
        public float x2 { [MethodImpl(MethodImplOptions.AggressiveInlining)] get => lo.z; [MethodImpl(MethodImplOptions.AggressiveInlining)] set => lo.z = value; }
        public float x3 { [MethodImpl(MethodImplOptions.AggressiveInlining)] get => hi.x; [MethodImpl(MethodImplOptions.AggressiveInlining)] set => hi.x = value; }
        public float x4 { [MethodImpl(MethodImplOptions.AggressiveInlining)] get => hi.y; [MethodImpl(MethodImplOptions.AggressiveInlining)] set => hi.y = value; }
        public float x5 { [MethodImpl(MethodImplOptions.AggressiveInlining)] get => hi.z; [MethodImpl(MethodImplOptions.AggressiveInlining)] set => hi.z = value; }

        public float this[int i]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                switch (i)
                {
                    case 0: return lo.x;
                    case 1: return lo.y;
                    case 2: return lo.z;
                    case 3: return hi.x;
                    case 4: return hi.y;
                    case 5: return hi.z;
                    default: return 0f;
                }
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                switch (i)
                {
                    case 0: lo.x = value; break;
                    case 1: lo.y = value; break;
                    case 2: lo.z = value; break;
                    case 3: hi.x = value; break;
                    case 4: hi.y = value; break;
                    case 5: hi.z = value; break;
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float6(float x0, float x1, float x2, float x3, float x4, float x5)
        {
            lo = new float3(x0, x1, x2);
            hi = new float3(x3, x4, x5);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float6(float3 lo, float3 hi)
        {
            this.lo = lo;
            this.hi = hi;
        }

        public static readonly float6 zero = new float6(float3.zero, float3.zero);

        // --- Arithmetic operators (leverage float3 SIMD ops) ---

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float6 operator +(float6 a, float6 b)
            => new float6(a.lo + b.lo, a.hi + b.hi);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float6 operator -(float6 a, float6 b)
            => new float6(a.lo - b.lo, a.hi - b.hi);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float6 operator *(float6 a, float s)
            => new float6(a.lo * s, a.hi * s);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float6 operator *(float s, float6 a)
            => new float6(a.lo * s, a.hi * s);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float6 operator -(float6 a)
            => new float6(-a.lo, -a.hi);

        /// <summary>Dot product.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dot(float6 a, float6 b)
            => math.dot(a.lo, b.lo) + math.dot(a.hi, b.hi);

        /// <summary>Squared magnitude.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float MagnitudeSq() => Dot(this, this);

        /// <summary>Magnitude.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public float Magnitude() => math.sqrt(MagnitudeSq());

        public override string ToString()
            => $"({lo.x:F3}, {lo.y:F3}, {lo.z:F3}, {hi.x:F3}, {hi.y:F3}, {hi.z:F3})";
    }
}
