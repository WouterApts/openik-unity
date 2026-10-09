using System.Runtime.CompilerServices;

namespace OpenIK
{
    /// <summary>
    /// A 6-row, N-column matrix stored as an array of float6 columns.
    /// Used as the Jacobian in IK Solvers: 6 rows (3 position + 3 orientation), N columns (one per DOF).
    /// </summary>
    public class Matrix6xN
    {
        private float6[] _columns;
        private int _n;

        /// <summary>Number of columns (DOFs).</summary>
        public int N => _n;

        public Matrix6xN(int n)
        {
            _n = n;
            _columns = new float6[n];
        }

        public void Resize(int n)
        {
            if (n == _n && _columns != null) return;
            _n = n;
            _columns = new float6[n];
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear()
        {
            for (int i = 0; i < _n; i++)
                _columns[i] = float6.zero;
        }

        /// <summary>Get/set a column.</summary>
        public float6 this[int col]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => _columns[col];
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set => _columns[col] = value;
        }

        /// <summary>Get/set an element.</summary>
        public float this[int row, int col]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => _columns[col][row];
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                var c = _columns[col];
                c[row] = value;
                _columns[col] = c;
            }
        }

        /// <summary>
        /// Computes J * Jᵀ (6×6).
        /// Each element [r,c] = sum over k of column[k][r] * column[k][c],
        /// but builds it as a sum of outer products of each column with itself.
        /// </summary>
        public Matrix6x6 MultiplyJJt()
        {
            Matrix6x6 result = Matrix6x6.zero;

            for (int k = 0; k < _n; k++)
            {
                float6 col = _columns[k];

                // Accumulate col * colᵀ into result (symmetric, but we fill all 6x6)
                result.c0 += col * col.x0;
                result.c1 += col * col.x1;
                result.c2 += col * col.x2;
                result.c3 += col * col.x3;
                result.c4 += col * col.x4;
                result.c5 += col * col.x5;
            }

            return result;
        }

        /// <summary>
        /// Computes Jᵀ * v where v is a float6, producing an N-length result.
        /// Each element i = dot(column[i], v).
        /// </summary>
        /// <param name="v">The vector.</param>
        /// <param name="result">Pre-allocated output array (length >= N).</param>
        public void MultiplyJtVec(float6 v, float[] result)
        {
            for (int i = 0; i < _n; i++)
                result[i] = float6.Dot(_columns[i], v);
        }
    }
}
