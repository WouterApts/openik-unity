using System.Runtime.CompilerServices;
using Unity.Mathematics;

namespace OpenIK
{
    /// <summary>
    /// A stack-allocated 6x6 matrix (column-major order).
    /// Designed for Jacobian IK where the Jacobian is at most 6 rows
    /// (3 position + 3 orientation) by N columns (one per DOF).
    /// </summary>
    public struct Matrix6x6
    {
        // 6 columns, each a float6 (a pair of float3s).
        public float6 c0, c1, c2, c3, c4, c5;

        // m[row, col]
        public float this[int row, int col]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                switch (col)
                {
                    case 0: return c0[row];
                    case 1: return c1[row];
                    case 2: return c2[row];
                    case 3: return c3[row];
                    case 4: return c4[row];
                    case 5: return c5[row];
                    default: return 0f;
                }
            }
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            set
            {
                switch (col)
                {
                    case 0: c0[row] = value; break;
                    case 1: c1[row] = value; break;
                    case 2: c2[row] = value; break;
                    case 3: c3[row] = value; break;
                    case 4: c4[row] = value; break;
                    case 5: c5[row] = value; break;
                }
            }
        }

        public static readonly Matrix6x6 zero = new Matrix6x6();

        public static readonly Matrix6x6 identity = new Matrix6x6
        {
            c0 = new float6(1, 0, 0, 0, 0, 0),
            c1 = new float6(0, 1, 0, 0, 0, 0),
            c2 = new float6(0, 0, 1, 0, 0, 0),
            c3 = new float6(0, 0, 0, 1, 0, 0),
            c4 = new float6(0, 0, 0, 0, 1, 0),
            c5 = new float6(0, 0, 0, 0, 0, 1),
        };

        public Matrix6x6(float6 c0, float6 c1, float6 c2, float6 c3, float6 c4, float6 c5)
        {
            this.c0 = c0;
            this.c1 = c1;
            this.c2 = c2;
            this.c3 = c3;
            this.c4 = c4;
            this.c5 = c5;
        }

        /// <summary>Matrix * vector.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float6 operator *(Matrix6x6 m, float6 v)
        {
            return m.c0 * v.x0 + m.c1 * v.x1 + m.c2 * v.x2
                 + m.c3 * v.x3 + m.c4 * v.x4 + m.c5 * v.x5;
        }

        /// <summary>Matrix * matrix.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix6x6 operator *(Matrix6x6 a, Matrix6x6 b)
        {
            return new Matrix6x6(
                a * b.c0,
                a * b.c1,
                a * b.c2,
                a * b.c3,
                a * b.c4,
                a * b.c5
            );
        }

        /// <summary>Element-wise addition.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix6x6 operator +(Matrix6x6 a, Matrix6x6 b)
        {
            return new Matrix6x6(
                a.c0 + b.c0, a.c1 + b.c1, a.c2 + b.c2,
                a.c3 + b.c3, a.c4 + b.c4, a.c5 + b.c5
            );
        }

        /// <summary>Scalar multiplication.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix6x6 operator *(Matrix6x6 m, float s)
        {
            return new Matrix6x6(
                m.c0 * s, m.c1 * s, m.c2 * s,
                m.c3 * s, m.c4 * s, m.c5 * s
            );
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix6x6 operator *(float s, Matrix6x6 m) => m * s;

        /// <summary>Transpose. Swaps rows and columns.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Matrix6x6 Transpose()
        {
            return new Matrix6x6(
                new float6(c0.x0, c1.x0, c2.x0, c3.x0, c4.x0, c5.x0),
                new float6(c0.x1, c1.x1, c2.x1, c3.x1, c4.x1, c5.x1),
                new float6(c0.x2, c1.x2, c2.x2, c3.x2, c4.x2, c5.x2),
                new float6(c0.x3, c1.x3, c2.x3, c3.x3, c4.x3, c5.x3),
                new float6(c0.x4, c1.x4, c2.x4, c3.x4, c4.x4, c5.x4),
                new float6(c0.x5, c1.x5, c2.x5, c3.x5, c4.x5, c5.x5)
            );
        }

        /// <summary>
        /// Gauss-Jordan inversion with partial pivoting. Returns false if singular.
        /// Prefer Cholesky Invert for symmetric positive-definite matrices (faster).
        /// </summary>
        public bool Invert(out Matrix6x6 result)
        {
            // Augmented matrix [this | I] — work with flat indexing for clarity.
            // We only need the right half at the end.
            float[,] a = new float[6, 12];

            for (int r = 0; r < 6; r++)
            {
                for (int c = 0; c < 6; c++)
                    a[r, c] = this[r, c];
                a[r, r + 6] = 1f;
            }

            for (int col = 0; col < 6; col++)
            {
                // Partial pivot
                int maxRow = col;
                float maxVal = math.abs(a[col, col]);
                for (int r = col + 1; r < 6; r++)
                {
                    float v = math.abs(a[r, col]);
                    if (v > maxVal) { maxVal = v; maxRow = r; }
                }

                if (maxVal < 1e-12f)
                {
                    result = zero;
                    return false;
                }

                // Swap rows
                if (maxRow != col)
                {
                    for (int j = 0; j < 12; j++)
                    {
                        float tmp = a[col, j];
                        a[col, j] = a[maxRow, j];
                        a[maxRow, j] = tmp;
                    }
                }

                // Scale pivot row
                float pivot = 1f / a[col, col];
                for (int j = 0; j < 12; j++)
                    a[col, j] *= pivot;

                // Eliminate column
                for (int r = 0; r < 6; r++)
                {
                    if (r == col) continue;
                    float factor = a[r, col];
                    for (int j = 0; j < 12; j++)
                        a[r, j] -= factor * a[col, j];
                }
            }

            result = new Matrix6x6();
            for (int r = 0; r < 6; r++)
                for (int c = 0; c < 6; c++)
                    result[r, c] = a[r, c + 6];

            return true;
        }

        // ---------------------------------------------------------------
        //  Cholesky decomposition  (for symmetric positive-definite matrices)
        // ---------------------------------------------------------------

        /// <summary>
        /// Cholesky decomposition: A = L * Lᵀ where L is lower-triangular.
        /// Only reads the lower triangle of this matrix (assumes symmetry).
        /// <returns>false if the matrix is not positive-definite.</returns>
        /// </summary>
        public bool Cholesky(out Matrix6x6 L)
        {
            L = zero;

            for (int j = 0; j < 6; j++)
            {
                // Diagonal element: L[j,j] = sqrt(A[j,j] - sum(L[j,k]^2, k<j))
                float sum = 0f;
                for (int k = 0; k < j; k++)
                {
                    float Ljk = L[j, k];
                    sum += Ljk * Ljk;
                }

                float diag = this[j, j] - sum;
                if (diag <= 0f)
                {
                    L = zero;
                    return false;
                }

                float Ljj = math.sqrt(diag);
                L[j, j] = Ljj;

                float invLjj = 1f / Ljj;

                // Off-diagonal elements: L[i,j] = (A[i,j] - sum(L[i,k]*L[j,k], k<j)) / L[j,j]
                for (int i = j + 1; i < 6; i++)
                {
                    float s = 0f;
                    for (int k = 0; k < j; k++)
                        s += L[i, k] * L[j, k];

                    L[i, j] = (this[i, j] - s) * invLjj;
                }
            }

            return true;
        }

        /// <summary>
        /// Solves L * x = b via forward substitution where L is lower-triangular.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float6 ForwardSubstitute(Matrix6x6 L, float6 b)
        {
            float6 x = float6.zero;
            for (int i = 0; i < 6; i++)
            {
                float sum = 0f;
                for (int k = 0; k < i; k++)
                    sum += L[i, k] * x[k];

                x[i] = (b[i] - sum) / L[i, i];
            }
            return x;
        }

        /// <summary>
        /// Solves Lᵀ * x = b via backward substitution where L is lower-triangular.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float6 BackwardSubstitute(Matrix6x6 L, float6 b)
        {
            float6 x = float6.zero;
            for (int i = 5; i >= 0; i--)
            {
                float sum = 0f;
                for (int k = i + 1; k < 6; k++)
                    sum += L[k, i] * x[k]; // L[k,i] = Lᵀ[i,k]

                x[i] = (b[i] - sum) / L[i, i];
            }
            return x;
        }

        /// <summary>
        /// Solves A * x = b where A is symmetric positive-definite, via Cholesky.
        /// Equivalent to x = A⁻¹ * b, but without forming the full inverse.
        /// <returns>false if A is not positive-definite.</returns>
        /// </summary>
        public bool CholeskySolve(float6 b, out float6 x)
        {
            if (!Cholesky(out Matrix6x6 L))
            {
                x = float6.zero;
                return false;
            }

            // A = LLᵀ  →  LLᵀx = b  →  Ly = b (forward), then Lᵀx = y (backward)
            float6 y = ForwardSubstitute(L, b);
            x = BackwardSubstitute(L, y);
            return true;
        }

        /// <summary>
        /// Inverts a symmetric positive-definite matrix via Cholesky decomposition.
        /// </summary>
        public bool CholeskyInvert(out Matrix6x6 result)
        {
            if (!Cholesky(out Matrix6x6 L))
            {
                result = zero;
                return false;
            }

            // Solve L * Lᵀ * X = I column by column.
            result = new Matrix6x6();
            for (int col = 0; col < 6; col++)
            {
                // Unit basis vector for this column
                float6 e = float6.zero;
                e[col] = 1f;

                float6 y = ForwardSubstitute(L, e);
                float6 x = BackwardSubstitute(L, y);

                // Write solution into the result column
                switch (col)
                {
                    case 0: result.c0 = x; break;
                    case 1: result.c1 = x; break;
                    case 2: result.c2 = x; break;
                    case 3: result.c3 = x; break;
                    case 4: result.c4 = x; break;
                    case 5: result.c5 = x; break;
                }
            }

            return true;
        }
    }
}
