// =============================================================================
//  matrix_math.cpp  (v2)
// =============================================================================
#include "matrix_math.h"
#include <math.h>
#include <Arduino.h>

// ---------------------------------------------------------------------------
//  Element-wise
// ---------------------------------------------------------------------------
void mat_add(const float *A, const float *B, float *C, int r, int c)
{
    int n = r * c;
    for (int i = 0; i < n; i++) C[i] = A[i] + B[i];
}

void mat_sub(const float *A, const float *B, float *C, int r, int c)
{
    int n = r * c;
    for (int i = 0; i < n; i++) C[i] = A[i] - B[i];
}

void mat_copy(const float *src, float *dst, int r, int c)
{
    memcpy(dst, src, (size_t)(r * c) * sizeof(float));
}

void mat_zero(float *A, int r, int c)
{
    memset(A, 0, (size_t)(r * c) * sizeof(float));
}

void mat_scale(float *A, float s, int r, int c)
{
    int n = r * c;
    for (int i = 0; i < n; i++) A[i] *= s;
}

// ---------------------------------------------------------------------------
//  mat_mul : C(m×p) = A(m×n) × B(n×p)
//
//  Inner-product formulation.  For the EKF the dominant cost is the
//  16×16 × 16×16 product (2·16³ = 8192 FP ops).  At 150 MHz with the
//  M33 FPU executing ~1 FLOP/cycle, this takes ~55 µs — well within the
//  2 ms IMU budget at 500 Hz.
// ---------------------------------------------------------------------------
void mat_mul(const float *A, const float *B, float *C, int m, int n, int p)
{
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            float s = 0.0f;
            for (int k = 0; k < n; k++)
                s += A[i*n + k] * B[k*p + j];
            C[i*p + j] = s;
        }
    }
}

// ---------------------------------------------------------------------------
//  mat_mul_trans_B : C(m×p) = A(m×n) × B(p×n)ᵀ
//
//  Equivalent to C = A × Bᵀ without building the transpose.
//  Accessing B as B[j*n+k] gives row j of B, i.e., column j of Bᵀ.
//
//  Cache behaviour: A is row-sequential (good), B row-sequential (good).
//  This is slightly better than forming the explicit transpose first.
// ---------------------------------------------------------------------------
void mat_mul_trans_B(const float *A, const float *B, float *C,
                     int m, int n, int p)
{
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            float s = 0.0f;
            for (int k = 0; k < n; k++)
                s += A[i*n + k] * B[j*n + k];
            C[i*p + j] = s;
        }
    }
}

// ---------------------------------------------------------------------------
//  mat_transpose : C(n×m) = Aᵀ(m×n)
// ---------------------------------------------------------------------------
void mat_transpose(const float *A, float *C, int m, int n)
{
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            C[j*m + i] = A[i*n + j];
}

// ---------------------------------------------------------------------------
//  mat_sandwich : C = A × B × Aᵀ   (all n×n square)
//  tmp must be caller-supplied n×n scratch.
//  Used for covariance propagation: F · P · Fᵀ
// ---------------------------------------------------------------------------
void mat_sandwich(const float *A, const float *B,
                  float *C, float *tmp, int n)
{
    mat_mul       (A, B,   tmp, n, n, n);   // tmp = A × B
    mat_mul_trans_B(tmp, A, C,  n, n, n);   // C   = tmp × Aᵀ
}

// ---------------------------------------------------------------------------
//  mat_identity
// ---------------------------------------------------------------------------
void mat_identity(float *A, int n)
{
    mat_zero(A, n, n);
    for (int i = 0; i < n; i++) A[i*n + i] = 1.0f;
}

// =============================================================================
//  mat_inv — general n×n matrix inversion via Gauss-Jordan with partial pivot
//
//  Algorithm: augmented matrix [A | I] transformed to [I | A⁻¹] via:
//    1. Partial pivot selection (maximise numerical stability)
//    2. Row swap
//    3. Pivot row normalisation  (divide row by pivot element)
//    4. Column elimination       (subtract scaled pivot row from all others)
//
//  Complexity: O(n³)
//  Stack: 2·n² floats as VLA.  For n=16: 512 floats = 2 KB.
//
//  Singular detection: |pivot| < 1e-12 → return false.
//  Caller should fall back to skipping the update step if false.
//
//  Numerical notes:
//    • Partial pivoting keeps errors bounded to O(n·κ·ε_machine)
//      where κ is the matrix condition number.
//    • For the EKF innovation covariance S = HPHᵀ+R, S is symmetric
//      positive definite by construction, so condition number is bounded
//      and this inversion is reliable.
//    • For production use with very large n or ill-conditioned S, consider
//      Cholesky factorisation (exploits symmetry, more stable, half the ops).
// =============================================================================
bool mat_inv(const float *A, float *Ainv, int n)
{
    const int N2 = 2 * n;
    float aug[n * N2];   // augmented matrix [A | I], VLA on stack

    // Initialise: left = A, right = I
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++)
            aug[i*N2 + j]   = A[i*n + j];
        for (int j = 0; j < n; j++)
            aug[i*N2 + n+j] = (i == j) ? 1.0f : 0.0f;
    }

    for (int col = 0; col < n; col++) {
        // ── Step 1: find pivot row ─────────────────────────────────────────
        int   pivot_row = col;
        float pivot_val = fabsf(aug[col*N2 + col]);
        for (int row = col+1; row < n; row++) {
            float v = fabsf(aug[row*N2 + col]);
            if (v > pivot_val) { pivot_val = v; pivot_row = row; }
        }
        if (pivot_val < 1e-12f) return false;   // singular

        // ── Step 2: swap rows ─────────────────────────────────────────────
        if (pivot_row != col) {
            for (int j = 0; j < N2; j++) {
                float tmp           = aug[col*N2 + j];
                aug[col*N2 + j]     = aug[pivot_row*N2 + j];
                aug[pivot_row*N2+j] = tmp;
            }
        }

        // ── Step 3: normalise pivot row ───────────────────────────────────
        float inv_piv = 1.0f / aug[col*N2 + col];
        for (int j = 0; j < N2; j++) aug[col*N2 + j] *= inv_piv;

        // ── Step 4: eliminate column from all other rows ──────────────────
        for (int row = 0; row < n; row++) {
            if (row == col) continue;
            float fac = aug[row*N2 + col];
            if (fabsf(fac) < 1e-15f) continue;   // already zero
            for (int j = 0; j < N2; j++)
                aug[row*N2 + j] -= fac * aug[col*N2 + j];
        }
    }

    // Extract right-hand side → Ainv
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            Ainv[i*n + j] = aug[i*N2 + n+j];

    return true;
}

// ---------------------------------------------------------------------------
//  Specialisations
// ---------------------------------------------------------------------------
bool mat_inv_1x1(const float *A, float *Ainv)
{
    if (fabsf(A[0]) < 1e-12f) return false;
    Ainv[0] = 1.0f / A[0];
    return true;
}

bool mat_inv_3x3(const float *A, float *Ainv) { return mat_inv(A, Ainv, 3); }
bool mat_inv_6x6(const float *A, float *Ainv) { return mat_inv(A, Ainv, 6); }

// ---------------------------------------------------------------------------
//  mat_frob_norm : Frobenius norm  ||A||_F = sqrt(Σ aᵢⱼ²)
// ---------------------------------------------------------------------------
float mat_frob_norm(const float *A, int r, int c)
{
    float s = 0.0f;
    int n = r * c;
    for (int i = 0; i < n; i++) s += A[i] * A[i];
    return sqrtf(s);
}

// ---------------------------------------------------------------------------
//  mat_is_pd : returns true if all diagonal elements > threshold
//  Quick PD check without full eigendecomposition.
//  Used as filter health monitor — negative diagonal → likely diverged.
// ---------------------------------------------------------------------------
bool mat_is_pd(const float *A, int n, float threshold)
{
    for (int i = 0; i < n; i++)
        if (A[i*n + i] <= threshold) return false;
    return true;
}

// ---------------------------------------------------------------------------
//  mat_print
// ---------------------------------------------------------------------------
void mat_print(const char *label, const float *A, int r, int c)
{
    Serial.print(label);
    Serial.println(":");
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            Serial.print(A[i*c + j], 5);
            Serial.print('\t');
        }
        Serial.println();
    }
}
