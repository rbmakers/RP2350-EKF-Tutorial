// =============================================================================
//  matrix_math.cpp
// =============================================================================
#include "matrix_math.h"
#include <math.h>
#include <Arduino.h>   // for Serial in mat_print

// -----------------------------------------------------------------------------
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

// C(m×p) = A(m×n) × B(n×p)
void mat_mul(const float *A, const float *B, float *C, int m, int n, int p)
{
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            float sum = 0.0f;
            for (int k = 0; k < n; k++)
                sum += A[i*n + k] * B[k*p + j];
            C[i*p + j] = sum;
        }
    }
}

// C(m×p) = A(m×n) × B(p×n)ᵀ   — avoids explicit transpose allocation
void mat_mul_trans_B(const float *A, const float *B, float *C,
                     int m, int n, int p)
{
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            float sum = 0.0f;
            for (int k = 0; k < n; k++)
                sum += A[i*n + k] * B[j*n + k];   // B indexed transposed
            C[i*p + j] = sum;
        }
    }
}

// C(c×r) = Aᵀ(r×c)
void mat_transpose(const float *A, float *C, int r, int c)
{
    for (int i = 0; i < r; i++)
        for (int j = 0; j < c; j++)
            C[j*r + i] = A[i*c + j];
}

// D(n×n) = A(n×n) × B(n×n) × Aᵀ(n×n)   — uses caller-supplied tmp(n×n)
void mat_sandwich(const float *A, const float *B, float *C, float *tmp, int n)
{
    mat_mul(A, B, tmp, n, n, n);          // tmp = A × B
    mat_mul_trans_B(tmp, A, C, n, n, n);  // C   = tmp × Aᵀ
}

void mat_identity(float *A, int n)
{
    mat_zero(A, n, n);
    for (int i = 0; i < n; i++) A[i*n + i] = 1.0f;
}

void mat_zero(float *A, int r, int c)
{
    memset(A, 0, r * c * sizeof(float));
}

void mat_copy(const float *src, float *dst, int r, int c)
{
    memcpy(dst, src, r * c * sizeof(float));
}

void mat_scale(float *A, float scalar, int r, int c)
{
    int n = r * c;
    for (int i = 0; i < n; i++) A[i] *= scalar;
}

// -----------------------------------------------------------------------------
//  Gauss-Jordan in-place inversion (general n×n)
//  Augmented matrix [A | I] → [I | A⁻¹]
//  Returns false if matrix is singular (pivot < EPS).
// -----------------------------------------------------------------------------
bool mat_inv(const float *A, float *Ainv, int n)
{
    // Build augmented matrix row-major: aug[i][j] for j in [0, 2n)
    const int N2 = 2 * n;
    float aug[n * N2];   // VLA — small n (≤15) is fine on RP2350 stack

    // Left half = A, right half = I
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++)  aug[i*N2 + j]   = A[i*n + j];
        for (int j = 0; j < n; j++)  aug[i*N2 + n+j] = (i == j) ? 1.0f : 0.0f;
    }

    // Forward elimination with partial pivoting
    for (int col = 0; col < n; col++) {
        // Find pivot row
        int pivot = col;
        float best = fabsf(aug[col*N2 + col]);
        for (int row = col+1; row < n; row++) {
            float v = fabsf(aug[row*N2 + col]);
            if (v > best) { best = v; pivot = row; }
        }
        if (best < 1e-12f) return false;  // singular

        // Swap rows
        if (pivot != col) {
            for (int j = 0; j < N2; j++) {
                float tmp = aug[col*N2 + j];
                aug[col*N2 + j]   = aug[pivot*N2 + j];
                aug[pivot*N2 + j] = tmp;
            }
        }

        // Scale pivot row
        float inv_pivot = 1.0f / aug[col*N2 + col];
        for (int j = 0; j < N2; j++) aug[col*N2 + j] *= inv_pivot;

        // Eliminate column
        for (int row = 0; row < n; row++) {
            if (row == col) continue;
            float factor = aug[row*N2 + col];
            for (int j = 0; j < N2; j++)
                aug[row*N2 + j] -= factor * aug[col*N2 + j];
        }
    }

    // Extract right half → Ainv
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            Ainv[i*n + j] = aug[i*N2 + n+j];

    return true;
}

bool mat_inv_3x3(const float *A, float *Ainv)  { return mat_inv(A, Ainv, 3); }
bool mat_inv_6x6(const float *A, float *Ainv)  { return mat_inv(A, Ainv, 6); }

// -----------------------------------------------------------------------------
//  Debug print
// -----------------------------------------------------------------------------
void mat_print(const char *label, const float *A, int r, int c)
{
    Serial.print(label);
    Serial.println(":");
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            Serial.print(A[i*c + j], 6);
            Serial.print('\t');
        }
        Serial.println();
    }
}
