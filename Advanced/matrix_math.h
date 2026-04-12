#pragma once
// =============================================================================
//  matrix_math.h  —  Lightweight fixed-size float matrix library  (v2)
//
//  Storage convention: row-major flat float array.
//    M[r * cols + c]  accesses row r, column c.
//
//  Design constraints:
//    • No dynamic allocation  — all temporaries caller-supplied or stack VLA
//    • No external dependencies beyond <string.h> and <math.h>
//    • Targeted at RP2350 Cortex-M33 FPU (single-precision, hardware sqrt)
//
//  Sizes used by 16-state quaternion EKF:
//    State          n = 16
//    GPS meas       p = 6
//    Baro meas      p = 1
//    Mag meas       p = 3
//    Covariance     16×16 = 256 floats = 1024 bytes
//    Jacobian F     16×16
//    Kalman gain    16×6  = 96 floats
//    Innovation cov  6×6  = 36 floats
//
//  Accessor macro (improves readability in EKF code):
// =============================================================================

#include <stdint.h>
#include <string.h>

#define MAT(M, cols, r, c)   ((M)[(r)*(cols) + (c)])

// -----------------------------------------------------------------------------
//  Element-wise operations  (general r×c)
// -----------------------------------------------------------------------------
void mat_add (const float *A, const float *B, float *C, int r, int c);
void mat_sub (const float *A, const float *B, float *C, int r, int c);
void mat_copy(const float *src, float *dst,   int r, int c);
void mat_zero(float *A, int r, int c);
void mat_scale(float *A, float s, int r, int c);

// -----------------------------------------------------------------------------
//  Matrix products
// -----------------------------------------------------------------------------

// C(m×p) = A(m×n) × B(n×p)
void mat_mul(const float *A, const float *B, float *C, int m, int n, int p);

// C(m×p) = A(m×n) × B(p×n)ᵀ   — avoids explicit transpose temp
void mat_mul_trans_B(const float *A, const float *B, float *C,
                     int m, int n, int p);

// C(n×m) = Aᵀ(m×n)
void mat_transpose(const float *A, float *C, int m, int n);

// C(n×n) = A(n×n) × B(n×n) × Aᵀ  — uses caller-supplied tmp(n×n)
// Used for covariance propagation: F·P·Fᵀ
void mat_sandwich(const float *A, const float *B,
                  float *C, float *tmp, int n);

// -----------------------------------------------------------------------------
//  Identity / initialisation
// -----------------------------------------------------------------------------
void mat_identity(float *A, int n);

// -----------------------------------------------------------------------------
//  Inversion — Gauss-Jordan with partial pivoting
//
//  mat_inv(A, Ainv, n):
//    General n×n inversion.  Uses an augmented matrix [A|I] approach.
//    Stack allocation: 2·n² floats.  For n≤16 this is at most 512 floats
//    (2 KB), well within RP2350's 512 KB SRAM.
//    Returns true on success, false if matrix is singular (|pivot| < 1e-12).
//
//  Specialisations (thin wrappers calling mat_inv for clarity):
//    mat_inv_1x1  — for barometer update (scalar)
//    mat_inv_3x3  — for magnetometer update
//    mat_inv_6x6  — for GPS update
// -----------------------------------------------------------------------------
bool mat_inv    (const float *A, float *Ainv, int n);
bool mat_inv_1x1(const float *A, float *Ainv);   // Ainv = 1/A[0]
bool mat_inv_3x3(const float *A, float *Ainv);
bool mat_inv_6x6(const float *A, float *Ainv);

// -----------------------------------------------------------------------------
//  Utility
// -----------------------------------------------------------------------------

// Frobenius norm (for health monitoring, e.g., P diagonal)
float mat_frob_norm(const float *A, int r, int c);

// Check if matrix is positive definite (all diagonal > threshold)
// Used to detect filter divergence
bool mat_is_pd(const float *A, int n, float threshold = 0.0f);

// Debug print to Serial
void mat_print(const char *label, const float *A, int r, int c);
