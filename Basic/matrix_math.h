#pragma once
// =============================================================================
//  matrix_math.h  —  Lightweight fixed-size float matrix library
//
//  All matrices are stored row-major in flat float arrays:
//      M[r * cols + c]  accesses row r, column c
//
//  No dynamic allocation.  No external dependencies.
//  Designed for EKF on RP2350 (Cortex-M33 with FPU).
// =============================================================================

#include <stdint.h>
#include <string.h>   // memset, memcpy

// Convenience accessor macro (optional, improves readability)
#define MAT(M, cols, r, c)   ((M)[(r)*(cols) + (c)])

// -----------------------------------------------------------------------------
//  Basic operations — general (runtime rows/cols)
// -----------------------------------------------------------------------------

// C = A + B   (r × c)
void mat_add(const float *A, const float *B, float *C, int r, int c);

// C = A − B   (r × c)
void mat_sub(const float *A, const float *B, float *C, int r, int c);

// C = A × B   (m×n) × (n×p) → (m×p)
void mat_mul(const float *A, const float *B, float *C, int m, int n, int p);

// C = Aᵀ      (r×c) → (c×r)
void mat_transpose(const float *A, float *C, int r, int c);

// C = A × Bᵀ  shorthand used heavily in KF: P·Hᵀ, F·P·Fᵀ
void mat_mul_trans_B(const float *A, const float *B, float *C,
                     int m, int n, int p);

// D = A × B × Aᵀ  (used in covariance propagation: F·P·Fᵀ)
void mat_sandwich(const float *A, const float *B, float *C, float *tmp,
                  int n);   // square n×n only

// Set to identity
void mat_identity(float *A, int n);

// Set to zero
void mat_zero(float *A, int r, int c);

// Copy
void mat_copy(const float *src, float *dst, int r, int c);

// Scale: A = A × scalar
void mat_scale(float *A, float scalar, int r, int c);

// -----------------------------------------------------------------------------
//  Specialised small-matrix inversions (Gauss-Jordan, in-place)
//  Returns true on success, false if singular.
// -----------------------------------------------------------------------------

// Invert 3×3 matrix (used internally)
bool mat_inv_3x3(const float *A, float *Ainv);

// Invert 6×6 matrix  ← used to invert innovation covariance S in EKF update
bool mat_inv_6x6(const float *A, float *Ainv);

// Invert n×n matrix  (Gauss-Jordan elimination, general)
bool mat_inv(const float *A, float *Ainv, int n);

// -----------------------------------------------------------------------------
//  Debug print (writes to Serial — call only from Core 0)
// -----------------------------------------------------------------------------
void mat_print(const char *label, const float *A, int r, int c);
