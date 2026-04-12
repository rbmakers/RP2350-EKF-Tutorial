#pragma once
// =============================================================================
//  ekf_ins.h  —  16-state Quaternion Strapdown INS / GNSS EKF
//
//  State vector  x[16]  (all NED / SI units):
//  ─────────────────────────────────────────────────────────────────────────
//   x[ 0.. 2]  p   = position NED           (m)
//   x[ 3.. 5]  v   = velocity NED           (m/s)
//   x[ 6.. 9]  q   = quaternion [q0,q1,q2,q3]  (Hamilton, scalar-first)
//   x[10..12]  bₐ  = accelerometer bias     (m/s²)
//   x[13..15]  bᵍ  = gyroscope bias         (rad/s)
//
//  WHY QUATERNION over Euler:
//  ─────────────────────────────────────────────────────────────────────────
//   • No gimbal lock (Euler singularity at pitch = ±90°)
//   • Smooth interpolation at all attitudes
//   • Cheaper trig: only 3 divisions replaced by 4 multiplies in C_bn
//   • Numerically better-conditioned Jacobian near ±90° pitch
//
//  Process model (continuous strapdown, in NED frame):
//  ─────────────────────────────────────────────────────────────────────────
//   ṗ = v
//   v̇ = C_bn(q) · (f_imu − bₐ)  +  g_ned
//   q̇ = ½ · Ω(ω̃) · q               where ω̃ = ω_imu − bᵍ
//   ḃₐ = 0   (Gauss-Markov random walk)
//   ḃᵍ = 0
//
//  Ω(ω) = [  0  −ωx −ωy −ωz ]
//          [ ωx   0   ωz −ωy ]
//          [ ωy  −ωz   0   ωx ]
//          [ ωz   ωy −ωx   0  ]
//
//  C_bn (body→NED) from unit quaternion q = [q0,q1,q2,q3]:
//   [q0²+q1²−q2²−q3²,  2(q1q2−q0q3),      2(q1q3+q0q2)    ]
//   [2(q1q2+q0q3),      q0²−q1²+q2²−q3²,   2(q2q3−q0q1)    ]
//   [2(q1q3−q0q2),      2(q2q3+q0q1),       q0²−q1²−q2²+q3²]
//
//  Jacobian F (16×16) discrete = I + Fc·Δt:
//  ─────────────────────────────────────────────────────────────────────────
//   Fc[0:3,  3:6]  = I₃                           dp/dv
//   Fc[3:6,  6:10] = ∂(C_bn·f̃)/∂q  (3×4)         dv/dq  ← analytical
//   Fc[3:6, 10:13] = −C_bn           (3×3)         dv/dbₐ
//   Fc[6:10, 6:10] = ½·Ω(ω̃)         (4×4)         dq/dq
//   Fc[6:10,13:16] = −½·Ξ(q)        (4×3)         dq/dbᵍ
//
//  Ξ(q) = [ −q1  −q2  −q3 ]   (right-quaternion-product matrix)
//          [  q0  −q3   q2 ]
//          [  q3   q0  −q1 ]
//          [ −q2   q1   q0 ]
//
//  Measurement models:
//  ─────────────────────────────────────────────────────────────────────────
//   GPS  (p=6): z=[pN,pE,pD,vN,vE,vD]     H is linear  (6×16)
//   Baro (p=1): z=[pD_deviation]           H is linear  (1×16)
//   Mag  (p=3): z=[bx,by,bz]_body_norm     h is nonlinear → Jacobian computed
//
//  Magnetometer nonlinear measurement model:
//   h(q) = C_bn^T(q) · m̂_ned   (unit NED reference field rotated to body)
//   H_mag[3×4 sub-block] = ∂h/∂q = [D₀^T·m̂ | D₁^T·m̂ | D₂^T·m̂ | D₃^T·m̂]
//   where Dᵢ = ∂C_bn/∂qᵢ (all zero outside quaternion columns)
//
//  Joseph-form covariance update:
//  ─────────────────────────────────────────────────────────────────────────
//   P⁺ = (I−KH)·P⁻·(I−KH)ᵀ + K·R·Kᵀ
//
//   Advantages over simple form  P⁺ = (I−KH)·P⁻:
//   1. Symmetric by construction
//   2. Positive semi-definite by construction
//   3. Numerically stable even when K is sub-optimal
// =============================================================================

#include <Arduino.h>
#include "config.h"
#include "matrix_math.h"
#include "gps_parser.h"

// =============================================================================
//  Helper: Quaternion utilities (all inline, no class)
// =============================================================================

// Normalise quaternion in-place, return norm (for health monitoring)
inline float quat_normalise(float q[4])
{
    float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n > 1e-9f) { q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n; }
    return n;
}

// Quaternion to Euler ZYX (for display/telemetry only, NOT used in EKF)
inline void quat_to_euler(const float q[4],
                           float &roll, float &pitch, float &yaw)
{
    float q0=q[0], q1=q[1], q2=q[2], q3=q[3];
    roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    pitch = asinf (2.0f*(q0*q2 - q3*q1));
    yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
}

// Euler ZYX → initial quaternion for init()
inline void euler_to_quat(float roll, float pitch, float yaw, float q[4])
{
    float cr = cosf(roll*0.5f),  sr = sinf(roll*0.5f);
    float cp = cosf(pitch*0.5f), sp = sinf(pitch*0.5f);
    float cy = cosf(yaw*0.5f),   sy = sinf(yaw*0.5f);
    q[0] = cr*cp*cy + sr*sp*sy;
    q[1] = sr*cp*cy - cr*sp*sy;
    q[2] = cr*sp*cy + sr*cp*sy;
    q[3] = cr*cp*sy - sr*sp*cy;
}

// =============================================================================
//  EKF_INS class
// =============================================================================
class EKF_INS {
public:
    EKF_INS();

    // -------------------------------------------------------------------------
    //  init()  — call after first GPS fix
    //    origin_lat/lon/alt: NED flat-earth origin (LLA, decimal degrees)
    //    init_roll/pitch/yaw: initial attitude estimate (rad)
    // -------------------------------------------------------------------------
    void init(double origin_lat, double origin_lon, float origin_alt,
              float init_roll = 0.0f, float init_pitch = 0.0f,
              float init_yaw  = 0.0f);

    // -------------------------------------------------------------------------
    //  predict()  — call at IMU_SAMPLE_HZ (500 Hz)
    // -------------------------------------------------------------------------
    void predict(float ax, float ay, float az,
                 float gx, float gy, float gz,
                 float dt);

    // -------------------------------------------------------------------------
    //  update_gps()  — position + velocity correction (~5 Hz)
    // -------------------------------------------------------------------------
    void update_gps(const GpsData &fix);

    // -------------------------------------------------------------------------
    //  update_baro()  — altitude correction (~50 Hz)
    //    baro_alt: altitude above MSL from BMP580 (m)
    // -------------------------------------------------------------------------
    void update_baro(float baro_alt_m);

    // -------------------------------------------------------------------------
    //  update_mag()  — heading correction (~100 Hz)
    //    mx/my/mz: normalised body-frame magnetic field (dimensionless)
    //    m_ned:    known normalised NED reference field [mN, mE, mD]
    //              (set from WMM model or via calibration at startup)
    // -------------------------------------------------------------------------
    void update_mag(float mx, float my, float mz,
                    const float m_ned[3]);

    // -------------------------------------------------------------------------
    //  Accessors
    // -------------------------------------------------------------------------
    const float* state()      const { return _x; }
    const float* covariance() const { return _P; }

    // Position
    float pos_n() const { return _x[IDX_PX]; }
    float pos_e() const { return _x[IDX_PY]; }
    float pos_d() const { return _x[IDX_PZ]; }
    // Velocity
    float vel_n() const { return _x[IDX_VX]; }
    float vel_e() const { return _x[IDX_VY]; }
    float vel_d() const { return _x[IDX_VZ]; }
    // Quaternion
    float q0() const { return _x[IDX_Q0]; }
    float q1() const { return _x[IDX_Q1]; }
    float q2() const { return _x[IDX_Q2]; }
    float q3() const { return _x[IDX_Q3]; }
    // Euler (computed from quaternion, for telemetry)
    void get_euler(float &roll, float &pitch, float &yaw) const;

    // 1-sigma position uncertainty (m)
    float sigma_pos_n() const { return sqrtf(fabsf(_P[IDX_PX*EKF_N + IDX_PX])); }
    float sigma_pos_e() const { return sqrtf(fabsf(_P[IDX_PY*EKF_N + IDX_PY])); }
    float sigma_alt()   const { return sqrtf(fabsf(_P[IDX_PZ*EKF_N + IDX_PZ])); }
    float sigma_yaw()   const;  // propagated from quaternion covariance

    bool  is_initialised() const { return _initialised; }

    // LLA → NED (public, used in main for GPS altitude alignment)
    void lla_to_ned(double lat, double lon, float alt,
                    float &n, float &e, float &d) const;

private:
    bool   _initialised;

    // State and covariance
    float  _x[EKF_N];                          // 16
    float  _P[EKF_N * EKF_N];                  // 16×16

    // Noise matrices
    float  _Q[EKF_N * EKF_N];
    float  _R_gps[EKF_P_GPS  * EKF_P_GPS];     // 6×6
    float  _R_baro[EKF_P_BARO * EKF_P_BARO];   // 1×1
    float  _R_mag [EKF_P_MAG  * EKF_P_MAG];    // 3×3

    // Measurement matrix (GPS is pre-built; mag is computed each call)
    float  _H_gps[EKF_P_GPS  * EKF_N];         // 6×16
    float  _H_baro[EKF_P_BARO * EKF_N];        // 1×16

    // NED origin
    double _origin_lat, _origin_lon;
    float  _origin_alt;

    // Pre-allocated working matrices (avoid stack pressure at 500 Hz)
    float  _F[EKF_N * EKF_N];
    float  _K[EKF_N * EKF_P_MAX];              // 16×6
    float  _S[EKF_P_MAX * EKF_P_MAX];          // 6×6
    float  _Sinv[EKF_P_MAX * EKF_P_MAX];
    float  _IKH[EKF_N * EKF_N];
    float  _tmp_nn[EKF_N * EKF_N];             // 16×16
    float  _tmp_np[EKF_N * EKF_P_MAX];         // 16×6
    float  _tmp_pn[EKF_P_MAX * EKF_N];         // 6×16

    // Internal helpers
    void build_Cbn(const float q[4], float Cbn[9]) const;
    void build_Jacobian_F(const float f_corr[3], const float w_corr[3],
                          float dt);
    void build_Q(float dt);

    // Generic Joseph-form update for any measurement dimension p
    // H: p×n, z_innov: p, R_diag: diagonal of R (p values)
    void ekf_update_joseph(const float *H, int p,
                           const float *z_innov, const float *R_diag);

    void symmetrise_P();
    void normalise_quaternion_state();
};
