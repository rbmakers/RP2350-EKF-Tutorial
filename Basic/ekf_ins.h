#pragma once
// =============================================================================
//  ekf_ins.h  —  15-state strapdown INS / GNSS EKF
//
//  State vector  x[15]  (all in NED / SI units):
//  ─────────────────────────────────────────────
//   [0..2]   p = position       (m, NED from origin)
//   [3..5]   v = velocity       (m/s, NED)
//   [6..8]   η = Euler angles   (rad, ZYX: roll φ, pitch θ, yaw ψ)
//   [9..11]  bₐ= accel bias     (m/s²)
//  [12..14]  bᵍ= gyro bias      (rad/s)
//
//  Process model (continuous strapdown equations):
//  ─────────────────────────────────────────────
//   ṗ = v
//   v̇ = C_bn(η) · (fᵢₘᵤ − bₐ)  +  gₙₑ_d
//   η̇ = T(η)   · (ωᵢₘᵤ − bᵍ)
//   ḃₐ = 0  (Gauss-Markov; treated as random walk here)
//   ḃᵍ = 0
//
//  Where:
//   C_bn  = body → NED rotation matrix (function of η)
//   T(η)  = Euler rate kinematics matrix
//   fᵢₘᵤ  = accelerometer measurement (m/s²) in body frame
//   ωᵢₘᵤ  = gyroscope measurement     (rad/s) in body frame
//   gₙₑ_d = [0, 0, +g]  gravity in NED  (+Down in NED convention)
//
//  Jacobian F (15×15) — analytical linearisation:
//  ─────────────────────────────────────────────
//   F = I₁₅  +  Fc · Δt          (first-order ZOH discretisation)
//
//   Fc block structure:
//     Fc[0:3, 3:6]  = I₃               dp/dv
//     Fc[3:6, 6:9]  = ∂(C_bn·f̃)/∂η   dv/dη  (f̃ = fᵢₘᵤ − bₐ)
//     Fc[3:6, 9:12] = −C_bn            dv/dbₐ
//     Fc[6:9, 6:9]  = ∂(T·ω̃)/∂η     dη/dη  (ω̃ = ωᵢₘᵤ − bᵍ)
//     Fc[6:9,12:15] = −T               dη/dbᵍ
//
//  GPS measurement model (p = 6):
//  ─────────────────────────────────────────────
//   z = [pN, pE, pD, vN, vE, vD]
//   H = [ I₃  0  0  0  0 ]   (6×15)
//       [ 0  I₃  0  0  0 ]
//   This is linear, so H is exact (no Jacobian needed).
// =============================================================================

#include <Arduino.h>
#include "config.h"
#include "matrix_math.h"
#include "gps_parser.h"

class EKF_INS {
public:
    EKF_INS();

    // -------------------------------------------------------------------------
    //  Initialise EKF state and covariance.
    //  Call once after obtaining first GPS fix.
    //    origin_lat/lon: LLA origin for NED flat-earth frame (decimal degrees)
    //    init_yaw:       initial heading estimate (rad) — e.g. from magnetometer
    // -------------------------------------------------------------------------
    void init(double origin_lat, double origin_lon, float origin_alt,
              float init_yaw = 0.0f);

    // -------------------------------------------------------------------------
    //  Predict step — call at IMU_SAMPLE_HZ (500 Hz)
    //    accel: specific force in body frame  (m/s²)
    //    gyro:  angular rate in body frame    (rad/s)
    //    dt:    integration timestep          (s)
    // -------------------------------------------------------------------------
    void predict(float ax, float ay, float az,
                 float gx, float gy, float gz,
                 float dt);

    // -------------------------------------------------------------------------
    //  Update step — call when a new GPS fix arrives (~5–10 Hz)
    //    fix:  parsed GPS data in LLA + NED velocity
    // -------------------------------------------------------------------------
    void update_gps(const GpsData &fix);

    // -------------------------------------------------------------------------
    //  Accessors
    // -------------------------------------------------------------------------
    const float* state()       const { return _x; }   // [15]
    const float* covariance()  const { return _P; }   // [15×15]

    float pos_n()   const { return _x[IDX_PX]; }
    float pos_e()   const { return _x[IDX_PY]; }
    float pos_d()   const { return _x[IDX_PZ]; }
    float vel_n()   const { return _x[IDX_VX]; }
    float vel_e()   const { return _x[IDX_VY]; }
    float vel_d()   const { return _x[IDX_VZ]; }
    float roll()    const { return _x[IDX_ROLL];  }
    float pitch()   const { return _x[IDX_PITCH]; }
    float yaw()     const { return _x[IDX_YAW];   }

    // 1-sigma position uncertainty (m)
    float sigma_pos_n() const { return sqrtf(_P[IDX_PX*EKF_N + IDX_PX]); }
    float sigma_pos_e() const { return sqrtf(_P[IDX_PY*EKF_N + IDX_PY]); }

    bool  is_initialised() const { return _initialised; }

    // Convert LLA fix to NED relative to stored origin
    void lla_to_ned(double lat, double lon, float alt,
                    float &north, float &east, float &down) const;

private:
    bool   _initialised;

    // EKF state and covariance
    float  _x[EKF_N];             // 15×1 state vector
    float  _P[EKF_N * EKF_N];     // 15×15 state covariance

    // Process noise (diagonal, precomputed)
    float  _Q[EKF_N * EKF_N];     // 15×15

    // Measurement noise (diagonal, fixed)
    float  _R[EKF_P * EKF_P];     // 6×6

    // NED frame origin (set once at init)
    double _origin_lat;
    double _origin_lon;
    float  _origin_alt;

    // Working matrices (pre-allocated to avoid stack pressure in tight loop)
    float  _F[EKF_N * EKF_N];     // Jacobian F
    float  _Fwork[EKF_N * EKF_N]; // scratch
    float  _H[EKF_P * EKF_N];     // measurement matrix
    float  _K[EKF_N * EKF_P];     // Kalman gain
    float  _S[EKF_P * EKF_P];     // innovation covariance
    float  _Sinv[EKF_P * EKF_P];  // S⁻¹
    float  _tmp1[EKF_N * EKF_N];
    float  _tmp2[EKF_N * EKF_P];
    float  _tmp3[EKF_P * EKF_N];

    // Internal helpers
    void build_Cbn(float roll, float pitch, float yaw,
                   float Cbn[9]) const;

    void build_T(float roll, float pitch, float yaw,
                 float T[9]) const;

    void build_Jacobian_F(float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float dt);

    void build_Q(float dt);

    // Symmetrise P to prevent numerical drift
    void symmetrise_P();

    // Angle wrapping utility
    static float wrap_pi(float angle);
};
