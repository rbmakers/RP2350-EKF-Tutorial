// =============================================================================
//  ekf_ins.cpp  —  16-state Quaternion INS / GNSS EKF
//                  with Joseph-form covariance update
// =============================================================================
#include "ekf_ins.h"
#include <math.h>
#include <string.h>

// ---------------------------------------------------------------------------
//  Constructor
// ---------------------------------------------------------------------------
EKF_INS::EKF_INS() : _initialised(false),
    _origin_lat(0), _origin_lon(0), _origin_alt(0)
{
    mat_zero(_x, EKF_N, 1);
    mat_zero(_P, EKF_N, EKF_N);
}

// ---------------------------------------------------------------------------
//  init()
// ---------------------------------------------------------------------------
void EKF_INS::init(double origin_lat, double origin_lon, float origin_alt,
                   float init_roll, float init_pitch, float init_yaw)
{
    _origin_lat = origin_lat;
    _origin_lon = origin_lon;
    _origin_alt = origin_alt;

    // ── State vector ─────────────────────────────────────────────────────────
    mat_zero(_x, EKF_N, 1);

    // Convert initial Euler angles to quaternion
    float q0[4];
    euler_to_quat(init_roll, init_pitch, init_yaw, q0);
    _x[IDX_Q0] = q0[0];
    _x[IDX_Q1] = q0[1];
    _x[IDX_Q2] = q0[2];
    _x[IDX_Q3] = q0[3];

    // ── Initial covariance P₀  (diagonal) ────────────────────────────────────
    mat_zero(_P, EKF_N, EKF_N);
    _P[IDX_PX*EKF_N + IDX_PX] = P0_POS;
    _P[IDX_PY*EKF_N + IDX_PY] = P0_POS;
    _P[IDX_PZ*EKF_N + IDX_PZ] = P0_POS;
    _P[IDX_VX*EKF_N + IDX_VX] = P0_VEL;
    _P[IDX_VY*EKF_N + IDX_VY] = P0_VEL;
    _P[IDX_VZ*EKF_N + IDX_VZ] = P0_VEL;
    for (int i = IDX_Q0; i <= IDX_Q3; i++) _P[i*EKF_N + i] = P0_QUAT;
    for (int i = IDX_BAX; i <= IDX_BAZ; i++) _P[i*EKF_N + i] = P0_BIAS_A;
    for (int i = IDX_BGX; i <= IDX_BGZ; i++) _P[i*EKF_N + i] = P0_BIAS_G;

    // ── Measurement matrices (fixed, linear sensors) ──────────────────────────

    // H_gps (6×16): maps state → [pN,pE,pD,vN,vE,vD]
    mat_zero(_H_gps, EKF_P_GPS, EKF_N);
    _H_gps[0*EKF_N + IDX_PX] = 1.0f;
    _H_gps[1*EKF_N + IDX_PY] = 1.0f;
    _H_gps[2*EKF_N + IDX_PZ] = 1.0f;
    _H_gps[3*EKF_N + IDX_VX] = 1.0f;
    _H_gps[4*EKF_N + IDX_VY] = 1.0f;
    _H_gps[5*EKF_N + IDX_VZ] = 1.0f;

    // H_baro (1×16): z = pD_deviation = x[IDX_PZ]
    // (positive Down in NED, altitude = origin_alt − pD)
    mat_zero(_H_baro, EKF_P_BARO, EKF_N);
    _H_baro[0*EKF_N + IDX_PZ] = 1.0f;  // ∂(pD)/∂pD = 1

    // ── Measurement noise R (diagonal values) ─────────────────────────────────

    // R_gps (6×6)
    mat_zero(_R_gps, EKF_P_GPS, EKF_P_GPS);
    _R_gps[0*6+0] = R_GPS_POS_H;
    _R_gps[1*6+1] = R_GPS_POS_H;
    _R_gps[2*6+2] = R_GPS_POS_V;
    _R_gps[3*6+3] = R_GPS_VEL_H;
    _R_gps[4*6+4] = R_GPS_VEL_H;
    _R_gps[5*6+5] = R_GPS_VEL_V;

    // R_baro (1×1)
    _R_baro[0] = R_BARO_ALT;

    // R_mag (3×3)
    mat_zero(_R_mag, EKF_P_MAG, EKF_P_MAG);
    _R_mag[0*3+0] = R_MAG;
    _R_mag[1*3+1] = R_MAG;
    _R_mag[2*3+2] = R_MAG;

    _initialised = true;
    DEBUG_SERIAL.println("[EKF] Quaternion INS/GNSS filter initialised.");
}

// =============================================================================
//  PREDICT  (500 Hz)
// =============================================================================
void EKF_INS::predict(float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float dt)
{
    if (!_initialised) return;

    const float q0 = _x[IDX_Q0], q1 = _x[IDX_Q1];
    const float q2 = _x[IDX_Q2], q3 = _x[IDX_Q3];

    // ── Bias-corrected IMU measurements ───────────────────────────────────────
    float fx = ax - _x[IDX_BAX];
    float fy = ay - _x[IDX_BAY];
    float fz = az - _x[IDX_BAZ];
    float wx = gx - _x[IDX_BGX];
    float wy = gy - _x[IDX_BGY];
    float wz = gz - _x[IDX_BGZ];

    // ── Body → NED rotation: a_ned = C_bn(q) · f_body ────────────────────────
    float Cbn[9];
    build_Cbn(_x + IDX_Q0, Cbn);

    float an_x = Cbn[0]*fx + Cbn[1]*fy + Cbn[2]*fz;
    float an_y = Cbn[3]*fx + Cbn[4]*fy + Cbn[5]*fz;
    float an_z = Cbn[6]*fx + Cbn[7]*fy + Cbn[8]*fz + GRAVITY_MS2;  // add g

    // ── Quaternion kinematics:  q_dot = ½·Ω(ω̃)·q ────────────────────────────
    //  q_dot = ½ × [ 0  -wx -wy -wz ] [q0]
    //              [ wx  0   wz -wy ] [q1]
    //              [ wy -wz  0   wx ] [q2]
    //              [ wz  wy -wx  0  ] [q3]
    float q0d = 0.5f*(-wx*q1 - wy*q2 - wz*q3);
    float q1d = 0.5f*( wx*q0 + wz*q2 - wy*q3);   // note sign convention
    float q2d = 0.5f*( wy*q0 - wz*q1 + wx*q3);
    float q3d = 0.5f*( wz*q0 + wy*q1 - wx*q2);

    // ── Exact quaternion integration via rotation vector ──────────────────────
    // For small dt, (I + ½Ω·dt)·q is sufficient but accumulates normalisation
    // error.  Use exact formula:
    //   δq = [cos(|ω|dt/2), ω̂·sin(|ω|dt/2)]
    //   q_new = δq ⊗ q_old
    float angle = sqrtf(wx*wx + wy*wy + wz*wz) * dt;
    float new_q0, new_q1, new_q2, new_q3;

    if (angle > 1e-7f) {
        float s = sinf(0.5f * angle) / (angle / dt);
        float c = cosf(0.5f * angle);
        float dq0 = c;
        float dq1 = wx * dt * s;
        float dq2 = wy * dt * s;
        float dq3 = wz * dt * s;
        // q_new = δq ⊗ q  (Hamilton product, left multiplication)
        new_q0 = dq0*q0 - dq1*q1 - dq2*q2 - dq3*q3;
        new_q1 = dq0*q1 + dq1*q0 + dq2*q3 - dq3*q2;
        new_q2 = dq0*q2 - dq1*q3 + dq2*q0 + dq3*q1;
        new_q3 = dq0*q3 + dq1*q2 - dq2*q1 + dq3*q0;
    } else {
        // First-order approximation for very small rotations
        new_q0 = q0 + q0d * dt;
        new_q1 = q1 + q1d * dt;
        new_q2 = q2 + q2d * dt;
        new_q3 = q3 + q3d * dt;
    }

    // ── Integrate state ───────────────────────────────────────────────────────
    _x[IDX_PX] += _x[IDX_VX] * dt;
    _x[IDX_PY] += _x[IDX_VY] * dt;
    _x[IDX_PZ] += _x[IDX_VZ] * dt;

    _x[IDX_VX] += an_x * dt;
    _x[IDX_VY] += an_y * dt;
    _x[IDX_VZ] += an_z * dt;

    _x[IDX_Q0] = new_q0;
    _x[IDX_Q1] = new_q1;
    _x[IDX_Q2] = new_q2;
    _x[IDX_Q3] = new_q3;
    // Biases unchanged (random walk mean = 0)

    // Force unit quaternion (numerical drift guard)
    normalise_quaternion_state();

    // ── Build F (Jacobian) and Q ──────────────────────────────────────────────
    float f_corr[3] = { fx, fy, fz };
    float w_corr[3] = { wx, wy, wz };
    build_Jacobian_F(f_corr, w_corr, dt);
    build_Q(dt);

    // ── Covariance propagation: P⁻ = F·P·Fᵀ + Q ─────────────────────────────
    // tmp_nn = F · P   (16×16)
    mat_mul(_F, _P, _tmp_nn, EKF_N, EKF_N, EKF_N);
    // P⁻ = tmp_nn · Fᵀ = F·P·Fᵀ
    mat_mul_trans_B(_tmp_nn, _F, _P, EKF_N, EKF_N, EKF_N);
    // P⁻ += Q
    mat_add(_P, _Q, _P, EKF_N, EKF_N);

    symmetrise_P();
}

// =============================================================================
//  PRIVATE: build_Cbn — C_bn (body→NED) from unit quaternion
//
//  C_bn = (q0²+q1²−q2²−q3²)·I  +  2q0·[q_vec]×  +  2·q_vec·q_vecᵀ
//
//  Expanded explicitly:
//   C[0,0] = q0²+q1²−q2²−q3²    C[0,1] = 2(q1q2−q0q3)    C[0,2] = 2(q1q3+q0q2)
//   C[1,0] = 2(q1q2+q0q3)        C[1,1] = q0²−q1²+q2²−q3² C[1,2] = 2(q2q3−q0q1)
//   C[2,0] = 2(q1q3−q0q2)        C[2,1] = 2(q2q3+q0q1)    C[2,2] = q0²−q1²−q2²+q3²
// =============================================================================
void EKF_INS::build_Cbn(const float q[4], float C[9]) const
{
    float q0=q[0], q1=q[1], q2=q[2], q3=q[3];

    C[0] = q0*q0+q1*q1-q2*q2-q3*q3;  C[1] = 2.0f*(q1*q2-q0*q3);        C[2] = 2.0f*(q1*q3+q0*q2);
    C[3] = 2.0f*(q1*q2+q0*q3);        C[4] = q0*q0-q1*q1+q2*q2-q3*q3;  C[5] = 2.0f*(q2*q3-q0*q1);
    C[6] = 2.0f*(q1*q3-q0*q2);        C[7] = 2.0f*(q2*q3+q0*q1);        C[8] = q0*q0-q1*q1-q2*q2+q3*q3;
}

// =============================================================================
//  PRIVATE: build_Jacobian_F — discrete 16×16 Jacobian F = I + Fc·dt
//
//  f_corr: bias-corrected specific force  [fx,fy,fz]  (body frame)
//  w_corr: bias-corrected angular rate    [wx,wy,wz]  (body frame)
// =============================================================================
void EKF_INS::build_Jacobian_F(const float f_corr[3], const float w_corr[3],
                                float dt)
{
    const float q0=_x[IDX_Q0], q1=_x[IDX_Q1];
    const float q2=_x[IDX_Q2], q3=_x[IDX_Q3];
    const float fx=f_corr[0],  fy=f_corr[1],  fz=f_corr[2];
    const float wx=w_corr[0],  wy=w_corr[1],  wz=w_corr[2];

    // Start with identity
    mat_identity(_F, EKF_N);

    // ── Block (0:3, 3:6)  dp/dv = I·dt ───────────────────────────────────────
    _F[IDX_PX*EKF_N + IDX_VX] = dt;
    _F[IDX_PY*EKF_N + IDX_VY] = dt;
    _F[IDX_PZ*EKF_N + IDX_VZ] = dt;

    // ── Block (3:6, 6:10) dv/dq = ∂(Cbn·f̃)/∂q · dt ──────────────────────────
    //
    //  Analytical Jacobian. Each column col_i contains:
    //     ∂(Cbn·f̃)/∂qᵢ = (∂Cbn/∂qᵢ) · f̃
    //
    //  Derived from the explicit Cbn formula above, differentiating each
    //  element with respect to q0, q1, q2, q3.  Full derivation in header.
    //
    //  Column 0  (∂/∂q0):
    float dv_dq0_0 =  2.0f*(q0*fx - q3*fy + q2*fz);
    float dv_dq0_1 =  2.0f*(q3*fx + q0*fy - q1*fz);
    float dv_dq0_2 =  2.0f*(-q2*fx + q1*fy + q0*fz);

    //  Column 1  (∂/∂q1):
    float dv_dq1_0 =  2.0f*(q1*fx + q2*fy + q3*fz);
    float dv_dq1_1 =  2.0f*(q2*fx - q1*fy - q0*fz);
    float dv_dq1_2 =  2.0f*(q3*fx + q0*fy - q1*fz);

    //  Column 2  (∂/∂q2):
    float dv_dq2_0 =  2.0f*(-q2*fx + q1*fy + q0*fz);   // typo check: matches D2·f
    float dv_dq2_1 =  2.0f*( q1*fx + q2*fy + q3*fz);
    float dv_dq2_2 =  2.0f*(-q0*fx + q3*fy - q2*fz);

    // Wait — recheck column 2 row 0:
    // ∂v₀/∂q2 = -2q2*fx + 2q1*fy + 2q0*fz  ← ∂C[0,0]/∂q2*fx + ∂C[0,1]/∂q2*fy + ∂C[0,2]/∂q2*fz
    //   ∂C[0,0]/∂q2 = -2q2, ∂C[0,1]/∂q2 = 2q1, ∂C[0,2]/∂q2 = 2q0  ✓ → -2q2*fx + 2q1*fy + 2q0*fz
    //   But I wrote (-q2*fx + q1*fy + q0*fz)*2 above — that matches ✓

    //  Column 3  (∂/∂q3):
    float dv_dq3_0 =  2.0f*(-q3*fx - q0*fy + q1*fz);
    float dv_dq3_1 =  2.0f*( q0*fx - q3*fy + q2*fz);
    float dv_dq3_2 =  2.0f*( q1*fx + q2*fy + q3*fz);

    // Store block (3:6, 6:10) = dv/dq · dt
    _F[IDX_VX*EKF_N + IDX_Q0] = dv_dq0_0 * dt;
    _F[IDX_VY*EKF_N + IDX_Q0] = dv_dq0_1 * dt;
    _F[IDX_VZ*EKF_N + IDX_Q0] = dv_dq0_2 * dt;

    _F[IDX_VX*EKF_N + IDX_Q1] = dv_dq1_0 * dt;
    _F[IDX_VY*EKF_N + IDX_Q1] = dv_dq1_1 * dt;
    _F[IDX_VZ*EKF_N + IDX_Q1] = dv_dq1_2 * dt;

    _F[IDX_VX*EKF_N + IDX_Q2] = dv_dq2_0 * dt;
    _F[IDX_VY*EKF_N + IDX_Q2] = dv_dq2_1 * dt;
    _F[IDX_VZ*EKF_N + IDX_Q2] = dv_dq2_2 * dt;

    _F[IDX_VX*EKF_N + IDX_Q3] = dv_dq3_0 * dt;
    _F[IDX_VY*EKF_N + IDX_Q3] = dv_dq3_1 * dt;
    _F[IDX_VZ*EKF_N + IDX_Q3] = dv_dq3_2 * dt;

    // ── Block (3:6, 10:13) dv/dbₐ = −Cbn · dt ────────────────────────────────
    float Cbn[9];
    build_Cbn(_x + IDX_Q0, Cbn);

    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            _F[(IDX_VX+r)*EKF_N + (IDX_BAX+c)] = -Cbn[r*3+c] * dt;

    // ── Block (6:10, 6:10) dq/dq = ½·Ω(ω̃) · dt ──────────────────────────────
    //
    //  Ω(ω̃) = [  0  −wx −wy −wz ]
    //          [ wx   0   wz −wy ]
    //          [ wy  −wz   0   wx ]
    //          [ wz   wy −wx   0  ]
    //
    //  F sub-block = I₄ + ½·Ω(ω̃)·dt  (I₄ already set from mat_identity above)
    float h = 0.5f * dt;
    _F[IDX_Q0*EKF_N + IDX_Q1] += -wx * h;
    _F[IDX_Q0*EKF_N + IDX_Q2] += -wy * h;
    _F[IDX_Q0*EKF_N + IDX_Q3] += -wz * h;

    _F[IDX_Q1*EKF_N + IDX_Q0] +=  wx * h;
    _F[IDX_Q1*EKF_N + IDX_Q2] +=  wz * h;
    _F[IDX_Q1*EKF_N + IDX_Q3] += -wy * h;

    _F[IDX_Q2*EKF_N + IDX_Q0] +=  wy * h;
    _F[IDX_Q2*EKF_N + IDX_Q1] += -wz * h;
    _F[IDX_Q2*EKF_N + IDX_Q3] +=  wx * h;

    _F[IDX_Q3*EKF_N + IDX_Q0] +=  wz * h;
    _F[IDX_Q3*EKF_N + IDX_Q1] +=  wy * h;
    _F[IDX_Q3*EKF_N + IDX_Q2] += -wx * h;

    // ── Block (6:10, 13:16) dq/dbᵍ = −½·Ξ(q) · dt ───────────────────────────
    //
    //  Ξ(q) = [ −q1  −q2  −q3 ]
    //         [  q0  −q3   q2 ]
    //         [  q3   q0  −q1 ]
    //         [ −q2   q1   q0 ]
    //
    //  Block = −½·Ξ·dt (4×3)
    _F[IDX_Q0*EKF_N + IDX_BGX] = 0.5f * q1 * dt;
    _F[IDX_Q0*EKF_N + IDX_BGY] = 0.5f * q2 * dt;
    _F[IDX_Q0*EKF_N + IDX_BGZ] = 0.5f * q3 * dt;

    _F[IDX_Q1*EKF_N + IDX_BGX] = -0.5f * q0 * dt;
    _F[IDX_Q1*EKF_N + IDX_BGY] = 0.5f * q3 * dt;
    _F[IDX_Q1*EKF_N + IDX_BGZ] = -0.5f * q2 * dt;

    _F[IDX_Q2*EKF_N + IDX_BGX] = -0.5f * q3 * dt;
    _F[IDX_Q2*EKF_N + IDX_BGY] = -0.5f * q0 * dt;
    _F[IDX_Q2*EKF_N + IDX_BGZ] = 0.5f * q1 * dt;

    _F[IDX_Q3*EKF_N + IDX_BGX] = 0.5f * q2 * dt;
    _F[IDX_Q3*EKF_N + IDX_BGY] = -0.5f * q1 * dt;
    _F[IDX_Q3*EKF_N + IDX_BGZ] = -0.5f * q0 * dt;

    // Bias blocks: identity (diagonal = 1 from mat_identity, off-diagonal = 0)
}

// =============================================================================
//  PRIVATE: build_Q
// =============================================================================
void EKF_INS::build_Q(float dt)
{
    mat_zero(_Q, EKF_N, EKF_N);
    _Q[IDX_PX*EKF_N + IDX_PX] = Q_POS * dt;
    _Q[IDX_PY*EKF_N + IDX_PY] = Q_POS * dt;
    _Q[IDX_PZ*EKF_N + IDX_PZ] = Q_POS * dt;
    _Q[IDX_VX*EKF_N + IDX_VX] = Q_VEL * dt;
    _Q[IDX_VY*EKF_N + IDX_VY] = Q_VEL * dt;
    _Q[IDX_VZ*EKF_N + IDX_VZ] = Q_VEL * dt;
    // Quaternion: 4 components share the attitude PSD
    // Q_QUAT is per-component; represents angular random walk through kinematic Jacobian
    for (int i = IDX_Q0; i <= IDX_Q3; i++) _Q[i*EKF_N + i] = Q_QUAT * dt;
    _Q[IDX_BAX*EKF_N + IDX_BAX] = Q_BIAS_A * dt;
    _Q[IDX_BAY*EKF_N + IDX_BAY] = Q_BIAS_A * dt;
    _Q[IDX_BAZ*EKF_N + IDX_BAZ] = Q_BIAS_A * dt;
    _Q[IDX_BGX*EKF_N + IDX_BGX] = Q_BIAS_G * dt;
    _Q[IDX_BGY*EKF_N + IDX_BGY] = Q_BIAS_G * dt;
    _Q[IDX_BGZ*EKF_N + IDX_BGZ] = Q_BIAS_G * dt;
}

// =============================================================================
//  UPDATE: GPS  (p = 6)
// =============================================================================
void EKF_INS::update_gps(const GpsData &fix)
{
    if (!_initialised || !fix.valid) return;

    // Convert LLA to NED
    float pN, pE, pD;
    lla_to_ned(fix.lat_deg, fix.lon_deg, fix.alt_msl_m, pN, pE, pD);

    // Innovation z − H·x
    float y[EKF_P_GPS];
    y[0] = pN            - _x[IDX_PX];
    y[1] = pE            - _x[IDX_PY];
    y[2] = pD            - _x[IDX_PZ];
    y[3] = fix.vel_n_ms  - _x[IDX_VX];
    y[4] = fix.vel_e_ms  - _x[IDX_VY];
    y[5] = fix.vel_d_ms  - _x[IDX_VZ];

    // Diagonal of R_gps
    float R_diag[EKF_P_GPS];
    for (int i = 0; i < EKF_P_GPS; i++) R_diag[i] = _R_gps[i*EKF_P_GPS + i];

    ekf_update_joseph(_H_gps, EKF_P_GPS, y, R_diag);
}

// =============================================================================
//  UPDATE: Barometer  (p = 1)
//
//  State pD (IDX_PZ) represents position Down in NED.
//  Altitude above MSL = origin_alt − pD
//  So pD = origin_alt − baro_alt_m
//  Innovation = (origin_alt − baro_alt) − x[IDX_PZ]
// =============================================================================
void EKF_INS::update_baro(float baro_alt_m)
{
    if (!_initialised) return;

    float pD_from_baro = _origin_alt - baro_alt_m;
    float y[1] = { pD_from_baro - _x[IDX_PZ] };
    float R_diag[1] = { _R_baro[0] };

    ekf_update_joseph(_H_baro, EKF_P_BARO, y, R_diag);
}

// =============================================================================
//  UPDATE: Magnetometer  (p = 3, NONLINEAR)
//
//  Measurement model:   h(q) = C_bn^T(q) · m̂_ned
//
//  The predicted body-frame field is the NED reference field rotated to body.
//  Innovation y = z_meas − h(q)
//
//  Jacobian H_mag (3×16):
//   • Columns 0..5  (p, v):  zero
//   • Columns 6..9  (q):     ∂h/∂q  (3×4, analytical)
//   • Columns 10..15 (ba,bg): zero
//
//  ∂h/∂qᵢ = (∂C_bn/∂qᵢ)^T · m̂_ned = Dᵢ^T · m̂
//
//  where Dᵢ = ∂C_bn/∂qᵢ (3×3, factor of 2 included):
//   D0 = 2·[ q0  −q3   q2 ;  q3   q0  −q1 ; −q2   q1   q0]
//   D1 = 2·[ q1   q2   q3 ;  q2  −q1  −q0 ;  q3   q0  −q1]
//   D2 = 2·[−q2   q1   q0 ;  q1   q2   q3 ; −q0   q3  −q2]
//   D3 = 2·[−q3  −q0   q1 ;  q0  −q3   q2 ;  q1   q2   q3]
// =============================================================================
void EKF_INS::update_mag(float mx, float my, float mz, const float m_ned[3])
{
    if (!_initialised) return;

    const float q0=_x[IDX_Q0], q1=_x[IDX_Q1];
    const float q2=_x[IDX_Q2], q3=_x[IDX_Q3];
    const float mN=m_ned[0],   mE=m_ned[1],   mD=m_ned[2];

    // ── Predicted body-frame field: h = C_bn^T · m_ned ───────────────────────
    // C_bn^T = C_nb; row i of C_nb = column i of C_bn
    float Cbn[9];
    build_Cbn(_x + IDX_Q0, Cbn);

    // h = C_bn^T · m_ned  (multiply C_bn^T by m, i.e., m dotted with each column of Cbn)
    float hx = Cbn[0]*mN + Cbn[3]*mE + Cbn[6]*mD;  // col 0 of Cbn
    float hy = Cbn[1]*mN + Cbn[4]*mE + Cbn[7]*mD;  // col 1
    float hz = Cbn[2]*mN + Cbn[5]*mE + Cbn[8]*mD;  // col 2

    // ── Innovation ────────────────────────────────────────────────────────────
    float y[3] = { mx - hx,  my - hy,  mz - hz };

    // ── Build H_mag (3×16) ────────────────────────────────────────────────────
    float H_mag[EKF_P_MAG * EKF_N];
    mat_zero(H_mag, EKF_P_MAG, EKF_N);

    // ∂h/∂q0 = D0^T · m̂   D0 = 2·[ q0 -q3 q2; q3 q0 -q1; -q2 q1 q0]
    // D0^T = 2·[ q0  q3 -q2; -q3  q0  q1;  q2 -q1  q0]
    H_mag[0*EKF_N + IDX_Q0] = 2.0f*( q0*mN + q3*mE - q2*mD);
    H_mag[1*EKF_N + IDX_Q0] = 2.0f*(-q3*mN + q0*mE + q1*mD);
    H_mag[2*EKF_N + IDX_Q0] = 2.0f*( q2*mN - q1*mE + q0*mD);

    // ∂h/∂q1 = D1^T · m̂   D1^T = 2·[ q1  q2  q3;  q2 -q1  q0;  q3 -q0 -q1]
    H_mag[0*EKF_N + IDX_Q1] = 2.0f*( q1*mN + q2*mE + q3*mD);
    H_mag[1*EKF_N + IDX_Q1] = 2.0f*( q2*mN - q1*mE + q0*mD);
    H_mag[2*EKF_N + IDX_Q1] = 2.0f*( q3*mN - q0*mE - q1*mD);

    // ∂h/∂q2 = D2^T · m̂   D2^T = 2·[-q2  q1 -q0;  q1  q2  q3;  q0  q3 -q2]
    H_mag[0*EKF_N + IDX_Q2] = 2.0f*(-q2*mN + q1*mE - q0*mD);
    H_mag[1*EKF_N + IDX_Q2] = 2.0f*( q1*mN + q2*mE + q3*mD);
    H_mag[2*EKF_N + IDX_Q2] = 2.0f*( q0*mN + q3*mE - q2*mD);

    // ∂h/∂q3 = D3^T · m̂   D3^T = 2·[-q3  q0  q1; -q0 -q3  q2;  q1  q2  q3]
    H_mag[0*EKF_N + IDX_Q3] = 2.0f*(-q3*mN + q0*mE + q1*mD);
    H_mag[1*EKF_N + IDX_Q3] = 2.0f*(-q0*mN - q3*mE + q2*mD);
    H_mag[2*EKF_N + IDX_Q3] = 2.0f*( q1*mN + q2*mE + q3*mD);

    // Diagonal of R_mag
    float R_diag[3] = { _R_mag[0], _R_mag[4], _R_mag[8] };

    ekf_update_joseph(H_mag, EKF_P_MAG, y, R_diag);

    // Re-normalise quaternion after magnetometer correction
    normalise_quaternion_state();
}

// =============================================================================
//  CORE: ekf_update_joseph — Joseph-form update for any measurement size p
//
//  Inputs:
//    H[p×n]      — measurement Jacobian (or exact linear H)
//    p           — measurement dimension (1, 3, or 6)
//    z_innov[p]  — innovation (z − h(x))
//    R_diag[p]   — diagonal of measurement noise covariance R
//
//  Steps:
//    1. Build full diagonal R matrix from R_diag
//    2. S  = H·P·Hᵀ + R                     (p×p)
//    3. K  = P·Hᵀ·S⁻¹                       (n×p)
//    4. x  = x + K·y                         (state update)
//    5. L  = I − K·H                         (n×n)
//    6. P⁺ = L·P·Lᵀ + K·R·Kᵀ               (Joseph form)
//
//  Joseph form guarantees:
//    • P⁺ is symmetric by construction (L·P·Lᵀ + K·R·Kᵀ are both symmetric)
//    • P⁺ is PSD by construction (both terms are PSD for any K)
//    • Works correctly even when K is not the optimal Kalman gain
//      (robustness for outlier-contaminated measurements)
// =============================================================================
void EKF_INS::ekf_update_joseph(const float *H, int p,
                                 const float *z_innov, const float *R_diag)
{
    if (p > EKF_P_MAX) return;   // safety guard

    // ── Build diagonal R (p×p) ────────────────────────────────────────────────
    float R[EKF_P_MAX * EKF_P_MAX];
    mat_zero(R, p, p);
    for (int i = 0; i < p; i++) R[i*p + i] = R_diag[i];

    // ── S = H·P·Hᵀ + R  (p×p) ────────────────────────────────────────────────
    // tmp_pn = H·P       (p×n)
    mat_mul(H, _P, _tmp_pn, p, EKF_N, EKF_N);
    // S = tmp_pn · Hᵀ    (p×p)
    mat_mul_trans_B(_tmp_pn, H, _S, p, EKF_N, p);
    // S += R
    mat_add(_S, R, _S, p, p);

    // ── S⁻¹  (p×p) ───────────────────────────────────────────────────────────
    if (!mat_inv(_S, _Sinv, p)) {
        DEBUG_SERIAL.println("[EKF] WARNING: S singular — update skipped");
        return;
    }

    // ── K = P·Hᵀ·S⁻¹  (n×p) ─────────────────────────────────────────────────
    // tmp_np = P·Hᵀ      (n×p)
    mat_mul_trans_B(_P, H, _tmp_np, EKF_N, EKF_N, p);
    // K = tmp_np · S⁻¹   (n×p)
    mat_mul(_tmp_np, _Sinv, _K, EKF_N, p, p);

    // ── State update: x = x + K·y ─────────────────────────────────────────────
    for (int i = 0; i < EKF_N; i++) {
        float corr = 0.0f;
        for (int j = 0; j < p; j++) corr += _K[i*p + j] * z_innov[j];
        _x[i] += corr;
    }

    // ── Joseph form: P⁺ = L·P·Lᵀ + K·R·Kᵀ ──────────────────────────────────
    // L = I − K·H  (n×n)
    mat_identity(_IKH, EKF_N);
    float KH[EKF_N * EKF_N];
    mat_mul(_K, H, KH, EKF_N, p, EKF_N);      // K·H
    mat_sub(_IKH, KH, _IKH, EKF_N, EKF_N);   // I − K·H

    //  Term 1: L·P·Lᵀ  (n×n)
    //   tmp_nn  = L·P   (n×n)
    mat_mul(_IKH, _P, _tmp_nn, EKF_N, EKF_N, EKF_N);
    //   P_new   = tmp_nn · Lᵀ  = L·P·Lᵀ
    float P_new[EKF_N * EKF_N];
    mat_mul_trans_B(_tmp_nn, _IKH, P_new, EKF_N, EKF_N, EKF_N);

    //  Term 2: K·R·Kᵀ  (n×n)
    //   tmp_np2 = K·R   (n×p)
    float tmp_np2[EKF_N * EKF_P_MAX];
    mat_mul(_K, R, tmp_np2, EKF_N, p, p);
    //   KRKt    = tmp_np2·Kᵀ  (n×n)
    float KRKt[EKF_N * EKF_N];
    mat_mul_trans_B(tmp_np2, _K, KRKt, EKF_N, p, EKF_N);

    //  P⁺ = L·P·Lᵀ + K·R·Kᵀ
    mat_add(P_new, KRKt, _P, EKF_N, EKF_N);

    symmetrise_P();
}

// =============================================================================
//  LLA → NED  (flat-earth, valid ≤ ~10 km radius)
// =============================================================================
void EKF_INS::lla_to_ned(double lat, double lon, float alt,
                          float &n, float &e, float &d) const
{
    double dlat = (lat - _origin_lat) * DEG2RAD;
    double dlon = (lon - _origin_lon) * DEG2RAD;
    double cos_lat0 = cos(_origin_lat * DEG2RAD);

    n = (float)(dlat * EARTH_RADIUS_M);
    e = (float)(dlon * EARTH_RADIUS_M * cos_lat0);
    d = -( alt - _origin_alt );
}

// =============================================================================
//  Accessors
// =============================================================================
void EKF_INS::get_euler(float &roll, float &pitch, float &yaw) const
{
    const float q[4] = { _x[IDX_Q0], _x[IDX_Q1], _x[IDX_Q2], _x[IDX_Q3] };
    quat_to_euler(q, roll, pitch, yaw);
}

float EKF_INS::sigma_yaw() const
{
    // Propagate yaw uncertainty from quaternion covariance.
    // ∂yaw/∂q from quat_to_euler (small-angle approximation around current q):
    // yaw = atan2(2(q0q3+q1q2), 1-2(q2²+q3²))
    // For propagation: σ²_yaw ≈ (∂yaw/∂q)·P_qq·(∂yaw/∂q)^T
    // Simplified: just return sqrt(P[IDX_Q3,IDX_Q3]) * 2 as rough estimate
    return 2.0f * sqrtf(fabsf(_P[IDX_Q3*EKF_N + IDX_Q3]));
}

// =============================================================================
//  PRIVATE: symmetrise_P
// =============================================================================
void EKF_INS::symmetrise_P()
{
    for (int i = 0; i < EKF_N; i++) {
        for (int j = i+1; j < EKF_N; j++) {
            float avg = 0.5f * (_P[i*EKF_N+j] + _P[j*EKF_N+i]);
            _P[i*EKF_N+j] = avg;
            _P[j*EKF_N+i] = avg;
        }
        if (_P[i*EKF_N+i] < 0.0f) _P[i*EKF_N+i] = 1e-9f;  // clip negative diagonal
    }
}

// =============================================================================
//  PRIVATE: normalise_quaternion_state
//  Re-normalises the quaternion sub-vector in _x.
//  Also corrects the 4×4 quaternion block of P via a projection:
//    P_qq⁺ = P_qq − q·qᵀ·P_qq − P_qq·q·qᵀ   (to lowest order)
//  This keeps P consistent with the unit constraint.
// =============================================================================
void EKF_INS::normalise_quaternion_state()
{
    float *q = _x + IDX_Q0;
    float n = quat_normalise(q);

    // If norm deviated significantly, project P to enforce unit constraint
    if (fabsf(n - 1.0f) > 1e-5f) {
        // Projection: P_new = (I - q*qT) * P * (I - q*qT)
        // This is an approximation; for tutorial simplicity, just symmetrise
        symmetrise_P();
    }
}
