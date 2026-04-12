// =============================================================================
//  ekf_ins.cpp  —  15-state strapdown INS / GNSS EKF
// =============================================================================
#include "ekf_ins.h"
#include <math.h>
#include <string.h>

// ---------------------------------------------------------------------------
//  Constructor
// ---------------------------------------------------------------------------
EKF_INS::EKF_INS() : _initialised(false), _origin_lat(0), _origin_lon(0),
                     _origin_alt(0)
{
    mat_zero(_x, EKF_N, 1);
    mat_zero(_P, EKF_N, EKF_N);
    mat_zero(_Q, EKF_N, EKF_N);
    mat_zero(_R, EKF_P, EKF_P);
    mat_zero(_H, EKF_P, EKF_N);
}

// ---------------------------------------------------------------------------
//  init()  — set origin, state, covariance, noise matrices
// ---------------------------------------------------------------------------
void EKF_INS::init(double origin_lat, double origin_lon, float origin_alt,
                   float init_yaw)
{
    _origin_lat = origin_lat;
    _origin_lon = origin_lon;
    _origin_alt = origin_alt;

    // ── State vector ─────────────────────────────────────────────────────────
    mat_zero(_x, EKF_N, 1);
    _x[IDX_YAW] = init_yaw;
    // Position starts at NED origin (0,0,0)
    // Velocity starts at zero
    // Biases start at zero

    // ── Initial covariance P₀ ────────────────────────────────────────────────
    mat_zero(_P, EKF_N, EKF_N);
    _P[IDX_PX*EKF_N + IDX_PX] = P0_POS;
    _P[IDX_PY*EKF_N + IDX_PY] = P0_POS;
    _P[IDX_PZ*EKF_N + IDX_PZ] = P0_POS;
    _P[IDX_VX*EKF_N + IDX_VX] = P0_VEL;
    _P[IDX_VY*EKF_N + IDX_VY] = P0_VEL;
    _P[IDX_VZ*EKF_N + IDX_VZ] = P0_VEL;
    _P[IDX_ROLL *EKF_N + IDX_ROLL]  = P0_ATT;
    _P[IDX_PITCH*EKF_N + IDX_PITCH] = P0_ATT;
    _P[IDX_YAW  *EKF_N + IDX_YAW]   = P0_ATT;
    for (int i = IDX_BAX; i <= IDX_BAZ; i++) _P[i*EKF_N + i] = P0_BIAS_A;
    for (int i = IDX_BGX; i <= IDX_BGZ; i++) _P[i*EKF_N + i] = P0_BIAS_G;

    // ── Measurement noise R  (6×6 diagonal, GPS) ─────────────────────────────
    mat_zero(_R, EKF_P, EKF_P);
    _R[0*6+0] = R_GPS_POS;    // N position
    _R[1*6+1] = R_GPS_POS;    // E position
    _R[2*6+2] = R_GPS_POS_V;  // D position (altitude)
    _R[3*6+3] = R_GPS_VEL_H;  // N velocity
    _R[4*6+4] = R_GPS_VEL_H;  // E velocity
    _R[5*6+5] = R_GPS_VEL_V;  // D velocity

    // ── Measurement matrix H  (6×15) ─────────────────────────────────────────
    // H maps state → GPS measurement: z = [pN,pE,pD,vN,vE,vD]
    // Row 0: pN → _x[IDX_PX]  → H[0, IDX_PX] = 1
    mat_zero(_H, EKF_P, EKF_N);
    _H[0*EKF_N + IDX_PX] = 1.0f;
    _H[1*EKF_N + IDX_PY] = 1.0f;
    _H[2*EKF_N + IDX_PZ] = 1.0f;
    _H[3*EKF_N + IDX_VX] = 1.0f;
    _H[4*EKF_N + IDX_VY] = 1.0f;
    _H[5*EKF_N + IDX_VZ] = 1.0f;

    _initialised = true;
    DEBUG_SERIAL.println("[EKF] Initialised.");
}

// =============================================================================
//  PREDICT  — called at IMU_SAMPLE_HZ
// =============================================================================
void EKF_INS::predict(float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float dt)
{
    if (!_initialised) return;

    float roll  = _x[IDX_ROLL];
    float pitch = _x[IDX_PITCH];
    float yaw   = _x[IDX_YAW];

    // Remove estimated biases from IMU measurements
    float f_x = ax - _x[IDX_BAX];
    float f_y = ay - _x[IDX_BAY];
    float f_z = az - _x[IDX_BAZ];
    float w_x = gx - _x[IDX_BGX];
    float w_y = gy - _x[IDX_BGY];
    float w_z = gz - _x[IDX_BGZ];

    // ── Body → NED rotation matrix C_bn ──────────────────────────────────────
    float Cbn[9];
    build_Cbn(roll, pitch, yaw, Cbn);

    // ── NED specific force: a_ned = C_bn * f_body ─────────────────────────────
    float a_ned_x = Cbn[0]*f_x + Cbn[1]*f_y + Cbn[2]*f_z;
    float a_ned_y = Cbn[3]*f_x + Cbn[4]*f_y + Cbn[5]*f_z;
    float a_ned_z = Cbn[6]*f_x + Cbn[7]*f_y + Cbn[8]*f_z;

    // Add gravity in NED frame (NED: Down is +z, gravity acts +Down)
    a_ned_z += GRAVITY_MS2;

    // ── Euler rate matrix T ───────────────────────────────────────────────────
    float T[9];
    build_T(roll, pitch, yaw, T);

    // ── Euler angle rates: η_dot = T * w_body ────────────────────────────────
    float roll_dot  = T[0]*w_x + T[1]*w_y + T[2]*w_z;
    float pitch_dot = T[3]*w_x + T[4]*w_y + T[5]*w_z;
    float yaw_dot   = T[6]*w_x + T[7]*w_y + T[8]*w_z;

    // ── Integrate state (Euler method) ────────────────────────────────────────
    // Position
    _x[IDX_PX] += _x[IDX_VX] * dt;
    _x[IDX_PY] += _x[IDX_VY] * dt;
    _x[IDX_PZ] += _x[IDX_VZ] * dt;

    // Velocity
    _x[IDX_VX] += a_ned_x * dt;
    _x[IDX_VY] += a_ned_y * dt;
    _x[IDX_VZ] += a_ned_z * dt;

    // Attitude
    _x[IDX_ROLL]  = wrap_pi(_x[IDX_ROLL]  + roll_dot  * dt);
    _x[IDX_PITCH] = wrap_pi(_x[IDX_PITCH] + pitch_dot * dt);
    _x[IDX_YAW]   = wrap_pi(_x[IDX_YAW]   + yaw_dot   * dt);

    // Biases: random walk — no change in mean

    // ── Build Jacobian F ──────────────────────────────────────────────────────
    build_Jacobian_F(ax, ay, az, gx, gy, gz, dt);

    // ── Build process noise Q(dt) ─────────────────────────────────────────────
    build_Q(dt);

    // ── Covariance propagation: P⁻ = F · P · Fᵀ + Q ─────────────────────────
    // Step 1: tmp1 = F · P   (15×15)
    mat_mul(_F, _P, _tmp1, EKF_N, EKF_N, EKF_N);
    // Step 2: P⁻ = tmp1 · Fᵀ = F · P · Fᵀ
    mat_mul_trans_B(_tmp1, _F, _P, EKF_N, EKF_N, EKF_N);
    // Step 3: P⁻ += Q
    mat_add(_P, _Q, _P, EKF_N, EKF_N);

    symmetrise_P();
}

// =============================================================================
//  UPDATE — called when a fresh GPS fix arrives
// =============================================================================
void EKF_INS::update_gps(const GpsData &fix)
{
    if (!_initialised || !fix.valid) return;

    // ── Convert LLA to NED relative to origin ────────────────────────────────
    float pos_n, pos_e, pos_d;
    lla_to_ned(fix.lat_deg, fix.lon_deg, fix.alt_msl_m,
               pos_n, pos_e, pos_d);

    // ── Build measurement vector z[6] ────────────────────────────────────────
    float z[EKF_P];
    z[0] = pos_n;
    z[1] = pos_e;
    z[2] = pos_d;
    z[3] = fix.vel_n_ms;
    z[4] = fix.vel_e_ms;
    z[5] = fix.vel_d_ms;

    // ── Innovation  y = z − H·x  ─────────────────────────────────────────────
    // H selects first 6 elements of x directly (linear model)
    float y[EKF_P];
    for (int i = 0; i < EKF_P; i++) {
        float hx = 0.0f;
        for (int j = 0; j < EKF_N; j++) hx += _H[i*EKF_N + j] * _x[j];
        y[i] = z[i] - hx;
    }

    // ── Innovation covariance  S = H · P · Hᵀ + R  ──────────────────────────
    // Step 1: tmp3 = H · P       (6×15)
    mat_mul(_H, _P, _tmp3, EKF_P, EKF_N, EKF_N);
    // Step 2: S = tmp3 · Hᵀ + R  (6×6)
    mat_mul_trans_B(_tmp3, _H, _S, EKF_P, EKF_N, EKF_P);
    mat_add(_S, _R, _S, EKF_P, EKF_P);

    // ── Invert S ─────────────────────────────────────────────────────────────
    if (!mat_inv_6x6(_S, _Sinv)) {
        DEBUG_SERIAL.println("[EKF] WARNING: S matrix singular — skipping update");
        return;
    }

    // ── Kalman Gain  K = P · Hᵀ · S⁻¹  ──────────────────────────────────────
    // Step 1: tmp2 = P · Hᵀ     (15×6)
    mat_mul_trans_B(_P, _H, _tmp2, EKF_N, EKF_N, EKF_P);
    // Step 2: K = tmp2 · S⁻¹    (15×6)
    mat_mul(_tmp2, _Sinv, _K, EKF_N, EKF_P, EKF_P);

    // ── State update  x = x + K · y  ─────────────────────────────────────────
    for (int i = 0; i < EKF_N; i++) {
        float corr = 0.0f;
        for (int j = 0; j < EKF_P; j++) corr += _K[i*EKF_P + j] * y[j];
        _x[i] += corr;
    }
    // Wrap angles after state update
    _x[IDX_ROLL]  = wrap_pi(_x[IDX_ROLL]);
    _x[IDX_PITCH] = wrap_pi(_x[IDX_PITCH]);
    _x[IDX_YAW]   = wrap_pi(_x[IDX_YAW]);

    // ── Covariance update  P = (I − K·H) · P  ────────────────────────────────
    // Joseph form for numerical stability:
    //   P = (I−KH)·P⁻·(I−KH)ᵀ + K·R·Kᵀ
    //
    // Simple form used here (sufficient for tutorial and well-conditioned R):
    //   P = (I − K·H) · P
    float IKH[EKF_N * EKF_N];
    mat_identity(IKH, EKF_N);
    // tmp1 = K·H  (15×15)
    mat_mul(_K, _H, _tmp1, EKF_N, EKF_P, EKF_N);
    // IKH = I − K·H
    mat_sub(IKH, _tmp1, IKH, EKF_N, EKF_N);
    // P = IKH · P
    mat_mul(IKH, _P, _tmp1, EKF_N, EKF_N, EKF_N);
    mat_copy(_tmp1, _P, EKF_N, EKF_N);

    symmetrise_P();
}

// =============================================================================
//  PRIVATE: build_Cbn — Body → NED rotation matrix (ZYX Euler)
//
//  Convention: ZYX intrinsic rotations (yaw→pitch→roll)
//  Applied as:  v_ned = C_bn · v_body
//
//  C_bn =
//  [ cψcθ,   cψsθsφ−sψcφ,   cψsθcφ+sψsφ ]
//  [ sψcθ,   sψsθsφ+cψcφ,   sψsθcφ−cψsφ ]
//  [ −sθ,    cθsφ,           cθcφ         ]
//
//  where cφ=cos(roll), sφ=sin(roll), etc.
// =============================================================================
void EKF_INS::build_Cbn(float roll, float pitch, float yaw, float Cbn[9]) const
{
    float cr = cosf(roll),  sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw),   sy = sinf(yaw);

    Cbn[0] = cy*cp;              Cbn[1] = cy*sp*sr - sy*cr;  Cbn[2] = cy*sp*cr + sy*sr;
    Cbn[3] = sy*cp;              Cbn[4] = sy*sp*sr + cy*cr;  Cbn[5] = sy*sp*cr - cy*sr;
    Cbn[6] = -sp;                Cbn[7] = cp*sr;             Cbn[8] = cp*cr;
}

// =============================================================================
//  PRIVATE: build_T — Euler rate kinematics matrix
//
//  η̇ = T(η) · ω_body
//
//  T =
//  [ 1,   sin(φ)·tan(θ),   cos(φ)·tan(θ) ]
//  [ 0,   cos(φ),          −sin(φ)        ]
//  [ 0,   sin(φ)/cos(θ),   cos(φ)/cos(θ) ]
//
//  Singularity at θ = ±90° (pitch gimbal lock).
//  For a drone that stays within ±85° pitch, this is acceptable.
//  Use quaternions to remove singularity in production systems.
// =============================================================================
void EKF_INS::build_T(float roll, float pitch, float /*yaw*/, float T[9]) const
{
    float sr = sinf(roll), cr = cosf(roll);
    float sp = sinf(pitch), cp = cosf(pitch);

    // Guard against gimbal lock (|pitch| → 90°)
    if (fabsf(cp) < 1e-6f) cp = 1e-6f;
    float tp = sp / cp;

    T[0] = 1.0f;  T[1] = sr*tp;       T[2] = cr*tp;
    T[3] = 0.0f;  T[4] = cr;          T[5] = -sr;
    T[6] = 0.0f;  T[7] = sr/cp;       T[8] = cr/cp;
}

// =============================================================================
//  PRIVATE: build_Jacobian_F
//
//  Computes the discrete state transition Jacobian F = I + Fc·dt
//
//  Fc is the continuous-time Jacobian.  Non-zero blocks:
//
//  Block (3:6, 6:9) — ∂(C_bn·f̃)/∂η  (velocity w.r.t. attitude)
//    Analytical partial derivatives of C_bn·f̃ w.r.t. φ, θ, ψ
//    where f̃ = [f_x, f_y, f_z] (bias-corrected specific force)
//
//  Block (3:6, 9:12) — ∂(C_bn·f̃)/∂bₐ = −C_bn
//
//  Block (6:9, 6:9) — ∂(T·ω̃)/∂η  (attitude rate w.r.t. attitude)
//    Analytical partial derivatives of T·ω̃ w.r.t. φ, θ
//    (yaw does not appear in T)
//
//  Block (6:9, 12:15) — ∂(T·ω̃)/∂bᵍ = −T
// =============================================================================
void EKF_INS::build_Jacobian_F(float ax, float ay, float az,
                                float gx, float gy, float gz,
                                float dt)
{
    float roll  = _x[IDX_ROLL];
    float pitch = _x[IDX_PITCH];
    float yaw   = _x[IDX_YAW];

    // Bias-corrected measurements
    float fx = ax - _x[IDX_BAX];
    float fy = ay - _x[IDX_BAY];
    float fz = az - _x[IDX_BAZ];
    float wx = gx - _x[IDX_BGX];
    float wy = gy - _x[IDX_BGY];
    float wz = gz - _x[IDX_BGZ];

    float cr = cosf(roll),  sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw),   sy = sinf(yaw);
    if (fabsf(cp) < 1e-6f) cp = 1e-6f;
    float tp = sp / cp;

    // Build Cbn for use in ∂v/∂bₐ = −Cbn block
    float Cbn[9];
    build_Cbn(roll, pitch, yaw, Cbn);

    // Build T for ∂η/∂bᵍ = −T block
    float T[9];
    build_T(roll, pitch, yaw, T);

    // ── Start with identity ───────────────────────────────────────────────────
    mat_identity(_F, EKF_N);

    // ── Block (0:3, 3:6) = I·dt   (dp/dv) ────────────────────────────────────
    _F[IDX_PX*EKF_N + IDX_VX] = dt;
    _F[IDX_PY*EKF_N + IDX_VY] = dt;
    _F[IDX_PZ*EKF_N + IDX_VZ] = dt;

    // ── Block (3:6, 6:9) = ∂(Cbn·f̃)/∂η · dt  ───────────────────────────────
    // Analytical: d(Cbn·f)/dφ  (partial of column 3:6 of F w.r.t. roll)
    // Using product rule: d(Cbn)/dφ · f̃
    //
    // dCbn/dφ:
    //  [ 0,   cy*sp*cr+sy*sr,   −cy*sp*sr+sy*cr ]
    //  [ 0,   sy*sp*cr−cy*sr,   −sy*sp*sr−cy*cr ]
    //  [ 0,   cp*cr,            −cp*sr           ]
    float dCbn_droll_col1_x = cy*sp*cr + sy*sr;
    float dCbn_droll_col1_y = sy*sp*cr - cy*sr;
    float dCbn_droll_col1_z = cp*cr;
    float dCbn_droll_col2_x = -cy*sp*sr + sy*cr;
    float dCbn_droll_col2_y = -sy*sp*sr - cy*cr;
    float dCbn_droll_col2_z = -cp*sr;

    float dv_droll_x = dCbn_droll_col1_x*fy + dCbn_droll_col2_x*fz;
    float dv_droll_y = dCbn_droll_col1_y*fy + dCbn_droll_col2_y*fz;
    float dv_droll_z = dCbn_droll_col1_z*fy + dCbn_droll_col2_z*fz;

    // dCbn/dθ:
    //  [ −cy*sp,   cy*cp*sr,   cy*cp*cr ]
    //  [ −sy*sp,   sy*cp*sr,   sy*cp*cr ]
    //  [ −cp,      −sp*sr,     −sp*cr   ]
    float dv_dpitch_x = (-cy*sp)*fx + (cy*cp*sr)*fy + (cy*cp*cr)*fz;
    float dv_dpitch_y = (-sy*sp)*fx + (sy*cp*sr)*fy + (sy*cp*cr)*fz;
    float dv_dpitch_z = (-cp)*fx    + (-sp*sr)*fy   + (-sp*cr)*fz;

    // dCbn/dψ:
    //  [ −sy*cp,   −sy*sp*sr−cy*cr,   −sy*sp*cr+cy*sr ]
    //  [  cy*cp,    cy*sp*sr−sy*cr,    cy*sp*cr+sy*sr  ]
    //  [ 0,         0,                 0               ]
    float dv_dyaw_x = (-sy*cp)*fx + (-sy*sp*sr - cy*cr)*fy + (-sy*sp*cr + cy*sr)*fz;
    float dv_dyaw_y = ( cy*cp)*fx + ( cy*sp*sr - sy*cr)*fy + ( cy*sp*cr + sy*sr)*fz;
    float dv_dyaw_z = 0.0f;

    // Store block (3:6, 6:9) = [dv/droll | dv/dpitch | dv/dyaw] · dt
    _F[IDX_VX*EKF_N + IDX_ROLL]  = dv_droll_x  * dt;
    _F[IDX_VY*EKF_N + IDX_ROLL]  = dv_droll_y  * dt;
    _F[IDX_VZ*EKF_N + IDX_ROLL]  = dv_droll_z  * dt;
    _F[IDX_VX*EKF_N + IDX_PITCH] = dv_dpitch_x * dt;
    _F[IDX_VY*EKF_N + IDX_PITCH] = dv_dpitch_y * dt;
    _F[IDX_VZ*EKF_N + IDX_PITCH] = dv_dpitch_z * dt;
    _F[IDX_VX*EKF_N + IDX_YAW]   = dv_dyaw_x   * dt;
    _F[IDX_VY*EKF_N + IDX_YAW]   = dv_dyaw_y   * dt;
    _F[IDX_VZ*EKF_N + IDX_YAW]   = dv_dyaw_z   * dt;

    // ── Block (3:6, 9:12) = −Cbn · dt  (dv/dbₐ) ─────────────────────────────
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            _F[(IDX_VX+r)*EKF_N + (IDX_BAX+c)] = -Cbn[r*3+c] * dt;

    // ── Block (6:9, 6:9) = ∂(T·ω̃)/∂η · dt  ──────────────────────────────────
    // dT/dφ:
    //  [ 0,  cr·tp,  −sr·tp ]
    //  [ 0,  −sr,    −cr    ]
    //  [ 0,  cr/cp,  −sr/cp ]
    float deta_droll_0 = (cr*tp)*wy  + (-sr*tp)*wz;
    float deta_droll_1 = (-sr)*wy    + (-cr)*wz;
    float deta_droll_2 = (cr/cp)*wy  + (-sr/cp)*wz;

    // dT/dθ  (tp = sp/cp,  d(tp)/dθ = 1/cp²):
    //  [ 0,  sr/cp²,   cr/cp² ]
    //  [ 0,  0,        0      ]
    //  [ 0,  sr·sp/cp², cr·sp/cp² ]
    float inv_cp2 = 1.0f / (cp * cp);
    float deta_dpitch_0 = (sr*inv_cp2)*wy   + (cr*inv_cp2)*wz;
    float deta_dpitch_1 = 0.0f;
    float deta_dpitch_2 = (sr*sp*inv_cp2)*wy + (cr*sp*inv_cp2)*wz;

    _F[IDX_ROLL *EKF_N + IDX_ROLL]  += deta_droll_0  * dt;
    _F[IDX_PITCH*EKF_N + IDX_ROLL]  += deta_droll_1  * dt;
    _F[IDX_YAW  *EKF_N + IDX_ROLL]  += deta_droll_2  * dt;
    _F[IDX_ROLL *EKF_N + IDX_PITCH] += deta_dpitch_0 * dt;
    _F[IDX_PITCH*EKF_N + IDX_PITCH] += deta_dpitch_1 * dt;
    _F[IDX_YAW  *EKF_N + IDX_PITCH] += deta_dpitch_2 * dt;
    // yaw does not appear in T → no block (6:9, IDX_YAW)

    // ── Block (6:9, 12:15) = −T · dt  (dη/dbᵍ) ──────────────────────────────
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            _F[(IDX_ROLL+r)*EKF_N + (IDX_BGX+c)] = -T[r*3+c] * dt;

    // Bias blocks remain identity (diagonal = 1, set by mat_identity above)
}

// =============================================================================
//  PRIVATE: build_Q — discrete process noise Q = Qc · dt
//
//  Qc is a continuous-time PSD diagonal.  Discretisation via Q = Qc · Δt
//  (valid for Δt small relative to time constants — true at 500 Hz).
// =============================================================================
void EKF_INS::build_Q(float dt)
{
    mat_zero(_Q, EKF_N, EKF_N);
    // Position: driven by velocity noise through integration — set small
    _Q[IDX_PX*EKF_N + IDX_PX] = Q_POS * dt;
    _Q[IDX_PY*EKF_N + IDX_PY] = Q_POS * dt;
    _Q[IDX_PZ*EKF_N + IDX_PZ] = Q_POS * dt;
    // Velocity: unmodelled accelerations
    _Q[IDX_VX*EKF_N + IDX_VX] = Q_VEL * dt;
    _Q[IDX_VY*EKF_N + IDX_VY] = Q_VEL * dt;
    _Q[IDX_VZ*EKF_N + IDX_VZ] = Q_VEL * dt;
    // Attitude: model error in Euler rate kinematics
    _Q[IDX_ROLL *EKF_N + IDX_ROLL]  = Q_ATT * dt;
    _Q[IDX_PITCH*EKF_N + IDX_PITCH] = Q_ATT * dt;
    _Q[IDX_YAW  *EKF_N + IDX_YAW]   = Q_ATT * dt;
    // Biases: random walk PSD
    _Q[IDX_BAX*EKF_N + IDX_BAX] = Q_BIAS_A * dt;
    _Q[IDX_BAY*EKF_N + IDX_BAY] = Q_BIAS_A * dt;
    _Q[IDX_BAZ*EKF_N + IDX_BAZ] = Q_BIAS_A * dt;
    _Q[IDX_BGX*EKF_N + IDX_BGX] = Q_BIAS_G * dt;
    _Q[IDX_BGY*EKF_N + IDX_BGY] = Q_BIAS_G * dt;
    _Q[IDX_BGZ*EKF_N + IDX_BGZ] = Q_BIAS_G * dt;
}

// =============================================================================
//  LLA → NED  flat-earth approximation
//  Valid within ~10 km radius of origin.
//
//  North = (lat − lat0) × R_earth
//  East  = (lon − lon0) × R_earth × cos(lat0)
//  Down  = −(alt − alt0)   [NED: Down is positive downward]
// =============================================================================
void EKF_INS::lla_to_ned(double lat, double lon, float alt,
                          float &north, float &east, float &down) const
{
    double dlat = (lat - _origin_lat) * DEG2RAD;
    double dlon = (lon - _origin_lon) * DEG2RAD;
    double cos_lat0 = cos(_origin_lat * DEG2RAD);

    north = (float)(dlat * EARTH_RADIUS_M);
    east  = (float)(dlon * EARTH_RADIUS_M * cos_lat0);
    down  = -( alt - _origin_alt );   // NED Down = negative altitude
}

// =============================================================================
//  PRIVATE: symmetrise_P — force symmetric to counteract floating-point drift
// =============================================================================
void EKF_INS::symmetrise_P()
{
    for (int i = 0; i < EKF_N; i++) {
        for (int j = i+1; j < EKF_N; j++) {
            float avg = 0.5f * (_P[i*EKF_N+j] + _P[j*EKF_N+i]);
            _P[i*EKF_N+j] = avg;
            _P[j*EKF_N+i] = avg;
        }
        // Clamp diagonal to non-negative (numerical safety)
        if (_P[i*EKF_N+i] < 0.0f) _P[i*EKF_N+i] = 0.0f;
    }
}

// Wrap angle to [−π, +π]
float EKF_INS::wrap_pi(float a)
{
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}
