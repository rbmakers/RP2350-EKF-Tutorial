// =============================================================================
//  flight_control.cpp
// =============================================================================
#include "flight_control.h"
#include <string.h>

FlightController::FlightController()
    : _mode(FlightMode::DISARMED), _prev_mode(FlightMode::DISARMED),
      _vel_sp_n(0), _vel_sp_e(0), _vel_sp_d(0),
      _pos_sp_n(0), _pos_sp_e(0), _alt_sp_d(0), _yaw_sp(0),
      _throttle_norm(0),
      _arm_switch_prev(false), _arm_timer_ms(0), _disarm_timer_ms(0),
      _airmode_active(false)
{
    _rate_sp[0] = _rate_sp[1] = _rate_sp[2] = 0.0f;
    _angle_sp[0] = _angle_sp[1] = 0.0f;
}

// ---------------------------------------------------------------------------
//  begin()
// ---------------------------------------------------------------------------
void FlightController::begin()
{
    // Configure rate PIDs
    _rate_pids.configure_from_config((float)CTRL_LOOP_HZ);

    // Angle PIDs (P-only)
    _angle_roll.set_gains (PID_ANGLE_ROLL_P,  0, 0);
    _angle_pitch.set_gains(PID_ANGLE_PITCH_P, 0, 0);

    // Altitude PIDs
    _alt_pos_pid.set_gains(PID_ALT_P, PID_ALT_I, 0);
    _alt_pos_pid.set_imax(0.5f);
    _alt_vel_pid.set_gains(PID_ALT_D, 0, 0);  // velocity P term

    // Position/velocity PIDs
    _vel_n_pid.set_gains(PID_VEL_N_P, PID_VEL_N_I, 0);
    _vel_e_pid.set_gains(PID_VEL_E_P, PID_VEL_E_I, 0);
    _vel_n_pid.set_imax(0.4f);
    _vel_e_pid.set_imax(0.4f);

    // Gyro LPF
    for (int i = 0; i < 3; i++)
        _gyro_lpf[i].configure((float)CTRL_LOOP_HZ, GYRO_LPF_HZ);
}

// =============================================================================
//  update_rate_loop()  — 2 kHz, raw gyro
// =============================================================================
void FlightController::update_rate_loop(const float gyro_rads[3],
                                         float throttle_norm,
                                         float dt, MotorOutputs &out)
{
    if (!is_armed()) {
        out.disarm();
        _rate_pids.reset_all();
        return;
    }

    // Apply gyro low-pass filter
    float gx = _gyro_lpf[0].update(gyro_rads[0]);
    float gy = _gyro_lpf[1].update(gyro_rads[1]);
    float gz = _gyro_lpf[2].update(gyro_rads[2]);

    // Run rate PIDs
    float roll_out  = _rate_pids.roll.compute (_rate_sp[0], gx, dt, throttle_norm);
    float pitch_out = _rate_pids.pitch.compute(_rate_sp[1], gy, dt, throttle_norm);
    float yaw_out   = _rate_pids.yaw.compute  (_rate_sp[2], gz, dt, 0.0f);

    // Airmode: keep motors spinning at minimum when throttle is zero
    // prevents PID integrator from winding up and allows mid-air corrections
    _airmode_active = (_throttle_norm > 0.05f);

    float base_throttle = _throttle_norm;
    if (_airmode_active && base_throttle < 0.05f) {
        // Idle up: allow PID outputs to still function
        base_throttle = 0.05f;
    }

    mix_quad_x(base_throttle, roll_out, pitch_out, yaw_out, out);
}

// =============================================================================
//  update_angle_loop()  — 500 Hz, EKF attitude
// =============================================================================
void FlightController::update_angle_loop(float roll_rad, float pitch_rad,
                                          float yaw_rad)
{
    if (!is_armed()) return;

    if (_mode == FlightMode::MANUAL) {
        // Rate mode: angle setpoints are direct rate commands from RC
        // (already set in update_from_rc)
        return;
    }

    // Angle → rate setpoint (P-only cascade)
    float roll_rate_sp  = _angle_roll.compute_p_only(
        _angle_sp[0], roll_rad) * RAD2DEG;
    float pitch_rate_sp = _angle_pitch.compute_p_only(
        _angle_sp[1], pitch_rad) * RAD2DEG;

    // Clamp to max rates
    roll_rate_sp  = constrain(roll_rate_sp,
        -MAX_RATE_ROLL_DPS,  MAX_RATE_ROLL_DPS)  * DEG2RAD;
    pitch_rate_sp = constrain(pitch_rate_sp,
        -MAX_RATE_PITCH_DPS, MAX_RATE_PITCH_DPS) * DEG2RAD;

    // Yaw rate: always direct from RC (heading hold is managed by yaw PID)
    // _rate_sp[2] already set in update_from_rc

    _rate_sp[0] = roll_rate_sp;
    _rate_sp[1] = pitch_rate_sp;
}

// =============================================================================
//  update_altitude_loop()  — 50 Hz, EKF vertical
// =============================================================================
void FlightController::update_altitude_loop(float pos_d, float vel_d, float dt)
{
    if (!is_armed()) return;
    if (_mode != FlightMode::ALT_HOLD && _mode != FlightMode::POS_HOLD
        && _mode != FlightMode::FAILSAFE) return;

    // Position error → vertical velocity setpoint
    float pos_err = _alt_sp_d - pos_d;  // positive = need to go up (decrease D)
    float vel_sp  = _alt_pos_pid.compute(0.0f, -pos_err, dt);  // flip sign: NED Down

    // Clamp velocity setpoint
    vel_sp = constrain(vel_sp, MAX_VEL_DOWN_MS, MAX_VEL_UP_MS);

    // Velocity error → throttle delta (PD on velocity)
    float vel_err    = vel_sp - vel_d;  // NED: positive = going up too slow
    float thr_delta  = _alt_vel_pid.compute_p_only(0.0f, -vel_err);

    // Base throttle = hover estimate + correction
    _throttle_norm = HOVER_THROTTLE + thr_delta;
    _throttle_norm = constrain(_throttle_norm, 0.0f, 1.0f);
}

// =============================================================================
//  update_position_loop()  — 10 Hz, EKF NED
// =============================================================================
void FlightController::update_position_loop(float pos_n, float pos_e,
                                             float vel_n, float vel_e,
                                             float yaw_rad, float dt)
{
    if (!is_armed()) return;
    if (_mode != FlightMode::POS_HOLD) return;

    // Position error in NED
    float err_n = _pos_sp_n - pos_n;
    float err_e = _pos_sp_e - pos_e;

    // Limit position error (prevent large commanded lean angles after GPS gap)
    float dist = sqrtf(err_n*err_n + err_e*err_e);
    if (dist > 10.0f) {
        err_n = err_n / dist * 10.0f;
        err_e = err_e / dist * 10.0f;
    }

    // Position error → velocity setpoint (NED)
    float vel_sp_n = _vel_n_pid.compute(0.0f, -err_n, dt);
    float vel_sp_e = _vel_e_pid.compute(0.0f, -err_e, dt);

    // Clamp velocity
    float vspd = sqrtf(vel_sp_n*vel_sp_n + vel_sp_e*vel_sp_e);
    if (vspd > MAX_VEL_HORZ_MS) {
        vel_sp_n = vel_sp_n / vspd * MAX_VEL_HORZ_MS;
        vel_sp_e = vel_sp_e / vspd * MAX_VEL_HORZ_MS;
    }

    // Velocity error → angle setpoint
    float vel_err_n = vel_sp_n - vel_n;
    float vel_err_e = vel_sp_e - vel_e;

    // Rotate NED velocity error to body frame for roll/pitch angle setpoints
    float cy = cosf(yaw_rad), sy = sinf(yaw_rad);
    float angle_pitch_sp = -(cy * vel_err_n + sy * vel_err_e) * PID_VEL_N_P;
    float angle_roll_sp  =  (-sy * vel_err_n + cy * vel_err_e) * PID_VEL_E_P;

    // Clamp lean angles
    angle_pitch_sp = constrain(angle_pitch_sp,
        -MAX_POS_TILT_DEG * DEG2RAD, MAX_POS_TILT_DEG * DEG2RAD);
    angle_roll_sp  = constrain(angle_roll_sp,
        -MAX_POS_TILT_DEG * DEG2RAD, MAX_POS_TILT_DEG * DEG2RAD);

    _angle_sp[0] = angle_roll_sp;
    _angle_sp[1] = angle_pitch_sp;
}

// =============================================================================
//  update_from_rc()  — read RC, update mode/setpoints/arming
// =============================================================================
void FlightController::update_from_rc(const CrsfReceiver &rc)
{
    check_arm_disarm(rc);

    if (!is_armed()) return;

    // Failsafe
    if (rc.is_failsafe() && _mode != FlightMode::FAILSAFE) {
        on_mode_change(FlightMode::FAILSAFE);
    }

    // Mode selection (only if not in failsafe)
    if (_mode != FlightMode::FAILSAFE) {
        FlightMode req = requested_mode(rc);
        if (req != _mode) on_mode_change(req);
    }

    // Throttle
    _throttle_norm = rc.throttle_norm();

    // RC roll/pitch/yaw inputs — used differently by mode
    float rc_roll   = rc.channel_norm(RC_CH_ROLL);
    float rc_pitch  = rc.channel_norm(RC_CH_PITCH);
    float rc_yaw    = rc.channel_norm(RC_CH_YAW);

    switch (_mode) {
    case FlightMode::MANUAL:
        // Direct rate commands
        _rate_sp[0] = rc_roll  * MAX_RATE_ROLL_DPS  * DEG2RAD;
        _rate_sp[1] = rc_pitch * MAX_RATE_PITCH_DPS * DEG2RAD;
        _rate_sp[2] = rc_yaw   * MAX_RATE_YAW_DPS   * DEG2RAD;
        break;

    case FlightMode::STABILIZE:
    case FlightMode::ALT_HOLD:
        // RC → angle setpoints (angle loop converts these to rate setpoints)
        _angle_sp[0] = rc_roll  * MAX_ANGLE_ROLL_DEG  * DEG2RAD;
        _angle_sp[1] = rc_pitch * MAX_ANGLE_PITCH_DEG * DEG2RAD;
        _rate_sp[2]  = rc_yaw   * MAX_RATE_YAW_DPS    * DEG2RAD;
        if (_mode == FlightMode::STABILIZE) {
            // No altitude hold — throttle from RC directly
            _throttle_norm = rc.throttle_norm();
        }
        break;

    case FlightMode::POS_HOLD:
        // Small RC inputs nudge the position setpoint
        if (fabsf(rc_roll) > 0.1f || fabsf(rc_pitch) > 0.1f) {
            // Pilot override: update pos setpoint in direction of stick
            // (simplified: caller must provide pos_sp update from current pos)
        }
        _rate_sp[2] = rc_yaw * MAX_RATE_YAW_DPS * DEG2RAD;
        break;

    case FlightMode::FAILSAFE:
        // Descend straight down
        _angle_sp[0] = 0.0f;
        _angle_sp[1] = 0.0f;
        _rate_sp[2]  = 0.0f;
        _vel_sp_d    = 0.5f;  // 0.5 m/s descent
        break;

    default: break;
    }
}

// ---------------------------------------------------------------------------
//  check_arm_disarm()
// ---------------------------------------------------------------------------
void FlightController::check_arm_disarm(const CrsfReceiver &rc)
{
    bool arm_switch = (rc.channel(RC_CH_ARM) > CRSF_ARM_THRESH);
    bool throttle_low = (rc.channel(RC_CH_THROTTLE) < (CRSF_MIN + 150));

    if (_mode == FlightMode::DISARMED) {
        // Arm condition: arm switch goes high AND throttle is low
        if (arm_switch && !_arm_switch_prev && throttle_low) {
            DEBUG_SERIAL.println("[FC] ARMED");
            on_mode_change(FlightMode::STABILIZE);
        }
    } else {
        // Disarm condition: arm switch goes low
        if (!arm_switch && _arm_switch_prev) {
            DEBUG_SERIAL.println("[FC] DISARMED");
            on_mode_change(FlightMode::DISARMED);
        }
        // Auto-disarm: if throttle at minimum for DISARM_TIMEOUT_S
        if (throttle_low) {
            if (_disarm_timer_ms == 0)
                _disarm_timer_ms = millis();
            else if (millis() - _disarm_timer_ms > (uint32_t)(DISARM_TIMEOUT_S * 1000)) {
                DEBUG_SERIAL.println("[FC] Auto-disarmed (throttle timeout)");
                on_mode_change(FlightMode::DISARMED);
            }
        } else {
            _disarm_timer_ms = 0;
        }
    }

    _arm_switch_prev = arm_switch;
}

// ---------------------------------------------------------------------------
//  requested_mode()
// ---------------------------------------------------------------------------
FlightMode FlightController::requested_mode(const CrsfReceiver &rc) const
{
    int16_t mode_ch = rc.channel(RC_CH_MODE);
    if (mode_ch < 600)        return FlightMode::MANUAL;
    else if (mode_ch < 1000)  return FlightMode::STABILIZE;
    else if (mode_ch < 1400)  return FlightMode::ALT_HOLD;
    else                      return FlightMode::POS_HOLD;
}

// ---------------------------------------------------------------------------
//  on_mode_change()
// ---------------------------------------------------------------------------
void FlightController::on_mode_change(FlightMode new_mode)
{
    _prev_mode = _mode;
    _mode      = new_mode;

    // Reset integrators on mode change to avoid integral kick
    _rate_pids.reset_all();
    _alt_pos_pid.reset();
    _alt_vel_pid.reset();
    _vel_n_pid.reset();
    _vel_e_pid.reset();

    DEBUG_SERIAL.print("[FC] Mode → ");
    DEBUG_SERIAL.println(mode_name());
}

// ---------------------------------------------------------------------------
//  capture_position_setpoint()  — call when entering a hold mode
// ---------------------------------------------------------------------------
void FlightController::capture_position_setpoint(float pos_n, float pos_e,
                                                   float pos_d, float yaw)
{
    _pos_sp_n = pos_n;
    _pos_sp_e = pos_e;
    _alt_sp_d = pos_d;
    _yaw_sp   = yaw;
}

// =============================================================================
//  mix_quad_x()  —  X-frame quadrotor motor mixing
//
//  Normalised output [0..1] per motor, then convert to DSHOT.
//  Motor layout (viewed from above):
//    M1 (FR, CW):  throttle + roll - pitch - yaw
//    M2 (FL, CCW): throttle - roll - pitch + yaw
//    M3 (RL, CW):  throttle - roll + pitch - yaw
//    M4 (RR, CCW): throttle + roll + pitch + yaw
// =============================================================================
void FlightController::mix_quad_x(float throttle, float roll, float pitch,
                                   float yaw, MotorOutputs &out)
{
    float m[4];
    m[0] = throttle + roll - pitch - yaw;   // M1 front-right
    m[1] = throttle - roll - pitch + yaw;   // M2 front-left
    m[2] = throttle - roll + pitch - yaw;   // M3 rear-left
    m[3] = throttle + roll + pitch + yaw;   // M4 rear-right

    // Find max and min
    float mx = m[0], mn = m[0];
    for (int i = 1; i < 4; i++) {
        if (m[i] > mx) mx = m[i];
        if (m[i] < mn) mn = m[i];
    }

    // Desaturate: if any motor exceeds 1.0, reduce all proportionally
    if (mx > 1.0f) {
        float excess = mx - 1.0f;
        for (int i = 0; i < 4; i++) m[i] -= excess;
    }

    // Airmode: if any motor goes below minimum, boost all by the deficit
    // This preserves attitude control even at zero throttle
    if (_airmode_active) {
        float deficit = 0.0f;
        for (int i = 0; i < 4; i++)
            if (m[i] < 0.0f && -m[i] > deficit) deficit = -m[i];
        if (deficit > 0.0f)
            for (int i = 0; i < 4; i++) m[i] += deficit;
    } else {
        // Non-airmode: clip to zero
        for (int i = 0; i < 4; i++) if (m[i] < 0.0f) m[i] = 0.0f;
    }

    // Convert to DSHOT values
    for (int i = 0; i < 4; i++)
        out.m[i] = DShotMotors::normalised_to_dshot(m[i]);
}

const char* FlightController::mode_name() const
{
    switch (_mode) {
    case FlightMode::DISARMED:  return "DISARMED";
    case FlightMode::MANUAL:    return "MANUAL";
    case FlightMode::STABILIZE: return "STABILIZE";
    case FlightMode::ALT_HOLD:  return "ALT_HOLD";
    case FlightMode::POS_HOLD:  return "POS_HOLD";
    case FlightMode::FAILSAFE:  return "FAILSAFE";
    default: return "UNKNOWN";
    }
}
