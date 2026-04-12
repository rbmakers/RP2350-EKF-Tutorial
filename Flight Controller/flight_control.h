#pragma once
// =============================================================================
//  flight_control.h  —  Flight mode state machine, PID cascade, motor mixer
//
//  Architecture (cascaded PID loops):
//
//    RC Input ──────────────────────────────────────────────────────────►
//                                                                        │
//  POSITION LOOP (10 Hz, GPS)                                           │
//    pos_error (m) → vel_setpoint (m/s)                                 │
//           ↓                                                            │
//  VELOCITY LOOP (50 Hz, EKF vel)                                       │
//    vel_error (m/s) → lean_angle_setpoint (deg)                        │
//           ↓                                                            │
//  ANGLE LOOP (500 Hz, EKF attitude)                                    │
//    angle_error (deg) → rate_setpoint (deg/s)                          │
//           ↓                                                            │
//  RATE LOOP (2 kHz, raw gyro)                                          │
//    rate_error (deg/s) → normalised torque output [-1..+1]             │
//           ↓                                                            │
//  MIXER → Motor outputs [0..1] → DSHOT throttle values                 │
//
//  Flight modes (selected by AUX2 RC channel):
//    DISARMED   Motors stopped. Arming requires: AUX1 high + throttle low
//    MANUAL     Rate mode only. Pilot commands angular rates directly.
//    STABILIZE  Angle mode (roll/pitch). Pilot commands lean angles.
//    ALT_HOLD   Stabilize + barometer altitude hold.
//    POS_HOLD   Alt_hold + GPS position hold (horizontal).
//
//  Failsafe:
//    On RC loss (> RC_FAILSAFE_MS ms), descend at controlled rate
//    and disarm after landing timeout.
// =============================================================================

#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "pid.h"
#include "crsf.h"
#include "ekf_ins.h"

// =============================================================================
//  Flight mode enum
// =============================================================================
enum class FlightMode : uint8_t {
    DISARMED   = 0,
    MANUAL     = 1,   // rate mode
    STABILIZE  = 2,   // angle mode
    ALT_HOLD   = 3,   // angle + altitude
    POS_HOLD   = 4,   // angle + altitude + position
    FAILSAFE   = 5,   // emergency descent
};

// =============================================================================
//  MotorOutputs — raw values sent to DSHOT
// =============================================================================
struct MotorOutputs {
    uint16_t m[4];  // DSHOT throttle values for motors 1-4

    void set_all(uint16_t v) { m[0]=m[1]=m[2]=m[3]=v; }
    void disarm()            { set_all(DSHOT_DISARM_VAL); }
};

// =============================================================================
//  FlightController class
// =============================================================================
class FlightController {
public:
    FlightController();

    // Call once from Core 0 setup()
    void begin();

    // -------------------------------------------------------------------------
    //  update_rate_loop() — MUST be called every 2 kHz tick (Core 0)
    //  Uses raw gyro for rate PID. Outputs motor values.
    //    gyro_rads: body-frame gyro [gx, gy, gz] (rad/s)
    //    throttle_norm: current normalised throttle [0..1] (for TPA)
    // -------------------------------------------------------------------------
    void update_rate_loop(const float gyro_rads[3], float throttle_norm,
                          float dt, MotorOutputs &out);

    // -------------------------------------------------------------------------
    //  update_angle_loop() — call every 4th tick (500 Hz, Core 0)
    //  Uses EKF attitude. Updates roll/pitch rate setpoints.
    //    roll_rad, pitch_rad, yaw_rad: EKF Euler angles
    // -------------------------------------------------------------------------
    void update_angle_loop(float roll_rad, float pitch_rad, float yaw_rad);

    // -------------------------------------------------------------------------
    //  update_altitude_loop() — call every 40th tick (50 Hz, Core 0)
    //  Uses EKF vertical position and velocity.
    //    pos_d: NED Down position (m, positive down)
    //    vel_d: NED Down velocity (m/s, positive down)
    // -------------------------------------------------------------------------
    void update_altitude_loop(float pos_d, float vel_d, float dt);

    // -------------------------------------------------------------------------
    //  update_position_loop() — call every 200th tick (10 Hz, Core 0)
    //  Uses EKF NED position and velocity.
    //    pos_n/e: NED position (m)
    //    vel_n/e: NED velocity (m/s)
    //    yaw_rad: heading (for rotating pos error to body frame)
    // -------------------------------------------------------------------------
    void update_position_loop(float pos_n, float pos_e,
                               float vel_n, float vel_e,
                               float yaw_rad, float dt);

    // -------------------------------------------------------------------------
    //  update_from_rc() — read RC inputs and update setpoints/mode
    //  Call at angle loop rate (500 Hz) to keep mode transitions smooth
    // -------------------------------------------------------------------------
    void update_from_rc(const CrsfReceiver &rc);

    // -------------------------------------------------------------------------
    //  Accessors
    // -------------------------------------------------------------------------
    FlightMode mode()      const { return _mode; }
    bool       is_armed()  const { return _mode != FlightMode::DISARMED
                                        && _mode != FlightMode::FAILSAFE; }

    // Setpoints (for telemetry/debugging)
    float rate_sp_roll()   const { return _rate_sp[0]; }
    float rate_sp_pitch()  const { return _rate_sp[1]; }
    float rate_sp_yaw()    const { return _rate_sp[2]; }
    float throttle_norm()  const { return _throttle_norm; }

    const char* mode_name() const;

private:
    FlightMode _mode;
    FlightMode _prev_mode;

    // ── Setpoints (flow from outer→inner loops) ───────────────────────────────
    float _rate_sp[3];           // roll/pitch/yaw rate setpoint (rad/s)
    float _angle_sp[2];          // roll/pitch angle setpoint (rad)
    float _vel_sp_n, _vel_sp_e;  // horizontal velocity setpoint (m/s)
    float _vel_sp_d;             // vertical velocity setpoint (m/s)
    float _pos_sp_n, _pos_sp_e;  // position hold setpoint NED (m)
    float _alt_sp_d;             // altitude setpoint (NED Down, m)
    float _yaw_sp;               // yaw angle setpoint (rad)
    float _throttle_norm;        // base throttle (normalised)

    // ── PID objects ───────────────────────────────────────────────────────────
    AxisPIDs   _rate_pids;       // roll/pitch/yaw rate PIDs (2 kHz)
    PID        _angle_roll;      // angle → rate setpoint (P only)
    PID        _angle_pitch;
    PID        _alt_pos_pid;     // altitude position error → vel setpoint
    PID        _alt_vel_pid;     // altitude velocity error → throttle delta
    PID        _vel_n_pid;       // north velocity error → pitch angle setpoint
    PID        _vel_e_pid;       // east  velocity error → roll  angle setpoint

    // ── Gyro low-pass filter ──────────────────────────────────────────────────
    LowPassFilter _gyro_lpf[3];

    // ── Arming state ──────────────────────────────────────────────────────────
    bool     _arm_switch_prev;
    uint32_t _arm_timer_ms;
    uint32_t _disarm_timer_ms;

    // ── Airmode ───────────────────────────────────────────────────────────────
    bool _airmode_active;

    // ── Mixer ─────────────────────────────────────────────────────────────────
    void mix_quad_x(float throttle, float roll_out, float pitch_out,
                    float yaw_out, MotorOutputs &out);

    // Arm/disarm logic
    void check_arm_disarm(const CrsfReceiver &rc);

    // Mode transition logic
    FlightMode requested_mode(const CrsfReceiver &rc) const;
    void on_mode_change(FlightMode new_mode);

    // Hold current position when entering a hold mode
    void capture_position_setpoint(float pos_n, float pos_e,
                                   float pos_d, float yaw);
};
