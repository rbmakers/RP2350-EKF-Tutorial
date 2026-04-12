#pragma once
// =============================================================================
//  pid.h  —  PID controller with Betaflight-class features
//
//  Features:
//    • Standard P + I + D with integral anti-windup clamp
//    • Feedforward term (FF × setpoint_derivative)
//    • I-term Relax (suppresses I accumulation during rapid setpoint changes)
//      Implemented as: I is scaled by (1 - |setpoint_rate| / relax_threshold)
//    • D-term on measurement (not error) — avoids derivative kick on setpoint steps
//    • First-order low-pass filter on D-term input
//    • Throttle PID Attenuation (TPA) — reduces D gain at high throttle
//
//  Rate PID usage (2 kHz, raw gyro):
//    pid.compute(setpoint_rads, gyro_rads, dt, throttle_norm)
//
//  Angle PID usage (500 Hz, proportional only):
//    float rate_sp = angle_pid.compute_angle(setpoint_deg, actual_deg)
//
//  Position/altitude PID usage (10-50 Hz):
//    float vel_sp = pos_pid.compute(setpoint_m, actual_m, dt)
// =============================================================================

#include <Arduino.h>
#include <math.h>

// First-order IIR low-pass filter helper
class LowPassFilter {
public:
    LowPassFilter() : _alpha(1.0f), _state(0.0f) {}

    // Call once with sample rate and cutoff frequency (Hz)
    void configure(float sample_hz, float cutoff_hz) {
        if (cutoff_hz <= 0.0f || sample_hz <= 0.0f) { _alpha = 1.0f; return; }
        float rc = 1.0f / (2.0f * 3.14159265f * cutoff_hz);
        float dt = 1.0f / sample_hz;
        _alpha = dt / (rc + dt);
    }

    float update(float x) {
        _state += _alpha * (x - _state);
        return _state;
    }

    void reset(float val = 0.0f) { _state = val; }
    float state() const { return _state; }

private:
    float _alpha;
    float _state;
};

// =============================================================================
//  PID class — general purpose, used for rate/angle/altitude/position loops
// =============================================================================
class PID {
public:
    PID() : _kp(0), _ki(0), _kd(0), _kff(0),
            _integral(0), _prev_measurement(0),
            _imax(1.0f), _prev_setpoint(0),
            _iterm_relax_hz(0), _tpa_breakpoint(1.0f), _tpa_rate(0),
            _dterm_filtered(0), _first_run(true)
    {}

    // -------------------------------------------------------------------------
    //  Configure gains and options
    // -------------------------------------------------------------------------
    void set_gains(float kp, float ki, float kd, float kff = 0.0f) {
        _kp = kp; _ki = ki; _kd = kd; _kff = kff;
    }

    void set_imax(float imax) { _imax = imax; }

    // I-term Relax: suppress integral when setpoint is changing faster than
    // relax_hz (normalised units/sec).  0 = disabled.
    void set_iterm_relax(float relax_hz) { _iterm_relax_hz = relax_hz; }

    // TPA: D-term attenuation above throttle breakpoint
    void set_tpa(float breakpoint, float rate) {
        _tpa_breakpoint = breakpoint;
        _tpa_rate       = rate;
    }

    // D-term low-pass filter
    void configure_dterm_lpf(float sample_hz, float cutoff_hz) {
        _dterm_lpf.configure(sample_hz, cutoff_hz);
    }

    // -------------------------------------------------------------------------
    //  compute() — full PID (Rate loop)
    //    setpoint:    desired value (rad/s for rate PID)
    //    measurement: actual value  (raw gyro rad/s)
    //    dt:          timestep (s)
    //    throttle:    normalised throttle [0..1] for TPA (set 0 if unused)
    // -------------------------------------------------------------------------
    float compute(float setpoint, float measurement, float dt, float throttle = 0.0f)
    {
        if (_first_run) {
            _prev_measurement = measurement;
            _prev_setpoint    = setpoint;
            _dterm_lpf.reset(measurement);
            _first_run = false;
        }

        float error = setpoint - measurement;

        // ── P term ────────────────────────────────────────────────────────────
        float p_out = _kp * error;

        // ── I term with relax ─────────────────────────────────────────────────
        float setpoint_rate = (setpoint - _prev_setpoint) / dt;
        float relax_factor  = 1.0f;
        if (_iterm_relax_hz > 0.0f) {
            // Suppress integral proportional to how fast setpoint is changing
            float norm = fabsf(setpoint_rate) / _iterm_relax_hz;
            relax_factor = (norm < 1.0f) ? (1.0f - norm) : 0.0f;
        }

        _integral += _ki * error * dt * relax_factor;
        // Anti-windup: clamp integral
        _integral = constrain(_integral, -_imax, _imax);

        float i_out = _integral;

        // ── D term — on measurement (avoids derivative kick) ──────────────────
        // D = kd × (−dm/dt)   where m = measurement
        float measurement_filtered = _dterm_lpf.update(measurement);
        float d_input = (measurement_filtered - _prev_measurement) / dt;

        // TPA — throttle-based D attenuation
        float tpa_scale = 1.0f;
        if (_tpa_rate > 0.0f && throttle > _tpa_breakpoint) {
            tpa_scale = 1.0f - _tpa_rate * (throttle - _tpa_breakpoint)
                                          / (1.0f - _tpa_breakpoint);
            tpa_scale = constrain(tpa_scale, 0.0f, 1.0f);
        }

        float d_out = -_kd * d_input * tpa_scale;

        // ── Feedforward — on setpoint derivative ──────────────────────────────
        float ff_out = _kff * setpoint_rate;

        // ── Store for next cycle ──────────────────────────────────────────────
        _prev_measurement = measurement_filtered;
        _prev_setpoint    = setpoint;

        return p_out + i_out + d_out + ff_out;
    }

    // -------------------------------------------------------------------------
    //  compute_p_only() — pure proportional (angle loop, position P)
    // -------------------------------------------------------------------------
    float compute_p_only(float setpoint, float measurement) const {
        return _kp * (setpoint - measurement);
    }

    // -------------------------------------------------------------------------
    //  reset() — clears integrator and derivative state (on mode changes)
    // -------------------------------------------------------------------------
    void reset() {
        _integral         = 0.0f;
        _prev_measurement = 0.0f;
        _prev_setpoint    = 0.0f;
        _dterm_filtered   = 0.0f;
        _first_run        = true;
        _dterm_lpf.reset(0.0f);
    }

    float integral()    const { return _integral; }
    float kp()          const { return _kp; }

private:
    float _kp, _ki, _kd, _kff;
    float _integral;
    float _prev_measurement;
    float _prev_setpoint;
    float _imax;
    float _iterm_relax_hz;
    float _tpa_breakpoint;
    float _tpa_rate;
    float _dterm_filtered;
    bool  _first_run;
    LowPassFilter _dterm_lpf;
};

// =============================================================================
//  AxisPIDs — convenience wrapper for roll/pitch/yaw rate PIDs
// =============================================================================
struct AxisPIDs {
    PID roll, pitch, yaw;

    void configure_from_config(float sample_hz) {
        // Rate PIDs
        roll.set_gains (PID_RATE_ROLL_P,  PID_RATE_ROLL_I,  PID_RATE_ROLL_D,  PID_RATE_ROLL_FF);
        pitch.set_gains(PID_RATE_PITCH_P, PID_RATE_PITCH_I, PID_RATE_PITCH_D, PID_RATE_PITCH_FF);
        yaw.set_gains  (PID_RATE_YAW_P,   PID_RATE_YAW_I,   PID_RATE_YAW_D,   PID_RATE_YAW_FF);

        roll.set_imax (PID_RATE_IMAX);
        pitch.set_imax(PID_RATE_IMAX);
        yaw.set_imax  (PID_RATE_IMAX);

        roll.set_iterm_relax (ITERM_RELAX_CUTOFF);
        pitch.set_iterm_relax(ITERM_RELAX_CUTOFF);

        roll.set_tpa (TPA_BREAKPOINT, TPA_RATE);
        pitch.set_tpa(TPA_BREAKPOINT, TPA_RATE);

        roll.configure_dterm_lpf (sample_hz, DTERM_LPF_HZ);
        pitch.configure_dterm_lpf(sample_hz, DTERM_LPF_HZ);
        yaw.configure_dterm_lpf  (sample_hz, DTERM_LPF_HZ);
    }

    void reset_all() { roll.reset(); pitch.reset(); yaw.reset(); }
};
