#pragma once
// =============================================================================
//  shared_state.h  —  Seqlock inter-core shared data (RP2350 dual-core)
//
//  Architecture:
//    Core 0 (writer): EKF output, IMU data
//    Core 1 (writer): GPS parsed fix, Baro data, Mag data
//    Both cores (reader): read the other core's data
//
//  Seqlock protocol:
//    Writer: seq++  (→ odd, signals write in progress)
//            DMB SY
//            [write data]
//            DMB SY
//            seq++  (→ even, signals write complete)
//
//    Reader: do {
//              seq1 = seq
//              DMB SY
//              [copy data]
//              DMB SY
//              seq2 = seq
//            } while (seq1 & 1 || seq1 != seq2)
//
//  Memory ordering:
//    RP2350 (Cortex-M33) has a weakly-ordered memory model.
//    'DMB SY' (Data Memory Barrier, full system) ensures all prior
//    memory accesses are globally observed before any later ones.
//    Without DMB, the compiler or hardware could reorder loads/stores
//    across the sequence counter, defeating the protocol.
// =============================================================================

#include <Arduino.h>
#include "gps_parser.h"

typedef volatile uint32_t seqlock_t;

static inline void dmb_sy()
{
    __asm__ volatile ("dmb sy" ::: "memory");
}

// ---------------------------------------------------------------------------
//  Sensor data shared from Core 1 → Core 0
// ---------------------------------------------------------------------------
struct SharedGps {
    seqlock_t seq;
    GpsData   data;
};

struct SharedBaro {
    seqlock_t seq;
    float     altitude_m;
    float     pressure_Pa;
    uint32_t  timestamp_ms;
    bool      valid;
};

struct SharedMag {
    seqlock_t seq;
    float     mx_norm;      // normalised body-frame field
    float     my_norm;
    float     mz_norm;
    float     strength_uT;  // raw field magnitude (for health monitoring)
    uint32_t  timestamp_ms;
    bool      valid;
};

// ---------------------------------------------------------------------------
//  EKF output shared from Core 0 → Core 1 (telemetry)
// ---------------------------------------------------------------------------
struct SharedEkf {
    seqlock_t seq;
    float pos_n, pos_e, pos_d;
    float vel_n, vel_e, vel_d;
    float q0, q1, q2, q3;
    float roll, pitch, yaw;             // Euler (rad) computed for telemetry
    float sigma_pos;                    // 1-sigma North position uncertainty
    float sigma_yaw;
    float bias_ax, bias_ay, bias_az;
    float bias_gx, bias_gy, bias_gz;
    uint32_t timestamp_ms;
};

// ---------------------------------------------------------------------------
//  SharedState container with seqlock read/write helpers
// ---------------------------------------------------------------------------
struct SharedState {
    SharedGps  gps;
    SharedBaro baro;
    SharedMag  mag;
    SharedEkf  ekf;

    // GPS
    void gps_write(const GpsData &d) {
        gps.seq++;  dmb_sy();
        gps.data = d;
        dmb_sy();  gps.seq++;
    }
    void gps_read(GpsData &out) {
        uint32_t s1, s2;
        do {
            s1 = gps.seq;  dmb_sy();
            out = gps.data;
            dmb_sy();      s2 = gps.seq;
        } while ((s1 & 1) || s1 != s2);
    }

    // Baro
    void baro_write(float alt, float pres, uint32_t ts, bool ok) {
        baro.seq++;  dmb_sy();
        baro.altitude_m  = alt;
        baro.pressure_Pa = pres;
        baro.timestamp_ms = ts;
        baro.valid       = ok;
        dmb_sy();  baro.seq++;
    }
    bool baro_read(float &alt, float &pres, uint32_t &ts) {
        uint32_t s1, s2;
        bool ok;
        do {
            s1 = baro.seq;  dmb_sy();
            alt  = baro.altitude_m;
            pres = baro.pressure_Pa;
            ts   = baro.timestamp_ms;
            ok   = baro.valid;
            dmb_sy();  s2 = baro.seq;
        } while ((s1 & 1) || s1 != s2);
        return ok;
    }

    // Mag
    void mag_write(float mx, float my, float mz, float str, uint32_t ts, bool ok) {
        mag.seq++;  dmb_sy();
        mag.mx_norm = mx;  mag.my_norm = my;  mag.mz_norm = mz;
        mag.strength_uT = str;
        mag.timestamp_ms = ts;
        mag.valid = ok;
        dmb_sy();  mag.seq++;
    }
    bool mag_read(float &mx, float &my, float &mz, uint32_t &ts) {
        uint32_t s1, s2;
        bool ok;
        do {
            s1 = mag.seq;  dmb_sy();
            mx = mag.mx_norm;  my = mag.my_norm;  mz = mag.mz_norm;
            ts = mag.timestamp_ms;
            ok = mag.valid;
            dmb_sy();  s2 = mag.seq;
        } while ((s1 & 1) || s1 != s2);
        return ok;
    }

    // EKF output
    void ekf_write(const SharedEkf &e) {
        ekf.seq++;  dmb_sy();
        // Copy all fields except seq
        ekf.pos_n = e.pos_n;  ekf.pos_e = e.pos_e;  ekf.pos_d = e.pos_d;
        ekf.vel_n = e.vel_n;  ekf.vel_e = e.vel_e;  ekf.vel_d = e.vel_d;
        ekf.q0 = e.q0;  ekf.q1 = e.q1;  ekf.q2 = e.q2;  ekf.q3 = e.q3;
        ekf.roll = e.roll;  ekf.pitch = e.pitch;  ekf.yaw = e.yaw;
        ekf.sigma_pos = e.sigma_pos;  ekf.sigma_yaw = e.sigma_yaw;
        ekf.bias_ax = e.bias_ax;  ekf.bias_ay = e.bias_ay;  ekf.bias_az = e.bias_az;
        ekf.bias_gx = e.bias_gx;  ekf.bias_gy = e.bias_gy;  ekf.bias_gz = e.bias_gz;
        ekf.timestamp_ms = e.timestamp_ms;
        dmb_sy();  ekf.seq++;
    }
    void ekf_read(SharedEkf &out) {
        uint32_t s1, s2;
        do {
            s1 = ekf.seq;  dmb_sy();
            out = ekf;
            dmb_sy();  s2 = ekf.seq;
        } while ((s1 & 1) || s1 != s2);
    }
};

extern SharedState g_shared;
