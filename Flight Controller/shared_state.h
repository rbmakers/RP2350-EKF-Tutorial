#pragma once
// =============================================================================
//  shared_state.h  —  Seqlock inter-core shared memory (RP2354A dual-core)
//
//  Core 0 writes: EKF output
//  Core 1 writes: GPS fix, Baro data, Mag data, CRSF RC channels
//  Both cores read: the other core's data
//
//  All transfers use the seqlock protocol with DMB SY memory barriers.
//  See INS_EKF_v2/shared_state.h for protocol rationale.
// =============================================================================

#include <Arduino.h>
#include "gps_parser.h"

typedef volatile uint32_t seqlock_t;

static inline void dmb_sy() { __asm__ volatile ("dmb sy" ::: "memory"); }

// ---------------------------------------------------------------------------
//  GPS (Core 1 → Core 0)
// ---------------------------------------------------------------------------
struct SharedGps {
    seqlock_t seq;
    GpsData   data;
};

// ---------------------------------------------------------------------------
//  Barometer (Core 1 → Core 0)
// ---------------------------------------------------------------------------
struct SharedBaro {
    seqlock_t seq;
    float    altitude_m;
    float    pressure_Pa;
    uint32_t timestamp_ms;
    bool     valid;
};

// ---------------------------------------------------------------------------
//  Magnetometer (Core 1 → Core 0)
// ---------------------------------------------------------------------------
struct SharedMag {
    seqlock_t seq;
    float    mx_norm;
    float    my_norm;
    float    mz_norm;
    float    strength_uT;
    uint32_t timestamp_ms;
    bool     valid;
};

// ---------------------------------------------------------------------------
//  CRSF RC channels (Core 1 → Core 0)
//  Raw CRSF values (172–1811). Core 0 calls normalisation helpers.
// ---------------------------------------------------------------------------
struct SharedCrsf {
    seqlock_t seq;
    int16_t  channels[16];
    uint8_t  link_quality;      // 0–100 %
    uint8_t  rssi;
    uint32_t timestamp_ms;
    bool     failsafe;
};

// ---------------------------------------------------------------------------
//  EKF + flight state (Core 0 → Core 1, for telemetry)
// ---------------------------------------------------------------------------
struct SharedFcsState {
    seqlock_t seq;
    // Navigation
    float pos_n, pos_e, pos_d;
    float vel_n, vel_e, vel_d;
    float q0, q1, q2, q3;
    float roll, pitch, yaw;         // rad
    float sigma_pos;                // 1-sigma position (m)
    // Flight control
    uint8_t  flight_mode;           // FlightMode enum cast
    bool     armed;
    float    throttle_norm;
    uint16_t motor_dshot[4];
    float    rate_sp[3];            // roll/pitch/yaw rate setpoints (rad/s)
    // Biases
    float bias_ax, bias_ay, bias_az;
    float bias_gx, bias_gy, bias_gz;
    uint32_t timestamp_ms;
};

// =============================================================================
//  SharedState container
// =============================================================================
struct SharedState {
    SharedGps     gps;
    SharedBaro    baro;
    SharedMag     mag;
    SharedCrsf    crsf;
    SharedFcsState fcs;

    // ── GPS ──────────────────────────────────────────────────────────────────
    void gps_write(const GpsData &d) {
        gps.seq++;  dmb_sy();
        gps.data = d;
        dmb_sy();   gps.seq++;
    }
    void gps_read(GpsData &out) {
        uint32_t s1, s2;
        do { s1=gps.seq; dmb_sy(); out=gps.data; dmb_sy(); s2=gps.seq; }
        while ((s1&1) || s1!=s2);
    }

    // ── Baro ─────────────────────────────────────────────────────────────────
    void baro_write(float alt, float pres, uint32_t ts, bool ok) {
        baro.seq++;  dmb_sy();
        baro.altitude_m=alt; baro.pressure_Pa=pres;
        baro.timestamp_ms=ts; baro.valid=ok;
        dmb_sy();   baro.seq++;
    }
    bool baro_read(float &alt, float &pres, uint32_t &ts) {
        uint32_t s1, s2; bool ok;
        do {
            s1=baro.seq; dmb_sy();
            alt=baro.altitude_m; pres=baro.pressure_Pa;
            ts=baro.timestamp_ms; ok=baro.valid;
            dmb_sy(); s2=baro.seq;
        } while ((s1&1) || s1!=s2);
        return ok;
    }

    // ── Mag ──────────────────────────────────────────────────────────────────
    void mag_write(float mx, float my, float mz, float str, uint32_t ts, bool ok) {
        mag.seq++;  dmb_sy();
        mag.mx_norm=mx; mag.my_norm=my; mag.mz_norm=mz;
        mag.strength_uT=str; mag.timestamp_ms=ts; mag.valid=ok;
        dmb_sy();   mag.seq++;
    }
    bool mag_read(float &mx, float &my, float &mz, uint32_t &ts) {
        uint32_t s1, s2; bool ok;
        do {
            s1=mag.seq; dmb_sy();
            mx=mag.mx_norm; my=mag.my_norm; mz=mag.mz_norm;
            ts=mag.timestamp_ms; ok=mag.valid;
            dmb_sy(); s2=mag.seq;
        } while ((s1&1) || s1!=s2);
        return ok;
    }

    // ── CRSF ─────────────────────────────────────────────────────────────────
    void crsf_write(const int16_t ch[16], uint8_t lq, uint8_t rssi_val,
                    uint32_t ts, bool fs) {
        crsf.seq++;  dmb_sy();
        memcpy(crsf.channels, ch, 16*sizeof(int16_t));
        crsf.link_quality=lq; crsf.rssi=rssi_val;
        crsf.timestamp_ms=ts; crsf.failsafe=fs;
        dmb_sy();   crsf.seq++;
    }
    void crsf_read(int16_t ch[16], uint8_t &lq, uint8_t &rssi_val,
                   uint32_t &ts, bool &fs) {
        uint32_t s1, s2;
        do {
            s1=crsf.seq; dmb_sy();
            memcpy(ch, crsf.channels, 16*sizeof(int16_t));
            lq=crsf.link_quality; rssi_val=crsf.rssi;
            ts=crsf.timestamp_ms; fs=crsf.failsafe;
            dmb_sy(); s2=crsf.seq;
        } while ((s1&1) || s1!=s2);
    }

    // ── FCS state (EKF + flight control output) ───────────────────────────────
    void fcs_write(const SharedFcsState &s) {
        fcs.seq++;  dmb_sy();
        // Copy all fields except seq
        fcs.pos_n=s.pos_n; fcs.pos_e=s.pos_e; fcs.pos_d=s.pos_d;
        fcs.vel_n=s.vel_n; fcs.vel_e=s.vel_e; fcs.vel_d=s.vel_d;
        fcs.q0=s.q0; fcs.q1=s.q1; fcs.q2=s.q2; fcs.q3=s.q3;
        fcs.roll=s.roll; fcs.pitch=s.pitch; fcs.yaw=s.yaw;
        fcs.sigma_pos=s.sigma_pos;
        fcs.flight_mode=s.flight_mode; fcs.armed=s.armed;
        fcs.throttle_norm=s.throttle_norm;
        memcpy(fcs.motor_dshot, s.motor_dshot, sizeof(s.motor_dshot));
        memcpy(fcs.rate_sp, s.rate_sp, sizeof(s.rate_sp));
        fcs.bias_ax=s.bias_ax; fcs.bias_ay=s.bias_ay; fcs.bias_az=s.bias_az;
        fcs.bias_gx=s.bias_gx; fcs.bias_gy=s.bias_gy; fcs.bias_gz=s.bias_gz;
        fcs.timestamp_ms=s.timestamp_ms;
        dmb_sy();   fcs.seq++;
    }
    void fcs_read(SharedFcsState &out) {
        uint32_t s1, s2;
        do { s1=fcs.seq; dmb_sy(); out=fcs; dmb_sy(); s2=fcs.seq; }
        while ((s1&1) || s1!=s2);
    }
};

extern SharedState g_shared;
