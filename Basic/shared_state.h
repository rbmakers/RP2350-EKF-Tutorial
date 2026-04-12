#pragma once
// =============================================================================
//  shared_state.h  —  Lock-free seqlock for RP2350 dual-core data sharing
//
//  Problem:
//    Core 1 (GPS parser) writes GpsData continuously.
//    Core 0 (EKF loop) reads GpsData at lower rate.
//    Without synchronisation, Core 0 could read a partially written struct
//    → corrupted fix data → EKF divergence.
//
//  Solution: Seqlock
//    Writer increments a sequence counter (odd during write, even when done).
//    Reader retries if it sees an odd counter or if counter changed mid-read.
//    Requires no OS mutex — very fast for single-writer / single-reader.
//
//  Memory ordering on RP2350 (Cortex-M33):
//    The M33 has a weakly-ordered memory model.
//    DMB (Data Memory Barrier) ensures all memory accesses before the barrier
//    are visible to the other core before any access after the barrier.
//    We use:  __dmb()  which expands to  DMB SY  (full system barrier).
//
//  Usage — Core 1 (writer):
//    g_shared.gps_write_begin();
//    g_shared.gps = new_fix;          // write the struct
//    g_shared.gps_write_end();
//
//  Usage — Core 0 (reader):
//    GpsData fix;
//    g_shared.gps_read(fix);          // retries until consistent
// =============================================================================

#include <Arduino.h>
#include "gps_parser.h"

// Volatile sequence counter type
typedef volatile uint32_t seqlock_t;

// ---------------------------------------------------------------------------
//  Inline DMB helper
// ---------------------------------------------------------------------------
static inline void dmb_sy()
{
    __asm__ volatile ("dmb sy" ::: "memory");
}

// ---------------------------------------------------------------------------
//  SharedState structure
// ---------------------------------------------------------------------------
struct SharedState {
    // GPS fix — written by Core 1, read by Core 0
    seqlock_t gps_seq;     // sequence counter (odd = write in progress)
    GpsData   gps;         // latest parsed GPS fix

    // EKF output — written by Core 0, read optionally by telemetry on Core 1
    seqlock_t ekf_seq;
    struct EkfOut {
        float pos_n, pos_e, pos_d;     // NED position (m from origin)
        float vel_n, vel_e, vel_d;     // NED velocity (m/s)
        float roll, pitch, yaw;        // Euler angles (rad)
        float bias_ax, bias_ay, bias_az;
        float bias_gx, bias_gy, bias_gz;
        float pos_unc;                 // sqrt(P[0,0]) — 1-sigma position uncertainty (m)
        uint32_t timestamp_ms;
    } ekf;

    // ---------------------------------------------------------------------------
    //  GPS seqlock write (called from Core 1)
    // ---------------------------------------------------------------------------
    void gps_write_begin() {
        gps_seq++;          // make odd → signals write in progress
        dmb_sy();           // ensure counter write is visible before data write
    }

    void gps_write_end() {
        dmb_sy();           // ensure data writes complete before counter increment
        gps_seq++;          // make even → write complete
    }

    // ---------------------------------------------------------------------------
    //  GPS seqlock read (called from Core 0)
    //  Spins until a consistent read is obtained.
    //  Returns quickly under normal operation (GPS rarely writes mid-read).
    // ---------------------------------------------------------------------------
    void gps_read(GpsData &out) {
        uint32_t seq1, seq2;
        do {
            seq1 = gps_seq;
            dmb_sy();       // ensure seq1 is loaded before data
            out = gps;      // copy struct
            dmb_sy();       // ensure data loaded before seq2
            seq2 = gps_seq;
        } while ((seq1 & 1) || (seq1 != seq2));
        // Retry condition:
        //   seq1 & 1  → caught write in progress
        //   seq1 != seq2 → counter changed during our read
    }

    // ---------------------------------------------------------------------------
    //  EKF output write (called from Core 0)
    // ---------------------------------------------------------------------------
    void ekf_write_begin() { ekf_seq++; dmb_sy(); }
    void ekf_write_end()   { dmb_sy(); ekf_seq++; }

    // EKF output read (from Core 1 for telemetry)
    void ekf_read(EkfOut &out) {
        uint32_t seq1, seq2;
        do {
            seq1 = ekf_seq;
            dmb_sy();
            out = ekf;
            dmb_sy();
            seq2 = ekf_seq;
        } while ((seq1 & 1) || (seq1 != seq2));
    }
};

// Global shared state instance — defined in INS_EKF.ino
extern SharedState g_shared;
