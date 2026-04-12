// =============================================================================
//  INS_EKF.ino  —  GNSS + INS navigation with 15-state EKF on RP2350
//
//  Dual-core architecture:
//  ───────────────────────────────────────────────────────────────────────────
//  Core 0  │  Control-law core
//  500 Hz  │  • BMI088 SPI read
//          │  • EKF predict    (every IMU sample)
//          │  • EKF update     (when GPS seqlock signals new fix)
//          │  • Write EKF output to shared_state
//          │  • Serial telemetry @ 10 Hz
//  ───────────────────────────────────────────────────────────────────────────
//  Core 1  │  Peripheral core
//  ~UART   │  • GPS UART parser (runs as fast as bytes arrive)
//  speed   │  • Writes new GpsData into shared_state via seqlock
//  ───────────────────────────────────────────────────────────────────────────
//
//  First GPS fix:
//    On the first valid GPS fix, the EKF origin is set and the filter
//    initialises.  Until then, Core 0 waits in idle.
//
//  Wire connections (example for RB-RP2354A):
//    GP2  → BMI088 SCK
//    GP3  → BMI088 MOSI
//    GP4  → BMI088 MISO
//    GP5  → BMI088 ACC CS
//    GP6  → BMI088 GYR CS
//    GP0  → GPS TX  (RP2350 UART1 TX)
//    GP1  ← GPS RX  (RP2350 UART1 RX)
//    USB  → Host PC (debug Serial)
// =============================================================================

#include "config.h"
#include "shared_state.h"
#include "bmi088_spi.h"
#include "gps_parser.h"
#include "ekf_ins.h"
#include <math.h>

// =============================================================================
//  Global objects
// =============================================================================
SharedState g_shared;                        // inter-core data (shared memory)

BMI088    imu(IMU_SPI_PORT,
              PIN_BMI088_ACC_CS,
              PIN_BMI088_GYR_CS);

GpsParser gps(GPS_SERIAL);                   // lives on Core 1

EKF_INS   ekf;                               // lives on Core 0

// =============================================================================
//  Core 1 entry point
//  The arduino-pico framework calls setup1() + loop1() on Core 1.
// =============================================================================
void setup1()
{
    // Serial1 pin assignment must be done before .begin()
    GPS_SERIAL.setTX(PIN_GPS_TX);
    GPS_SERIAL.setRX(PIN_GPS_RX);

    gps.begin(GPS_BAUD);

    // Indicate Core 1 is ready
    // (Core 0 will be waiting in setup() until shared data is valid)
}

void loop1()
{
    // Feed the parser — runs at full UART speed
    if (gps.update()) {
        GpsData fix = gps.get_fix();

        // Write new fix atomically to shared memory
        g_shared.gps_write_begin();
        g_shared.gps = fix;
        g_shared.gps_write_end();
    }
}

// =============================================================================
//  Core 0 setup
// =============================================================================
void setup()
{
    DEBUG_SERIAL.begin(DEBUG_BAUD);
    delay(500);
    DEBUG_SERIAL.println("\n============================");
    DEBUG_SERIAL.println("  GNSS+INS EKF on RP2350");
    DEBUG_SERIAL.println("============================");

    // Initialise SPI bus
    SPI.setRX(PIN_SPI_MISO);
    SPI.setTX(PIN_SPI_MOSI);
    SPI.setSCK(PIN_SPI_SCK);

    // Initialise BMI088
    if (!imu.begin()) {
        DEBUG_SERIAL.println("FATAL: BMI088 init failed. Check wiring.");
        while (true) { delay(1000); }
    }

    // Initialise shared state
    memset(&g_shared, 0, sizeof(g_shared));

    // Wait for first valid GPS fix (EKF needs a NED origin)
    DEBUG_SERIAL.println("Waiting for GPS fix...");
    while (true) {
        GpsData fix;
        g_shared.gps_read(fix);
        if (fix.valid) {
            DEBUG_SERIAL.print("GPS fix acquired: lat=");
            DEBUG_SERIAL.print(fix.lat_deg, 7);
            DEBUG_SERIAL.print(" lon=");
            DEBUG_SERIAL.println(fix.lon_deg, 7);

            // Initialise EKF with GPS origin
            // TODO: initialise yaw from magnetometer if available
            ekf.init(fix.lat_deg, fix.lon_deg, fix.alt_msl_m, 0.0f);
            break;
        }
        delay(100);
    }

    DEBUG_SERIAL.println("EKF initialised. Starting navigation loop.");
}

// =============================================================================
//  Core 0 main loop
// =============================================================================

// Track last GPS timestamp to detect new fixes
static uint32_t last_gps_ts = 0;

// Telemetry counter
static uint32_t telem_counter = 0;

// Loop timing
static uint32_t next_imu_us = 0;
static const uint32_t IMU_PERIOD_US = 1000000UL / IMU_SAMPLE_HZ;  // 2000 µs @ 500 Hz

void loop()
{
    // ── Rate-limit to IMU_SAMPLE_HZ ──────────────────────────────────────────
    uint32_t now_us = micros();
    if (now_us < next_imu_us) return;
    next_imu_us = now_us + IMU_PERIOD_US;

    // ── Read IMU ──────────────────────────────────────────────────────────────
    ImuData imu_data;
    if (!imu.read(imu_data)) {
        DEBUG_SERIAL.println("[WARN] IMU read failed");
        return;
    }

    // ── EKF Predict ──────────────────────────────────────────────────────────
    ekf.predict(imu_data.ax_ms2,  imu_data.ay_ms2,  imu_data.az_ms2,
                imu_data.gx_rads, imu_data.gy_rads, imu_data.gz_rads,
                IMU_DT_S);

    // ── EKF Update — on new GPS fix ───────────────────────────────────────────
    GpsData fix;
    g_shared.gps_read(fix);

    if (fix.valid && (fix.timestamp_ms != last_gps_ts)) {
        // Check fix is not stale
        if ((millis() - fix.timestamp_ms) < GPS_MAX_AGE_MS) {
            ekf.update_gps(fix);
            last_gps_ts = fix.timestamp_ms;
        }
    }

    // ── Write EKF output to shared state ─────────────────────────────────────
    g_shared.ekf_write_begin();
    g_shared.ekf.pos_n = ekf.pos_n();
    g_shared.ekf.pos_e = ekf.pos_e();
    g_shared.ekf.pos_d = ekf.pos_d();
    g_shared.ekf.vel_n = ekf.vel_n();
    g_shared.ekf.vel_e = ekf.vel_e();
    g_shared.ekf.vel_d = ekf.vel_d();
    g_shared.ekf.roll  = ekf.roll();
    g_shared.ekf.pitch = ekf.pitch();
    g_shared.ekf.yaw   = ekf.yaw();
    g_shared.ekf.bias_ax = ekf.state()[IDX_BAX];
    g_shared.ekf.bias_ay = ekf.state()[IDX_BAY];
    g_shared.ekf.bias_az = ekf.state()[IDX_BAZ];
    g_shared.ekf.bias_gx = ekf.state()[IDX_BGX];
    g_shared.ekf.bias_gy = ekf.state()[IDX_BGY];
    g_shared.ekf.bias_gz = ekf.state()[IDX_BGZ];
    g_shared.ekf.pos_unc = ekf.sigma_pos_n();
    g_shared.ekf.timestamp_ms = millis();
    g_shared.ekf_write_end();

    // ── Serial telemetry @ ~10 Hz (every 50 iterations at 500 Hz) ─────────────
    telem_counter++;
    if (telem_counter >= 50) {
        telem_counter = 0;
        print_telemetry();
    }
}

// =============================================================================
//  Telemetry print
// =============================================================================
void print_telemetry()
{
    // CSV format — easy to plot in Serial Plotter or log to SD
    // Header: T_ms, pN, pE, pD, vN, vE, vD, roll_deg, pitch_deg, yaw_deg,
    //         bax, bay, baz, bgx_mdps, bgy_mdps, bgz_mdps, sigma_pos

    DEBUG_SERIAL.print(millis());
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(ekf.pos_n(), 3);
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(ekf.pos_e(), 3);
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(ekf.pos_d(), 3);
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(ekf.vel_n(), 3);
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(ekf.vel_e(), 3);
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(ekf.vel_d(), 3);
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(ekf.roll()  * RAD2DEG, 2);
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(ekf.pitch() * RAD2DEG, 2);
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(ekf.yaw()   * RAD2DEG, 2);
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(ekf.state()[IDX_BAX] * 1000.0f, 4);  // mg
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(ekf.state()[IDX_BGX] * RAD2DEG * 1000.0f, 4);  // mdps
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.println(ekf.sigma_pos_n(), 3);
}
