// =============================================================================
//  INS_EKF_v2.ino  —  Advanced GNSS+INS Navigation System on RP2350
//
//  Sensors:
//    BMI088  — 6-DOF IMU (accel + gyro),  SPI
//    BMM350  — 3-axis magnetometer,        I2C
//    BMP580  — barometric altimeter,       I2C
//    GNSS    — UART NMEA (u-blox M8/M9/F9)
//
//  EKF:
//    16-state quaternion INS/GNSS filter
//    Update sources and rates:
//      Predict:     IMU @ 500 Hz   (Core 0)
//      GPS update:  GNSS ~  5 Hz   (Core 0, on new fix)
//      Baro update: BMP580 @ 50 Hz (Core 0)
//      Mag update:  BMM350 @100 Hz (Core 0)
//
//  Dual-core architecture:
//  ───────────────────────────────────────────────────────────────────────────
//  Core 0  │ Navigation core  (real-time)
//          │  • 500 Hz rate-limited loop
//          │  • BMI088 SPI read
//          │  • EKF predict
//          │  • EKF update (GPS / Baro / Mag) when new data available
//          │  • Write EKF output to shared_state
//          │  • Serial telemetry @ 10 Hz
//  ───────────────────────────────────────────────────────────────────────────
//  Core 1  │ Peripheral core  (I/O intensive)
//          │  • GPS UART parsing (continuous)
//          │  • BMP580 I2C read  @  50 Hz
//          │  • BMM350 I2C read  @ 100 Hz
//          │  • Write all sensor data to shared_state via seqlock
//  ───────────────────────────────────────────────────────────────────────────
//
//  NED magnetic reference field (m_ned):
//    Must be provided for the magnetometer update.
//    Options:
//    1. Fetch from World Magnetic Model (WMM) given lat/lon/alt
//    2. Measure at startup: average many samples with drone level+heading known
//    3. Calibration flight: figure-8 manoeuvre then solve
//    Here we use a placeholder that the user should update for their location.
//    For Taiwan (~25°N): declination ~−4.5°, inclination ~34°
//    m_ned ≈ [cos(34°)*cos(4.5°), -cos(34°)*sin(4.5°), sin(34°)] ≈ [0.827, -0.065, 0.559]
// =============================================================================

#include "config.h"
#include "shared_state.h"
#include "bmi088_spi.h"
#include "bmm350.h"
#include "bmp580.h"
#include "gps_parser.h"
#include "ekf_ins.h"
#include <math.h>

// =============================================================================
//  Global objects
// =============================================================================
SharedState g_shared;

// Core 0 objects
BMI088      imu(IMU_SPI_PORT, PIN_BMI088_ACC_CS, PIN_BMI088_GYR_CS);
EKF_INS     ekf;

// Core 1 objects
GpsParser   gps(GPS_SERIAL);
BMM350      mag(IMU_I2C_PORT, BMM350_I2C_ADDR);
BMP580      baro(IMU_I2C_PORT, BMP580_I2C_ADDR);

// NED reference magnetic field (normalised unit vector)
// IMPORTANT: Update these values for your location using the WMM or calibration.
// Use: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
static const float g_mag_ned[3] = { 0.827f, -0.065f, 0.559f };  // Taiwan example

// =============================================================================
//  Core 1 — Peripheral I/O
// =============================================================================
void setup1()
{
    // I2C bus — must initialise on the core that will use it
    IMU_I2C_PORT.setSDA(PIN_I2C_SDA);
    IMU_I2C_PORT.setSCL(PIN_I2C_SCL);
    IMU_I2C_PORT.begin();
    IMU_I2C_PORT.setClock(IMU_I2C_FREQ);

    // GPS UART
    GPS_SERIAL.setTX(PIN_GPS_TX);
    GPS_SERIAL.setRX(PIN_GPS_RX);
    gps.begin(GPS_BAUD);

    // Magnetometer
    if (!mag.begin()) {
        // Continue without magnetometer — EKF will run without mag updates
        DEBUG_SERIAL.println("[WARN] BMM350 init failed");
    }
    // Apply hard-iron offsets from config (set after calibration)
    mag.set_hard_iron(MAG_HARDIRON_X, MAG_HARDIRON_Y, MAG_HARDIRON_Z);

    // Barometer
    if (!baro.begin()) {
        DEBUG_SERIAL.println("[WARN] BMP580 init failed");
    }
}

// Core 1 timing
static uint32_t _c1_baro_next_ms = 0;
static uint32_t _c1_mag_next_ms  = 0;

void loop1()
{
    uint32_t now = millis();

    // ── GPS: run at full UART speed ───────────────────────────────────────────
    if (gps.update()) {
        GpsData fix = gps.get_fix();
        g_shared.gps_write(fix);
    }

    // ── Barometer: 50 Hz ─────────────────────────────────────────────────────
    if (now >= _c1_baro_next_ms) {
        _c1_baro_next_ms = now + (1000 / BARO_SAMPLE_HZ);
        BaroData bd;
        if (baro.read(bd)) {
            g_shared.baro_write(bd.altitude_m, bd.pressure_Pa,
                                bd.timestamp_ms, bd.valid);
        }
    }

    // ── Magnetometer: 100 Hz ─────────────────────────────────────────────────
    if (now >= _c1_mag_next_ms) {
        _c1_mag_next_ms = now + (1000 / MAG_SAMPLE_HZ);
        MagData md;
        if (mag.read(md)) {
            float strength = sqrtf(md.bx_uT*md.bx_uT +
                                   md.by_uT*md.by_uT +
                                   md.bz_uT*md.bz_uT);
            g_shared.mag_write(md.bx_norm, md.by_norm, md.bz_norm,
                               strength, md.timestamp_ms, md.valid);
        }
    }
}

// =============================================================================
//  Core 0 — Navigation (EKF)
// =============================================================================
void setup()
{
    DEBUG_SERIAL.begin(DEBUG_BAUD);
    delay(500);
    DEBUG_SERIAL.println("\n======================================");
    DEBUG_SERIAL.println("  Advanced GNSS+INS / Quaternion EKF");
    DEBUG_SERIAL.println("  RP2350  |  BMI088+BMM350+BMP580+GPS");
    DEBUG_SERIAL.println("======================================");

    // SPI for BMI088
    SPI.setRX(PIN_SPI_MISO);
    SPI.setTX(PIN_SPI_MOSI);
    SPI.setSCK(PIN_SPI_SCK);

    if (!imu.begin()) {
        DEBUG_SERIAL.println("FATAL: BMI088 init failed");
        while (true) delay(1000);
    }

    memset(&g_shared, 0, sizeof(g_shared));

    // ── Wait for first valid GPS fix ──────────────────────────────────────────
    DEBUG_SERIAL.println("Waiting for GPS fix...");
    GpsData first_fix;
    while (true) {
        g_shared.gps_read(first_fix);
        if (first_fix.valid) break;
        delay(200);
    }
    DEBUG_SERIAL.print("GPS fix: lat=");
    DEBUG_SERIAL.print(first_fix.lat_deg, 7);
    DEBUG_SERIAL.print(" lon=");
    DEBUG_SERIAL.print(first_fix.lon_deg, 7);
    DEBUG_SERIAL.print(" alt=");
    DEBUG_SERIAL.println(first_fix.alt_msl_m, 1);

    // ── Align barometer to GPS altitude ───────────────────────────────────────
    // Wait for baro to produce a valid reading, then compute P₀
    DEBUG_SERIAL.println("Aligning barometer to GPS altitude...");
    for (int tries = 0; tries < 50; tries++) {
        float alt, pres; uint32_t ts;
        if (g_shared.baro_read(alt, pres, ts)) {
            // Compute sea-level pressure so that baro altitude matches GPS
            // alt = 44330 * (1 - (P/P0)^0.190295)
            // P0 = P / (1 - alt/44330)^(1/0.190295)
            float ratio = 1.0f - first_fix.alt_msl_m / 44330.0f;
            float p0 = pres / powf(ratio, 5.255f);
            baro.set_sea_level_pressure(p0);
            DEBUG_SERIAL.print("Baro P0 = ");
            DEBUG_SERIAL.println(p0, 1);
            break;
        }
        delay(50);
    }

    // ── Estimate initial heading from magnetometer ────────────────────────────
    float init_yaw = 0.0f;
    {
        float mx, my, mz; uint32_t ts;
        if (g_shared.mag_read(mx, my, mz, ts)) {
            // With level attitude: yaw = atan2(-my_ned, mx_ned)
            // In body frame (level): mx≈mN, my≈mE → heading = atan2(my, mx)
            init_yaw = atan2f(my, mx);
            DEBUG_SERIAL.print("Initial yaw from mag: ");
            DEBUG_SERIAL.print(init_yaw * RAD2DEG, 1);
            DEBUG_SERIAL.println(" deg");
        }
    }

    // ── Initialise EKF ────────────────────────────────────────────────────────
    ekf.init(first_fix.lat_deg, first_fix.lon_deg, first_fix.alt_msl_m,
             0.0f, 0.0f, init_yaw);

    DEBUG_SERIAL.println("EKF ready. Starting navigation loop.\n");
    DEBUG_SERIAL.println("T_ms,pN,pE,pD,vN,vE,vD,roll,pitch,yaw,sigma_pos,sigma_yaw");
}

// =============================================================================
//  Core 0 main loop
// =============================================================================
static uint32_t _last_gps_ts   = 0;
static uint32_t _last_baro_ts  = 0;
static uint32_t _last_mag_ts   = 0;
static uint32_t _telem_counter = 0;
static uint32_t _next_imu_us   = 0;
static const uint32_t IMU_PERIOD_US = 1000000UL / IMU_SAMPLE_HZ;

void loop()
{
    // ── 500 Hz rate gate ──────────────────────────────────────────────────────
    uint32_t now_us = micros();
    if (now_us < _next_imu_us) return;
    _next_imu_us = now_us + IMU_PERIOD_US;

    // ── IMU read ──────────────────────────────────────────────────────────────
    ImuData imu_data;
    if (!imu.read(imu_data)) return;

    // ── EKF Predict ───────────────────────────────────────────────────────────
    ekf.predict(imu_data.ax_ms2,  imu_data.ay_ms2,  imu_data.az_ms2,
                imu_data.gx_rads, imu_data.gy_rads, imu_data.gz_rads,
                IMU_DT_S);

    // ── EKF Update: GPS ───────────────────────────────────────────────────────
    GpsData fix;
    g_shared.gps_read(fix);
    if (fix.valid && fix.timestamp_ms != _last_gps_ts &&
        (millis() - fix.timestamp_ms) < GPS_MAX_AGE_MS)
    {
        ekf.update_gps(fix);
        _last_gps_ts = fix.timestamp_ms;
    }

    // ── EKF Update: Barometer ─────────────────────────────────────────────────
    float baro_alt, baro_pres; uint32_t baro_ts;
    if (g_shared.baro_read(baro_alt, baro_pres, baro_ts) &&
        baro_ts != _last_baro_ts)
    {
        ekf.update_baro(baro_alt);
        _last_baro_ts = baro_ts;
    }

    // ── EKF Update: Magnetometer ──────────────────────────────────────────────
    float mag_x, mag_y, mag_z; uint32_t mag_ts;
    if (g_shared.mag_read(mag_x, mag_y, mag_z, mag_ts) &&
        mag_ts != _last_mag_ts)
    {
        ekf.update_mag(mag_x, mag_y, mag_z, g_mag_ned);
        _last_mag_ts = mag_ts;
    }

    // ── Publish EKF state to shared memory ────────────────────────────────────
    float roll, pitch, yaw;
    ekf.get_euler(roll, pitch, yaw);

    SharedEkf out;
    out.pos_n = ekf.pos_n();  out.pos_e = ekf.pos_e();  out.pos_d = ekf.pos_d();
    out.vel_n = ekf.vel_n();  out.vel_e = ekf.vel_e();  out.vel_d = ekf.vel_d();
    out.q0=ekf.q0(); out.q1=ekf.q1(); out.q2=ekf.q2(); out.q3=ekf.q3();
    out.roll=roll; out.pitch=pitch; out.yaw=yaw;
    out.sigma_pos = ekf.sigma_pos_n();
    out.sigma_yaw = ekf.sigma_yaw();
    out.bias_ax = ekf.state()[IDX_BAX];
    out.bias_ay = ekf.state()[IDX_BAY];
    out.bias_az = ekf.state()[IDX_BAZ];
    out.bias_gx = ekf.state()[IDX_BGX];
    out.bias_gy = ekf.state()[IDX_BGY];
    out.bias_gz = ekf.state()[IDX_BGZ];
    out.timestamp_ms = millis();
    g_shared.ekf_write(out);

    // ── Telemetry @ 10 Hz ─────────────────────────────────────────────────────
    if (++_telem_counter >= (IMU_SAMPLE_HZ / 10)) {
        _telem_counter = 0;
        print_telemetry(roll, pitch, yaw);
    }
}

// =============================================================================
//  Telemetry (CSV, readable in Serial Plotter or Python)
// =============================================================================
void print_telemetry(float roll, float pitch, float yaw)
{
    DEBUG_SERIAL.print(millis());        DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print(ekf.pos_n(), 2);  DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print(ekf.pos_e(), 2);  DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print(ekf.pos_d(), 2);  DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print(ekf.vel_n(), 3);  DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print(ekf.vel_e(), 3);  DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print(ekf.vel_d(), 3);  DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print(roll  * RAD2DEG, 2);  DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print(pitch * RAD2DEG, 2);  DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print(yaw   * RAD2DEG, 2);  DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.print(ekf.sigma_pos_n(), 3); DEBUG_SERIAL.print(',');
    DEBUG_SERIAL.println(ekf.sigma_yaw() * RAD2DEG, 2);
}
