#pragma once
// =============================================================================
//  config.h  —  Hardware pins, timing constants, and EKF tuning parameters
//  Target : RP2350 (earlephilhower/arduino-pico)
//  Board  : RB-RP2354A or any RP2350 board with BMI088 + UART GPS
// =============================================================================

// -----------------------------------------------------------------------------
//  SPI — BMI088
//  BMI088 has TWO separate SPI slaves: accelerometer and gyroscope
// -----------------------------------------------------------------------------
#define IMU_SPI_PORT    SPI         // SPI0
#define IMU_SPI_FREQ    10000000UL  // 10 MHz (BMI088 max)

#define PIN_SPI_SCK     2           // SPI0 SCK
#define PIN_SPI_MISO    4           // SPI0 MISO
#define PIN_SPI_MOSI    3           // SPI0 MOSI

#define PIN_BMI088_ACC_CS   5       // Accelerometer chip-select
#define PIN_BMI088_GYR_CS   6       // Gyroscope chip-select

// -----------------------------------------------------------------------------
//  UART — GPS receiver (e.g., u-blox M8N / M9N)
// -----------------------------------------------------------------------------
#define GPS_SERIAL      Serial1     // UART1
#define GPS_BAUD        115200      // Match receiver configuration
#define PIN_GPS_TX      0           // GP0 → GPS RX
#define PIN_GPS_RX      1           // GP1 ← GPS TX

// -----------------------------------------------------------------------------
//  Debug output
// -----------------------------------------------------------------------------
#define DEBUG_SERIAL    Serial
#define DEBUG_BAUD      115200

// -----------------------------------------------------------------------------
//  Timing
// -----------------------------------------------------------------------------
#define IMU_SAMPLE_HZ       500     // IMU prediction loop rate  (Core 0)
#define IMU_DT_S            (1.0f / IMU_SAMPLE_HZ)  // seconds

#define GPS_MAX_AGE_MS      2000    // Reject GPS fix older than this

// -----------------------------------------------------------------------------
//  EKF state indices  (n = 15)
// -----------------------------------------------------------------------------
#define EKF_N           15

#define IDX_PX          0           // position North  (m)
#define IDX_PY          1           // position East   (m)
#define IDX_PZ          2           // position Down   (m)
#define IDX_VX          3           // velocity North  (m/s)
#define IDX_VY          4           // velocity East   (m/s)
#define IDX_VZ          5           // velocity Down   (m/s)
#define IDX_ROLL        6           // roll  φ  (rad)
#define IDX_PITCH       7           // pitch θ  (rad)
#define IDX_YAW         8           // yaw   ψ  (rad)
#define IDX_BAX         9           // accel bias X  (m/s²)
#define IDX_BAY         10          // accel bias Y
#define IDX_BAZ         11          // accel bias Z
#define IDX_BGX         12          // gyro  bias X  (rad/s)
#define IDX_BGY         13
#define IDX_BGZ         14

// -----------------------------------------------------------------------------
//  EKF measurement indices  (GPS provides pos + vel → p = 6)
// -----------------------------------------------------------------------------
#define EKF_P           6           // measurement dimension

// -----------------------------------------------------------------------------
//  EKF initial covariance P₀  (diagonal, one per state)
// -----------------------------------------------------------------------------
#define P0_POS          25.0f       // (5 m)²
#define P0_VEL          1.0f        // (1 m/s)²
#define P0_ATT          0.0987f     // (~18 deg)²  ≈ (0.314 rad)²
#define P0_BIAS_A       0.01f       // (0.1 m/s²)²
#define P0_BIAS_G       0.0001f     // (0.01 rad/s)²

// -----------------------------------------------------------------------------
//  EKF process noise Q  (diagonal, continuous PSD → discretised × dt)
//  Units: state_unit² / s
//
//  Tip: these are the primary tuning knobs.
//  Too small → filter overconfident, may diverge.
//  Too large  → filter noisy, slow to track.
// -----------------------------------------------------------------------------
#define Q_POS           0.0f        // position driven by velocity model
#define Q_VEL           0.1f        // unmodelled acceleration (m/s²)² /s
#define Q_ATT           0.001f      // attitude random walk  (rad)²/s
#define Q_BIAS_A        1e-6f       // accel bias instability (m/s²)²/s
#define Q_BIAS_G        1e-8f       // gyro  bias instability (rad/s)²/s

// -----------------------------------------------------------------------------
//  EKF measurement noise R  (diagonal, GPS)
// -----------------------------------------------------------------------------
#define R_GPS_POS       4.0f        // (2 m)²  — horizontal
#define R_GPS_POS_V     9.0f        // (3 m)²  — vertical (worse)
#define R_GPS_VEL_H     0.25f       // (0.5 m/s)²
#define R_GPS_VEL_V     0.25f       // (0.5 m/s)²

// -----------------------------------------------------------------------------
//  Physical constants
// -----------------------------------------------------------------------------
#define GRAVITY_MS2     9.80665f    // m/s²

// -----------------------------------------------------------------------------
//  NED origin (set on first valid GPS fix)
//  LLA → NED uses flat-earth approximation valid within ~10 km
// -----------------------------------------------------------------------------
#define EARTH_RADIUS_M  6378137.0   // WGS-84 equatorial radius (m)
#define DEG2RAD         (3.14159265358979f / 180.0f)
#define RAD2DEG         (180.0f / 3.14159265358979f)
