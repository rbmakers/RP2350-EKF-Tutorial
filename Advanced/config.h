#pragma once
// =============================================================================
//  config.h  —  Advanced GNSS+INS Navigation System
//  Target  : RP2350 (earlephilhower/arduino-pico)
//  Sensors : BMI088 (SPI), BMM350 (I2C), BMP580 (I2C), GNSS UART
//  EKF     : 16-state quaternion, Joseph-form covariance
// =============================================================================

// -----------------------------------------------------------------------------
//  SPI — BMI088 (accelerometer + gyroscope)
// -----------------------------------------------------------------------------
#define IMU_SPI_PORT        SPI
#define IMU_SPI_FREQ        10000000UL

#define PIN_SPI_SCK         2
#define PIN_SPI_MISO        4
#define PIN_SPI_MOSI        3
#define PIN_BMI088_ACC_CS   5
#define PIN_BMI088_GYR_CS   6

// -----------------------------------------------------------------------------
//  I2C — BMM350 (magnetometer) + BMP580 (barometer)
//  Both share Wire (I2C0) on RP2350
// -----------------------------------------------------------------------------
#define IMU_I2C_PORT        Wire
#define IMU_I2C_FREQ        400000UL    // 400 kHz fast mode

#define PIN_I2C_SDA         20          // GP20
#define PIN_I2C_SCL         21          // GP21

#define BMM350_I2C_ADDR     0x14        // SDO → GND
#define BMP580_I2C_ADDR     0x46        // SDO → GND  (0x47 if SDO → VDD)

// -----------------------------------------------------------------------------
//  UART — GPS
// -----------------------------------------------------------------------------
#define GPS_SERIAL          Serial1
#define GPS_BAUD            115200
#define PIN_GPS_TX          0
#define PIN_GPS_RX          1

// -----------------------------------------------------------------------------
//  Debug UART
// -----------------------------------------------------------------------------
#define DEBUG_SERIAL        Serial
#define DEBUG_BAUD          115200

// -----------------------------------------------------------------------------
//  Timing
// -----------------------------------------------------------------------------
#define IMU_SAMPLE_HZ       500
#define IMU_DT_S            (1.0f / IMU_SAMPLE_HZ)

#define BARO_SAMPLE_HZ      50          // BMP580 update rate
#define MAG_SAMPLE_HZ       100         // BMM350 update rate

#define GPS_MAX_AGE_MS      2000

// -----------------------------------------------------------------------------
//  EKF State vector — 16 states (quaternion replaces Euler)
//
//   x = [ px, py, pz,           positions NED    (m)
//          vx, vy, vz,           velocities NED   (m/s)
//          q0, q1, q2, q3,       quaternion        (Hamilton, scalar first)
//          bax, bay, baz,        accel bias        (m/s²)
//          bgx, bgy, bgz ]       gyro  bias        (rad/s)
// -----------------------------------------------------------------------------
#define EKF_N           16

#define IDX_PX          0
#define IDX_PY          1
#define IDX_PZ          2
#define IDX_VX          3
#define IDX_VY          4
#define IDX_VZ          5
#define IDX_Q0          6       // quaternion scalar  w
#define IDX_Q1          7       // quaternion vector  x
#define IDX_Q2          8       // quaternion vector  y
#define IDX_Q3          9       // quaternion vector  z
#define IDX_BAX         10
#define IDX_BAY         11
#define IDX_BAZ         12
#define IDX_BGX         13
#define IDX_BGY         14
#define IDX_BGZ         15

// Measurement dimensions
#define EKF_P_GPS       6       // GPS: [pN,pE,pD,vN,vE,vD]
#define EKF_P_BARO      1       // Baro: [altitude deviation]
#define EKF_P_MAG       3       // Mag: [mx,my,mz] body frame
#define EKF_P_MAX       6       // maximum p for pre-allocation

// -----------------------------------------------------------------------------
//  Initial covariance P₀
// -----------------------------------------------------------------------------
#define P0_POS          25.0f           // (5 m)²
#define P0_VEL          1.0f            // (1 m/s)²
#define P0_QUAT         0.0308f         // (10°→~0.175 rad in half-angle)²
#define P0_BIAS_A       0.01f
#define P0_BIAS_G       1e-4f

// -----------------------------------------------------------------------------
//  Process noise Q (continuous PSD, discretised × dt)
// -----------------------------------------------------------------------------
#define Q_POS           0.0f
#define Q_VEL           0.1f            // unmodelled accel  (m/s²)²/s
#define Q_QUAT          5e-4f           // attitude random walk (rad²/s per component)
#define Q_BIAS_A        1e-6f
#define Q_BIAS_G        1e-8f

// -----------------------------------------------------------------------------
//  Measurement noise R
// -----------------------------------------------------------------------------
// GPS
#define R_GPS_POS_H     4.0f            // (2 m)²    horizontal
#define R_GPS_POS_V     9.0f            // (3 m)²    vertical
#define R_GPS_VEL_H     0.25f           // (0.5 m/s)²
#define R_GPS_VEL_V     0.25f

// Barometer
#define R_BARO_ALT      0.25f           // (0.5 m)²

// Magnetometer (normalised units)
#define R_MAG           0.04f           // per-axis (normalised field, ~dimensionless)

// -----------------------------------------------------------------------------
//  Magnetometer calibration (hard-iron offsets, loaded at startup)
// -----------------------------------------------------------------------------
#define MAG_HARDIRON_X  0.0f
#define MAG_HARDIRON_Y  0.0f
#define MAG_HARDIRON_Z  0.0f

// -----------------------------------------------------------------------------
//  Physical / geodetic constants
// -----------------------------------------------------------------------------
#define GRAVITY_MS2     9.80665f
#define EARTH_RADIUS_M  6378137.0
#define DEG2RAD         (3.14159265358979f / 180.0f)
#define RAD2DEG         (180.0f / 3.14159265358979f)
#define M_2PI           (2.0f * 3.14159265358979f)
