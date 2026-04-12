#pragma once
// =============================================================================
//  config.h  —  FCS_NAV: Flight Control System + Navigation
//  Hardware : RP2354A (RP2350 core + 8 MB on-chip PSRAM, QFN-60)
//  Sensors  : BMI088 (SPI) · BMM350 (I2C) · BMP580 (I2C) · GNSS UART
//  Receiver : ExpressLRS / CRSF (UART)
//  Actuators: 4× DSHOT600 brushless ESC via RP2350 PIO
// =============================================================================

// -----------------------------------------------------------------------------
//  RP2354A note
//  The RP2354A is the RP2350 die in QFN-60 with 8 MB PSRAM attached internally.
//  For this project the PSRAM is not used — all data fits in the 520 KB SRAM.
//  To enable PSRAM in arduino-pico: add `rp2040.psram=8` to board flags.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//  SPI — BMI088 (IMU)
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
// -----------------------------------------------------------------------------
#define IMU_I2C_PORT        Wire
#define IMU_I2C_FREQ        400000UL
#define PIN_I2C_SDA         20
#define PIN_I2C_SCL         21
#define BMM350_I2C_ADDR     0x14
#define BMP580_I2C_ADDR     0x46

// -----------------------------------------------------------------------------
//  UART — GPS (UART0 via Serial1)
// -----------------------------------------------------------------------------
#define GPS_SERIAL          Serial1
#define GPS_BAUD            115200
#define PIN_GPS_TX          0
#define PIN_GPS_RX          1

// -----------------------------------------------------------------------------
//  UART — CRSF / ExpressLRS receiver (UART1 via Serial2)
// -----------------------------------------------------------------------------
#define CRSF_SERIAL         Serial2
#define CRSF_BAUD           420000
#define PIN_CRSF_TX         8
#define PIN_CRSF_RX         9

// -----------------------------------------------------------------------------
//  PIO — DSHOT600 motor outputs (PIO0, state machines 0-3)
// -----------------------------------------------------------------------------
#define DSHOT_PIO           pio0
#define PIN_MOTOR_1         10      // Front-Right (CW)
#define PIN_MOTOR_2         11      // Front-Left  (CCW)
#define PIN_MOTOR_3         12      // Rear-Left   (CW)
#define PIN_MOTOR_4         13      // Rear-Right  (CCW)

// DSHOT600 PIO clock: 150 MHz / 15 + 10/16 fractional ≈ 9.6154 MHz
// Bit period = 16 PIO cycles ≈ 1664 ns → ~601 kbit/s (within spec)
// '1' bit: 12 cycles high + 4 cycles low
// '0' bit:  6 cycles high + 10 cycles low
#define DSHOT_PIO_CLK_INT   15      // integer divider
#define DSHOT_PIO_CLK_FRAC  160     // fractional (/256): 10/16 = 160/256

#define DSHOT_MIN_THROTTLE  48      // min armed throttle (DSHOT value)
#define DSHOT_MAX_THROTTLE  2047    // max throttle
#define DSHOT_DISARM_VAL    0       // 0 = disarm/stop command

// -----------------------------------------------------------------------------
//  Status LED
// -----------------------------------------------------------------------------
#define PIN_LED             22

// -----------------------------------------------------------------------------
//  Debug
// -----------------------------------------------------------------------------
#define DEBUG_SERIAL        Serial
#define DEBUG_BAUD          115200

// -----------------------------------------------------------------------------
//  Control loop timing
// -----------------------------------------------------------------------------
#define CTRL_LOOP_HZ        2000
#define CTRL_DT_S           (1.0f / CTRL_LOOP_HZ)      // 500 µs
#define CTRL_PERIOD_US      (1000000UL / CTRL_LOOP_HZ)  // 500 µs

// Sub-rates (expressed as tick dividers from 2 kHz base)
#define DIV_EKF_PREDICT     4       // 500 Hz — EKF predict + angle PID
#define DIV_MAG_UPDATE      20      // 100 Hz — magnetometer EKF update
#define DIV_BARO_UPDATE     40      //  50 Hz — barometer EKF update
#define DIV_POS_LOOP        200     //  10 Hz — position/velocity PID + telemetry

#define EKF_DT_S            (CTRL_DT_S * DIV_EKF_PREDICT)  // 0.002 s

#define GPS_MAX_AGE_MS      2000

// Sensor output data rates (for Core 1 scheduling)
#define BARO_SAMPLE_HZ      50
#define MAG_SAMPLE_HZ       100

// -----------------------------------------------------------------------------
//  EKF State vector (16-state quaternion INS)
// -----------------------------------------------------------------------------
#define EKF_N           16

#define IDX_PX          0
#define IDX_PY          1
#define IDX_PZ          2
#define IDX_VX          3
#define IDX_VY          4
#define IDX_VZ          5
#define IDX_Q0          6
#define IDX_Q1          7
#define IDX_Q2          8
#define IDX_Q3          9
#define IDX_BAX         10
#define IDX_BAY         11
#define IDX_BAZ         12
#define IDX_BGX         13
#define IDX_BGY         14
#define IDX_BGZ         15

#define EKF_P_GPS       6
#define EKF_P_BARO      1
#define EKF_P_MAG       3
#define EKF_P_MAX       6

// EKF initial covariance
#define P0_POS          25.0f
#define P0_VEL          1.0f
#define P0_QUAT         0.0308f
#define P0_BIAS_A       0.01f
#define P0_BIAS_G       1e-4f

// EKF process noise
#define Q_POS           0.0f
#define Q_VEL           0.1f
#define Q_QUAT          5e-4f
#define Q_BIAS_A        1e-6f
#define Q_BIAS_G        1e-8f

// EKF measurement noise
#define R_GPS_POS_H     4.0f
#define R_GPS_POS_V     9.0f
#define R_GPS_VEL_H     0.25f
#define R_GPS_VEL_V     0.25f
#define R_BARO_ALT      0.25f
#define R_MAG           0.04f

// Hard-iron calibration
#define MAG_HARDIRON_X  0.0f
#define MAG_HARDIRON_Y  0.0f
#define MAG_HARDIRON_Z  0.0f

// NED magnetic reference field (unit vector, Taiwan ~25°N as default)
// Update for your location: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
#define MAG_REF_N       0.827f
#define MAG_REF_E      -0.065f
#define MAG_REF_D       0.559f

// -----------------------------------------------------------------------------
//  RC / CRSF channel mapping (0-indexed)
// -----------------------------------------------------------------------------
#define RC_CH_ROLL      0       // aileron
#define RC_CH_PITCH     1       // elevator
#define RC_CH_THROTTLE  2       // throttle
#define RC_CH_YAW       3       // rudder
#define RC_CH_ARM       4       // AUX1 — arm switch
#define RC_CH_MODE      5       // AUX2 — flight mode

// CRSF raw range: 172–1811  (center 992 ≈ 1500µs)
#define CRSF_MIN        172
#define CRSF_MAX        1811
#define CRSF_MID        992
#define CRSF_DEADBAND   20      // raw counts (~10µs)
#define CRSF_ARM_THRESH 1700    // raw value above = armed
#define RC_FAILSAFE_MS  500     // declare failsafe if no packet for this long

// -----------------------------------------------------------------------------
//  PID tuning — Rate loop (2 kHz, raw gyro)
// -----------------------------------------------------------------------------
#define PID_RATE_ROLL_P     0.135f
#define PID_RATE_ROLL_I     0.300f
#define PID_RATE_ROLL_D     0.0025f
#define PID_RATE_ROLL_FF    0.0f        // feedforward gain

#define PID_RATE_PITCH_P    0.135f
#define PID_RATE_PITCH_I    0.300f
#define PID_RATE_PITCH_D    0.0025f
#define PID_RATE_PITCH_FF   0.0f

#define PID_RATE_YAW_P      0.200f
#define PID_RATE_YAW_I      0.100f
#define PID_RATE_YAW_D      0.0f
#define PID_RATE_YAW_FF     0.0f

// Integral anti-windup clamp (normalised output units)
#define PID_RATE_IMAX       0.3f

// Betaflight-class features
#define ITERM_RELAX_CUTOFF  20.0f   // Hz — I-term relax frequency
#define TPA_BREAKPOINT      0.5f    // normalised throttle where TPA starts
#define TPA_RATE            0.4f    // D-term reduction per unit above breakpoint

// D-term filter (simple first-order low-pass, cutoff Hz)
#define DTERM_LPF_HZ        90.0f

// Gyro filter (notch centre / bandwidth Hz — optional)
#define GYRO_LPF_HZ         120.0f

// Max rate setpoints (deg/s)
#define MAX_RATE_ROLL_DPS   360.0f
#define MAX_RATE_PITCH_DPS  360.0f
#define MAX_RATE_YAW_DPS    180.0f

// -----------------------------------------------------------------------------
//  PID tuning — Angle loop (500 Hz, EKF attitude)
// -----------------------------------------------------------------------------
#define PID_ANGLE_ROLL_P    4.5f    // output: roll rate setpoint (deg/s per deg error)
#define PID_ANGLE_PITCH_P   4.5f
#define MAX_ANGLE_ROLL_DEG  45.0f   // max commanded lean angle
#define MAX_ANGLE_PITCH_DEG 45.0f

// -----------------------------------------------------------------------------
//  PID tuning — Altitude hold (50 Hz, EKF pos+vel)
// -----------------------------------------------------------------------------
#define PID_ALT_P           0.8f    // pos error → vertical velocity setpoint
#define PID_ALT_I           0.1f
#define PID_ALT_D           0.3f    // vertical velocity error (vel P)
#define MAX_VEL_UP_MS       3.0f    // m/s
#define MAX_VEL_DOWN_MS    -2.0f
#define HOVER_THROTTLE      0.50f   // normalised hover throttle estimate (tune this!)

// -----------------------------------------------------------------------------
//  PID tuning — Position hold / GPS velocity (10 Hz, EKF NED)
// -----------------------------------------------------------------------------
#define PID_VEL_N_P         2.5f    // velocity error → angle setpoint (deg per m/s)
#define PID_VEL_E_P         2.5f
#define PID_VEL_N_I         0.5f
#define PID_VEL_E_I         0.5f
#define MAX_VEL_HORZ_MS     5.0f    // m/s horizontal velocity limit
#define MAX_POS_TILT_DEG    20.0f   // max angle allowed by position PID

// -----------------------------------------------------------------------------
//  Arming safety
// -----------------------------------------------------------------------------
#define ARM_THROTTLE_MAX    200     // DSHOT — throttle must be below this to arm
#define MOTOR_SPIN_TEST_VAL 100     // DSHOT — brief spin at arming for check
#define DISARM_TIMEOUT_S    10.0f   // auto-disarm if throttle low for this long

// -----------------------------------------------------------------------------
//  Physical / geodetic constants
// -----------------------------------------------------------------------------
#define GRAVITY_MS2         9.80665f
#define EARTH_RADIUS_M      6378137.0
#define DEG2RAD             (3.14159265358979f / 180.0f)
#define RAD2DEG             (180.0f / 3.14159265358979f)
