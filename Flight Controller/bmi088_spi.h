#pragma once
// =============================================================================
//  bmi088_spi.h  —  BMI088 6-axis IMU driver over SPI
//
//  BMI088 architecture note:
//    The BMI088 has TWO physically separate dies sharing a package:
//      • Accelerometer  (ACC)  — separate CS, separate register map
//      • Gyroscope      (GYR)  — separate CS, separate register map
//    Both are on the same SPI bus (SCK/MOSI/MISO shared).
//
//  SPI protocol:
//    Write: CS low, send (reg | 0x00), send data byte(s), CS high
//    Read:  CS low, send (reg | 0x80), [dummy byte for ACC only], recv, CS high
//           NOTE: accelerometer requires one dummy byte after address;
//                 gyroscope does NOT require a dummy byte.
//
//  Known hardware quirks addressed in this driver:
//    1. ACC chip power-on requires a dummy SPI read to switch from I2C to SPI mode
//    2. ACC soft-reset register must be written 0xB6 and needs 10 ms delay
//    3. ACC stays in suspend mode after reset; must write PWR_CTRL = 0x04
//    4. ACC normal mode entry needs 5 ms settle time
//    5. GYR soft-reset needs 30 ms before register access
//    6. SPI read on ACC requires one dummy byte between address and data
// =============================================================================

#include <SPI.h>
#include <Arduino.h>

// -----------------------------------------------------------------------------
//  BMI088 Register maps
// -----------------------------------------------------------------------------

// Accelerometer registers
namespace BMI088_ACC {
    constexpr uint8_t CHIP_ID       = 0x00;   // expected 0x1E
    constexpr uint8_t ERR_REG       = 0x02;
    constexpr uint8_t STATUS        = 0x03;
    constexpr uint8_t DATA_X_LSB    = 0x12;   // 6 bytes: ax_l, ax_h, ay_l, ay_h, az_l, az_h
    constexpr uint8_t SENSORTIME_0  = 0x18;
    constexpr uint8_t INT_STAT_1    = 0x1D;
    constexpr uint8_t TEMP_MSB      = 0x22;
    constexpr uint8_t CONF          = 0x40;   // output data rate + bandwidth
    constexpr uint8_t RANGE         = 0x41;   // ±3g / ±6g / ±12g / ±24g
    constexpr uint8_t INT1_IO_CTRL  = 0x53;
    constexpr uint8_t INT_MAP_DATA  = 0x58;
    constexpr uint8_t SELF_TEST     = 0x6D;
    constexpr uint8_t PWR_CONF      = 0x7C;   // active / suspend / low-power
    constexpr uint8_t PWR_CTRL      = 0x7D;   // accelerometer enable
    constexpr uint8_t SOFTRESET     = 0x7E;
}

// Gyroscope registers
namespace BMI088_GYR {
    constexpr uint8_t CHIP_ID       = 0x00;   // expected 0x0F
    constexpr uint8_t DATA_X_LSB    = 0x02;   // 6 bytes: gx_l, gx_h, ...
    constexpr uint8_t INT_STAT_1    = 0x0A;
    constexpr uint8_t RANGE         = 0x0F;   // ±125 / ±250 / ±500 / ±1000 / ±2000 dps
    constexpr uint8_t BANDWIDTH     = 0x10;   // ODR + filter bandwidth
    constexpr uint8_t LPM1          = 0x11;   // power mode
    constexpr uint8_t SOFTRESET     = 0x14;
    constexpr uint8_t INT_CTRL      = 0x15;
    constexpr uint8_t INT3_INT4_IO  = 0x16;
    constexpr uint8_t INT3_INT4_MAP = 0x18;
}

// Accelerometer CONF register values
constexpr uint8_t BMI088_ACC_ODR_800HZ   = 0xEB;  // OSR2  + 800 Hz
constexpr uint8_t BMI088_ACC_ODR_400HZ   = 0xEA;  // OSR2  + 400 Hz
constexpr uint8_t BMI088_ACC_RANGE_6G    = 0x01;  // ±6g
constexpr uint8_t BMI088_ACC_RANGE_12G   = 0x02;  // ±12g

// Gyroscope RANGE register values
constexpr uint8_t BMI088_GYR_RANGE_500  = 0x02;  // ±500 dps
constexpr uint8_t BMI088_GYR_RANGE_250  = 0x03;  // ±250 dps

// Gyroscope BANDWIDTH register values
constexpr uint8_t BMI088_GYR_BW_47HZ   = 0x04;  // ODR 400 Hz, filter 47 Hz
constexpr uint8_t BMI088_GYR_BW_116HZ  = 0x02;  // ODR 1000 Hz, filter 116 Hz

// -----------------------------------------------------------------------------
//  IMU raw + calibrated data packet
// -----------------------------------------------------------------------------
struct ImuData {
    // Calibrated outputs  (SI units, bias subtracted by EKF not driver)
    float ax_ms2;   // specific force X  (m/s²)
    float ay_ms2;
    float az_ms2;
    float gx_rads;  // angular rate X    (rad/s)
    float gy_rads;
    float gz_rads;
    float temp_c;   // board temperature from ACC sensor (°C)
    uint32_t timestamp_us;  // micros() at sample time
    bool valid;
};

// -----------------------------------------------------------------------------
//  BMI088 driver class
// -----------------------------------------------------------------------------
class BMI088 {
public:
    // Constructor — pass pin numbers and SPI port reference
    BMI088(SPIClass &spi, uint8_t cs_acc, uint8_t cs_gyr);

    // Initialise both ACC and GYR.  Returns true on success.
    // Call once from setup() before using read().
    bool begin();

    // Read latest sample from ACC and GYR into data struct.
    // Call at IMU_SAMPLE_HZ from Core 0 control loop.
    bool read(ImuData &out);

    // Retrieve full-scale sensitivity (for diagnostics)
    float accel_scale_ms2() const  { return _accel_lsb_to_ms2; }
    float gyro_scale_rads() const  { return _gyro_lsb_to_rads; }

private:
    SPIClass &_spi;
    uint8_t   _cs_acc;
    uint8_t   _cs_gyr;

    float     _accel_lsb_to_ms2;   // LSB → m/s²  (depends on RANGE setting)
    float     _gyro_lsb_to_rads;   // LSB → rad/s (depends on RANGE setting)

    SPISettings _spi_settings;

    // Low-level register helpers
    void    acc_write_reg(uint8_t reg, uint8_t val);
    uint8_t acc_read_reg (uint8_t reg);
    void    acc_read_burst(uint8_t reg, uint8_t *buf, uint8_t len);

    void    gyr_write_reg(uint8_t reg, uint8_t val);
    uint8_t gyr_read_reg (uint8_t reg);
    void    gyr_read_burst(uint8_t reg, uint8_t *buf, uint8_t len);

    bool    init_acc();
    bool    init_gyr();
};
