#pragma once
// =============================================================================
//  bmm350.h  —  BMM350 3-axis Magnetometer (I2C)
//
//  The BMM350 is Bosch's third-generation MEMS magnetometer.
//  Key characteristics:
//    • 16-bit (extendable to 20-bit) magnetic field data
//    • I2C interface (up to 1 MHz); SPI available on some packages
//    • Forced mode or Normal (continuous) mode
//    • On-chip OTP compensation for gain/offset
//    • I2C address: 0x14 (SDO=GND) or 0x15 (SDO=VDD)
//
//  Register architecture:
//    The BMM350 has a primary register page accessed via standard I2C.
//    OTP (One-Time Programmable) trim data is accessed via a special
//    command sequence — required for accurate compensation.
//
//  Data format:
//    Raw X/Y/Z magnetic data: 20-bit, 3 bytes each (XLSB | LSB | MSB[3:0])
//    Sign: two's-complement, bit 19 is sign bit
//    Raw sensitivity: 16384 LSB/µT (before OTP compensation)
//
//  Calibration note:
//    Hard-iron offsets (bias from nearby ferromagnetic material) must be
//    measured and subtracted.  This driver applies offsets set in config.h.
//    Soft-iron correction (full 3×3 matrix) is left to the EKF's mag update.
//
//  Usage (magnetometer heading reference):
//    1. Call begin() in setup()
//    2. Call read() at MAG_SAMPLE_HZ
//    3. Pass calibrated body-frame field to ekf.update_mag()
// =============================================================================

#include <Wire.h>
#include <Arduino.h>

// -----------------------------------------------------------------------------
//  BMM350 Register map
// -----------------------------------------------------------------------------
namespace BMM350_REG {
    constexpr uint8_t CHIP_ID           = 0x00;   // 0x33
    constexpr uint8_t REV_ID            = 0x01;
    constexpr uint8_t ERR_REG           = 0x02;
    constexpr uint8_t PAD_CTRL          = 0x03;
    constexpr uint8_t PMU_CMD_AGGR_SET  = 0x04;   // ODR + averaging
    constexpr uint8_t PMU_CMD_AXIS_EN   = 0x05;   // axis enable
    constexpr uint8_t PMU_CMD           = 0x06;   // power mode command
    constexpr uint8_t PMU_CMD_STATUS_0  = 0x07;
    constexpr uint8_t PMU_CMD_STATUS_1  = 0x08;
    constexpr uint8_t I3C_ERR           = 0x09;
    constexpr uint8_t INT_CTRL          = 0x2E;
    constexpr uint8_t INT_CTRL_IBI      = 0x2F;
    constexpr uint8_t INT_STATUS        = 0x30;

    // Data registers (20-bit per axis: XLSB=b[7:0], LSB=b[15:8], MSB=b[19:16])
    constexpr uint8_t DATA_X_XLSB       = 0x31;
    constexpr uint8_t DATA_X_LSB        = 0x32;
    constexpr uint8_t DATA_X_MSB        = 0x33;
    constexpr uint8_t DATA_Y_XLSB       = 0x34;
    constexpr uint8_t DATA_Y_LSB        = 0x35;
    constexpr uint8_t DATA_Y_MSB        = 0x36;
    constexpr uint8_t DATA_Z_XLSB       = 0x37;
    constexpr uint8_t DATA_Z_LSB        = 0x38;
    constexpr uint8_t DATA_Z_MSB        = 0x39;
    constexpr uint8_t DATA_T_XLSB       = 0x3A;   // temperature
    constexpr uint8_t DATA_T_LSB        = 0x3B;
    constexpr uint8_t DATA_T_MSB        = 0x3C;

    // OTP interface
    constexpr uint8_t OTP_CMD_REG       = 0x50;
    constexpr uint8_t OTP_DATA_MSB_REG  = 0x52;
    constexpr uint8_t OTP_DATA_LSB_REG  = 0x53;
    constexpr uint8_t OTP_STATUS_REG    = 0x55;

    constexpr uint8_t SOFTRESET         = 0x7E;
}

// PMU_CMD power mode values
constexpr uint8_t BMM350_PMU_CMD_SUSPEND    = 0x00;
constexpr uint8_t BMM350_PMU_CMD_NORMAL     = 0x01;
constexpr uint8_t BMM350_PMU_CMD_FORCED_LP  = 0x03;
constexpr uint8_t BMM350_PMU_CMD_FORCED_REG = 0x04;
constexpr uint8_t BMM350_PMU_CMD_FORCED_HP  = 0x05;

// PMU_CMD_AGGR_SET: ODR
constexpr uint8_t BMM350_ODR_400HZ  = 0x02;
constexpr uint8_t BMM350_ODR_200HZ  = 0x03;
constexpr uint8_t BMM350_ODR_100HZ  = 0x04;
constexpr uint8_t BMM350_ODR_50HZ   = 0x05;

// -----------------------------------------------------------------------------
//  MagData struct
// -----------------------------------------------------------------------------
struct MagData {
    float bx_uT;            // magnetic field X in body frame (µT)
    float by_uT;
    float bz_uT;
    float bx_norm;          // normalised (unit) body field for EKF
    float by_norm;
    float bz_norm;
    float temp_c;
    uint32_t timestamp_ms;
    bool valid;
};

// -----------------------------------------------------------------------------
//  BMM350 driver class
// -----------------------------------------------------------------------------
class BMM350 {
public:
    explicit BMM350(TwoWire &wire = Wire, uint8_t addr = 0x14);

    // Initialise. Returns true on success.
    // Must call Wire.begin(SDA, SCL) + Wire.setClock() before begin().
    bool begin();

    // Read latest sample.  Returns true if data is valid.
    bool read(MagData &out);

    // Set hard-iron offset correction (must be measured separately)
    void set_hard_iron(float ox_uT, float oy_uT, float oz_uT);

    bool is_ready() const { return _ready; }

private:
    TwoWire &_wire;
    uint8_t  _addr;
    bool     _ready;

    float    _offset_x;
    float    _offset_y;
    float    _offset_z;

    // Raw sensitivity: 16384 LSB/µT (before compensation)
    static constexpr float _lsb_to_uT = 1.0f / 16384.0f;

    // OTP compensation (populated by read_otp_trim())
    float    _mag_x_cross_axis;
    float    _mag_y_cross_axis;
    float    _temp_sensitivity;

    bool     read_otp_trim();
    int32_t  sign_extend_20bit(uint32_t raw);

    // I2C helpers
    bool     write_reg(uint8_t reg, uint8_t val);
    bool     read_reg(uint8_t reg, uint8_t &val);
    bool     read_burst(uint8_t reg, uint8_t *buf, uint8_t len);
};
