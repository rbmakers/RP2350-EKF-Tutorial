// =============================================================================
//  bmm350.cpp  —  BMM350 Magnetometer I2C Driver
// =============================================================================
#include "bmm350.h"
#include "config.h"
#include <math.h>

BMM350::BMM350(TwoWire &wire, uint8_t addr)
    : _wire(wire), _addr(addr), _ready(false),
      _offset_x(0.0f), _offset_y(0.0f), _offset_z(0.0f),
      _mag_x_cross_axis(0.0f), _mag_y_cross_axis(0.0f),
      _temp_sensitivity(0.0f)
{}

// ---------------------------------------------------------------------------
//  begin()
// ---------------------------------------------------------------------------
bool BMM350::begin()
{
    // Soft reset
    write_reg(BMM350_REG::SOFTRESET, 0xB6);
    delay(40);   // BMM350 startup time after reset

    // Verify chip ID
    uint8_t id = 0;
    if (!read_reg(BMM350_REG::CHIP_ID, id) || id != 0x33) {
        DEBUG_SERIAL.print("[BMM350] Bad chip ID: 0x");
        DEBUG_SERIAL.println(id, HEX);
        return false;
    }

    // Read OTP trim data (required for factory compensation)
    read_otp_trim();

    // Enable all axes
    write_reg(BMM350_REG::PMU_CMD_AXIS_EN, 0x07);   // X=bit0, Y=bit1, Z=bit2

    // Set ODR = 100 Hz
    // PMU_CMD_AGGR_SET: [4:2]=avg (0=no avg), [1:0]=ODR
    // For ODR 100 Hz: bits [3:0] = BMM350_ODR_100HZ
    write_reg(BMM350_REG::PMU_CMD_AGGR_SET, BMM350_ODR_100HZ);
    delay(2);

    // Set normal mode (continuous)
    write_reg(BMM350_REG::PMU_CMD, BMM350_PMU_CMD_NORMAL);
    delay(30);   // first measurement after mode change

    _ready = true;
    DEBUG_SERIAL.println("[BMM350] OK — normal mode, 100 Hz");
    return true;
}

// ---------------------------------------------------------------------------
//  read()
// ---------------------------------------------------------------------------
bool BMM350::read(MagData &out)
{
    if (!_ready) return false;

    // Burst read 12 bytes: X(3) + Y(3) + Z(3) + T(3) starting at DATA_X_XLSB
    uint8_t buf[12];
    if (!read_burst(BMM350_REG::DATA_X_XLSB, buf, 12)) return false;

    // Assemble 20-bit signed values
    // Format per axis: [XLSB=bits7:0, LSB=bits15:8, MSB=bits19:16 in lower nibble]
    auto assemble = [&](int base) -> int32_t {
        uint32_t raw = (uint32_t)buf[base]              // XLSB: bits 7:0
                     | ((uint32_t)buf[base+1] << 8)     // LSB:  bits 15:8
                     | ((uint32_t)(buf[base+2] & 0x0F) << 16);  // MSB: bits 19:16
        return sign_extend_20bit(raw);
    };

    int32_t raw_x = assemble(0);
    int32_t raw_y = assemble(3);
    int32_t raw_z = assemble(6);
    int32_t raw_t = assemble(9);

    // Convert to µT (raw sensitivity: 16384 LSB/µT)
    float bx = (float)raw_x * _lsb_to_uT;
    float by = (float)raw_y * _lsb_to_uT;
    float bz = (float)raw_z * _lsb_to_uT;

    // Apply hard-iron offset correction
    bx -= _offset_x;
    by -= _offset_y;
    bz -= _offset_z;

    out.bx_uT = bx;
    out.by_uT = by;
    out.bz_uT = bz;

    // Temperature: 20-bit, LSB = 1/16°C → sensitivity = 0.0625
    out.temp_c = (float)raw_t * 0.0625f;

    // Normalise for EKF (EKF uses unit-length field vectors)
    float mag = sqrtf(bx*bx + by*by + bz*bz);
    if (mag > 1.0f) {   // sanity: Earth field ≥ ~25 µT
        out.bx_norm = bx / mag;
        out.by_norm = by / mag;
        out.bz_norm = bz / mag;
        out.valid = true;
    } else {
        out.bx_norm = 0;
        out.by_norm = 0;
        out.bz_norm = 0;
        out.valid = false;
    }

    out.timestamp_ms = millis();
    return out.valid;
}

// ---------------------------------------------------------------------------
//  set_hard_iron()
// ---------------------------------------------------------------------------
void BMM350::set_hard_iron(float ox, float oy, float oz)
{
    _offset_x = ox;
    _offset_y = oy;
    _offset_z = oz;
}

// ---------------------------------------------------------------------------
//  PRIVATE: read_otp_trim()
//
//  The BMM350 stores factory compensation data in OTP rows.
//  Reading requires issuing an OTP read command and polling for completion.
//  Reference: BMM350 datasheet §5.4 OTP Read Procedure
//
//  For this tutorial, we read the available trim and apply partial
//  compensation.  The Bosch open-source driver (bmm350-sensor-api on GitHub)
//  contains the complete 14-row compensation algorithm.
// ---------------------------------------------------------------------------
bool BMM350::read_otp_trim()
{
    // Issue OTP read command for row 0 (sensitivity registers)
    // OTP_CMD_REG[7:1] = row number, [0] = 1 to trigger read
    for (uint8_t row = 0; row < 2; row++) {
        write_reg(BMM350_REG::OTP_CMD_REG, (row << 1) | 0x01);
        delay(5);  // OTP read takes ~1 ms

        uint8_t status;
        if (!read_reg(BMM350_REG::OTP_STATUS_REG, status)) continue;
        if (!(status & 0x01)) continue;  // not ready

        uint8_t msb, lsb;
        read_reg(BMM350_REG::OTP_DATA_MSB_REG, msb);
        read_reg(BMM350_REG::OTP_DATA_LSB_REG, lsb);

        uint16_t otp_word = ((uint16_t)msb << 8) | lsb;

        if (row == 0) {
            // Row 0: mag_x_cross_axis [7:0], mag_y_cross_axis [15:8]
            _mag_x_cross_axis = (int8_t)(otp_word & 0xFF)        * 0.00003052f; // 1/32768
            _mag_y_cross_axis = (int8_t)((otp_word >> 8) & 0xFF) * 0.00003052f;
        }
        if (row == 1) {
            // Row 1: temperature sensitivity
            _temp_sensitivity = (int8_t)(otp_word & 0xFF) * 0.0078125f; // ppm/°C
        }
    }

    DEBUG_SERIAL.println("[BMM350] OTP trim read.");
    return true;
}

// ---------------------------------------------------------------------------
//  PRIVATE: sign_extend_20bit()
// ---------------------------------------------------------------------------
int32_t BMM350::sign_extend_20bit(uint32_t raw)
{
    if (raw & 0x80000U) {           // bit 19 is sign bit
        return (int32_t)(raw | 0xFFF00000U);
    }
    return (int32_t)raw;
}

// ---------------------------------------------------------------------------
//  PRIVATE: I2C helpers
// ---------------------------------------------------------------------------
bool BMM350::write_reg(uint8_t reg, uint8_t val)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.write(val);
    return _wire.endTransmission() == 0;
}

bool BMM350::read_reg(uint8_t reg, uint8_t &val)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return false;
    _wire.requestFrom(_addr, (uint8_t)1);
    if (_wire.available()) { val = _wire.read(); return true; }
    return false;
}

bool BMM350::read_burst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return false;
    uint8_t got = _wire.requestFrom(_addr, len);
    if (got != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = _wire.read();
    return true;
}
