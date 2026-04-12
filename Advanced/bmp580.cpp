// =============================================================================
//  bmp580.cpp
// =============================================================================
#include "bmp580.h"
#include "config.h"
#include <math.h>

BMP580::BMP580(TwoWire &wire, uint8_t addr)
    : _wire(wire), _addr(addr), _ready(false),
      _p0_Pa(101325.0f)
{}

// ---------------------------------------------------------------------------
//  begin()
// ---------------------------------------------------------------------------
bool BMP580::begin()
{
    // Soft reset
    write_reg(BMP580_REG::CMD, 0xB6);
    delay(5);

    uint8_t id = 0;
    if (!read_reg(BMP580_REG::CHIP_ID, id)) {
        DEBUG_SERIAL.println("[BMP580] I2C error — no response");
        return false;
    }
    // Accept both BMP580 (0x50) and BMP585 (0x51)
    if (id != 0x50 && id != 0x51) {
        DEBUG_SERIAL.print("[BMP580] Bad chip ID: 0x");
        DEBUG_SERIAL.println(id, HEX);
        return false;
    }

    // ── Configure oversampling ────────────────────────────────────────────────
    // OSR_CONFIG: mode=normal(0x03), osr_p=8x(0x03<<2), osr_t=1x(0x00<<5)
    // = 0b00_011_11 = 0x0F... let me compute:
    // [1:0]=0x03 (normal), [4:2]=0x03 (osr_p 8x), [7:5]=0x00 (osr_t 1x)
    uint8_t osr_cfg = BMP580_MODE_NORMAL
                    | (BMP580_OSR_8X  << 2)   // pressure oversampling 8×
                    | (BMP580_OSR_1X  << 5);  // temperature oversampling 1×

    // Note: BMP580 OSR_CONFIG[1:0] is the power mode on some versions.
    // On others, mode is in a separate field.  The safe sequence is:
    // 1. Set sleep mode
    // 2. Configure OSR
    // 3. Set normal mode
    write_reg(BMP580_REG::OSR_CONFIG, BMP580_MODE_SLEEP);
    delay(2);

    // Pressure and temperature OSR (keeping sleep mode bits)
    write_reg(BMP580_REG::OSR_CONFIG,
              (BMP580_OSR_8X << 2) | (BMP580_OSR_1X << 5));
    delay(1);

    // ── Configure ODR: 50 Hz ─────────────────────────────────────────────────
    // ODR_CONFIG[4:0] = BMP580_ODR_50HZ(0x03), [6]=0 (deep standby disabled)
    write_reg(BMP580_REG::ODR_CONFIG, BMP580_ODR_50HZ);

    // ── Enable IIR filter for pressure: coefficient = 4 ─────────────────────
    // DSP_IIR: [2:0]=iir_flush_p (0=bypass,1=2,2=4,...),  [5:3]=iir_flush_t
    // Coefficient 4 → code 2
    write_reg(BMP580_REG::DSP_IIR, 0x02);   // IIR coeff = 4 for pressure

    // ── Enter normal mode ─────────────────────────────────────────────────────
    // Read current OSR_CONFIG, set mode bits to normal
    uint8_t osr_val = 0;
    read_reg(BMP580_REG::OSR_CONFIG, osr_val);
    osr_val = (osr_val & ~0x03) | BMP580_MODE_NORMAL;
    write_reg(BMP580_REG::OSR_CONFIG, osr_val);
    delay(10);   // wait for first measurement

    _ready = true;
    DEBUG_SERIAL.print("[BMP580] Chip ID=0x");
    DEBUG_SERIAL.print(id, HEX);
    DEBUG_SERIAL.println(" — normal mode, 50 Hz, IIR-4, OSR 8×");
    return true;
}

// ---------------------------------------------------------------------------
//  read()
// ---------------------------------------------------------------------------
bool BMP580::read(BaroData &out)
{
    if (!_ready) return false;

    // Check data ready
    uint8_t status = 0;
    read_reg(BMP580_REG::STATUS, status);
    if (!(status & 0x08)) {   // bit 3 = drdy_press
        out.valid = false;
        return false;
    }

    // Read 6 bytes: temperature (3) + pressure (3) starting at TEMP_DATA_0
    uint8_t buf[6];
    if (!read_burst(BMP580_REG::TEMP_DATA_0, buf, 6)) {
        out.valid = false;
        return false;
    }

    // Assemble 24-bit raw values (unsigned, then signed interpretation)
    // Temperature: 24-bit signed, units = °C / 2^16
    int32_t raw_t = (int32_t)buf[0]
                  | ((int32_t)buf[1] << 8)
                  | ((int32_t)(int8_t)buf[2] << 16);  // sign-extend MSB

    // Pressure: 24-bit unsigned, units = Pa × 2^6  (i.e., 1/64 Pa resolution)
    uint32_t raw_p = (uint32_t)buf[3]
                   | ((uint32_t)buf[4] << 8)
                   | ((uint32_t)buf[5] << 16);

    out.temperature_c = (float)raw_t / 65536.0f;
    out.pressure_Pa   = (float)raw_p / 64.0f;
    out.altitude_m    = compute_altitude(out.pressure_Pa);
    out.timestamp_ms  = millis();
    out.valid         = (out.pressure_Pa > 50000.0f && out.pressure_Pa < 120000.0f);

    return out.valid;
}

// ---------------------------------------------------------------------------
//  set_sea_level_pressure()
// ---------------------------------------------------------------------------
void BMP580::set_sea_level_pressure(float p0_Pa)
{
    _p0_Pa = p0_Pa;
    DEBUG_SERIAL.print("[BMP580] Sea-level ref set: ");
    DEBUG_SERIAL.print(p0_Pa, 2);
    DEBUG_SERIAL.println(" Pa");
}

// ---------------------------------------------------------------------------
//  PRIVATE: compute_altitude()
//
//  International barometric formula:
//    alt = (T₀/L) × [1 − (P/P₀)^(R·L/g)]
//  Simplified:
//    alt = 44330.0 × [1 − (P/P₀)^0.190295]
//
//  Validity: <11 km altitude (troposphere).
//  At 5000 m, error < 1 m vs full ICAO model.
// ---------------------------------------------------------------------------
float BMP580::compute_altitude(float P_Pa) const
{
    if (_p0_Pa < 1.0f) return 0.0f;   // guard
    return 44330.0f * (1.0f - powf(P_Pa / _p0_Pa, 0.190295f));
}

// ---------------------------------------------------------------------------
//  PRIVATE: I2C helpers
// ---------------------------------------------------------------------------
bool BMP580::write_reg(uint8_t reg, uint8_t val)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.write(val);
    return _wire.endTransmission() == 0;
}

bool BMP580::read_reg(uint8_t reg, uint8_t &val)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return false;
    _wire.requestFrom(_addr, (uint8_t)1);
    if (_wire.available()) { val = _wire.read(); return true; }
    return false;
}

bool BMP580::read_burst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return false;
    uint8_t got = _wire.requestFrom(_addr, len);
    if (got != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = _wire.read();
    return true;
}
