#pragma once
// =============================================================================
//  bmp580.h  —  BMP580 Barometric Pressure Sensor (I2C)
//
//  The BMP580 is Bosch's high-precision MEMS pressure sensor.
//  Key characteristics:
//    • 20-bit pressure data, 16-bit temperature data
//    • Pressure range: 30–125 kPa, resolution: 1/64 Pa
//    • Temperature range: −40 to +85 °C
//    • I2C (400 kHz) or SPI (up to 10 MHz)
//    • Internal IIR filter, configurable oversampling
//    • No external compensation needed (all coefficients on-chip)
//
//  Output data format:
//    Pressure register: 24-bit (3 bytes), value in Pa × 2⁶ (i.e., Pa/64 steps)
//      → pressure_Pa = raw / 64.0
//    Temperature register: 24-bit (3 bytes), value in °C × 2¹⁶
//      → temperature_C = raw / 65536.0
//
//  Altitude computation:
//    Uses the international barometric formula:
//      alt = T₀/L × [1 − (P/P₀)^(R·L/g)]
//    where T₀=288.15 K, L=0.0065 K/m, R=287.05 J/(kg·K), g=9.80665 m/s²
//    Simplified: alt = 44330 × [1 − (P/P₀)^(1/5.255)]
//
//  Sea-level reference P₀:
//    Must be set at ground level before flight.
//    Call set_sea_level_pressure(P_ground_Pa) during startup.
//
//  I2C address: 0x46 (SDO=GND) or 0x47 (SDO=VDD)
// =============================================================================

#include <Wire.h>
#include <Arduino.h>

// -----------------------------------------------------------------------------
//  BMP580 Register map (key registers)
// -----------------------------------------------------------------------------
namespace BMP580_REG {
    constexpr uint8_t CHIP_ID       = 0x01;   // 0x50 (BMP580), 0x51 (BMP585)
    constexpr uint8_t REV_ID        = 0x02;
    constexpr uint8_t CHIP_STATUS   = 0x11;
    constexpr uint8_t DRIVE_CONFIG  = 0x13;
    constexpr uint8_t INT_CONFIG    = 0x14;
    constexpr uint8_t INT_SOURCE    = 0x15;
    constexpr uint8_t FIFO_CONFIG   = 0x16;
    constexpr uint8_t FIFO_COUNT    = 0x17;
    constexpr uint8_t FIFO_SEL      = 0x18;
    constexpr uint8_t TEMP_DATA_0   = 0x1D;   // Temperature [7:0]
    constexpr uint8_t TEMP_DATA_1   = 0x1E;   // Temperature [15:8]
    constexpr uint8_t TEMP_DATA_2   = 0x1F;   // Temperature [23:16]
    constexpr uint8_t PRESS_DATA_0  = 0x20;   // Pressure [7:0]
    constexpr uint8_t PRESS_DATA_1  = 0x21;   // Pressure [15:8]
    constexpr uint8_t PRESS_DATA_2  = 0x22;   // Pressure [23:16]
    constexpr uint8_t INT_STATUS    = 0x27;
    constexpr uint8_t STATUS        = 0x28;   // drdy_temp[4], drdy_press[3]
    constexpr uint8_t FIFO_DATA     = 0x29;
    constexpr uint8_t NVM_ADDR      = 0x2B;
    constexpr uint8_t NVM_DATA_LSB  = 0x2C;
    constexpr uint8_t NVM_DATA_MSB  = 0x2D;
    constexpr uint8_t DSP_CONFIG    = 0x30;
    constexpr uint8_t DSP_IIR       = 0x31;
    constexpr uint8_t OOR_THR_P_LSB = 0x32;
    constexpr uint8_t OOR_THR_P_MSB = 0x33;
    constexpr uint8_t OOR_RANGE     = 0x34;
    constexpr uint8_t OOR_CONFIG    = 0x35;
    constexpr uint8_t OSR_CONFIG    = 0x36;   // [1:0]=mode, [4:2]=osr_p, [7:5]=osr_t
    constexpr uint8_t ODR_CONFIG    = 0x37;   // [4:0]=ODR code, [6]=deep_dis
    constexpr uint8_t OSP_CONFIG    = 0x38;
    constexpr uint8_t CMD           = 0x7E;   // 0xB6 = soft reset
}

// OSR_CONFIG power mode bits [1:0]
constexpr uint8_t BMP580_MODE_SLEEP   = 0x00;
constexpr uint8_t BMP580_MODE_FORCED  = 0x01;
constexpr uint8_t BMP580_MODE_NORMAL  = 0x03;

// Oversampling ratios for osr_p / osr_t  (bits [4:2] and [7:5])
constexpr uint8_t BMP580_OSR_1X   = 0x00;
constexpr uint8_t BMP580_OSR_2X   = 0x01;
constexpr uint8_t BMP580_OSR_4X   = 0x02;
constexpr uint8_t BMP580_OSR_8X   = 0x03;
constexpr uint8_t BMP580_OSR_16X  = 0x04;
constexpr uint8_t BMP580_OSR_32X  = 0x05;
constexpr uint8_t BMP580_OSR_64X  = 0x06;
constexpr uint8_t BMP580_OSR_128X = 0x07;

// ODR codes (ODR_CONFIG[4:0])
constexpr uint8_t BMP580_ODR_240HZ  = 0x00;
constexpr uint8_t BMP580_ODR_120HZ  = 0x01;
constexpr uint8_t BMP580_ODR_60HZ   = 0x02;
constexpr uint8_t BMP580_ODR_50HZ   = 0x03;
constexpr uint8_t BMP580_ODR_45HZ   = 0x04;
constexpr uint8_t BMP580_ODR_20HZ   = 0x06;

// -----------------------------------------------------------------------------
//  BaroData struct
// -----------------------------------------------------------------------------
struct BaroData {
    float   pressure_Pa;        // compensated pressure (Pa)
    float   temperature_c;      // temperature (°C)
    float   altitude_m;         // altitude above sea-level reference (m)
    uint32_t timestamp_ms;
    bool    valid;
};

// -----------------------------------------------------------------------------
//  BMP580 driver class
// -----------------------------------------------------------------------------
class BMP580 {
public:
    explicit BMP580(TwoWire &wire = Wire, uint8_t addr = 0x46);

    // Initialise. Returns true on success.
    bool begin();

    // Read latest pressure + temperature → compute altitude.
    bool read(BaroData &out);

    // Set sea-level reference pressure for altitude calculation.
    // Call once after receiving a GPS fix altitude to align baro and GPS.
    void set_sea_level_pressure(float p0_Pa);

    float sea_level_pressure() const { return _p0_Pa; }

    bool is_ready() const { return _ready; }

private:
    TwoWire &_wire;
    uint8_t  _addr;
    bool     _ready;
    float    _p0_Pa;      // sea-level reference pressure (Pa)

    float compute_altitude(float P_Pa) const;

    bool write_reg(uint8_t reg, uint8_t val);
    bool read_reg(uint8_t reg, uint8_t &val);
    bool read_burst(uint8_t reg, uint8_t *buf, uint8_t len);
};
