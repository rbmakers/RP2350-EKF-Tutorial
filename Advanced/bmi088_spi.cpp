// =============================================================================
//  bmi088_spi.cpp
// =============================================================================
#include "bmi088_spi.h"
#include "config.h"
#include <math.h>

// ---------------------------------------------------------------------------
//  Constructor
// ---------------------------------------------------------------------------
BMI088::BMI088(SPIClass &spi, uint8_t cs_acc, uint8_t cs_gyr)
    : _spi(spi), _cs_acc(cs_acc), _cs_gyr(cs_gyr),
      _accel_lsb_to_ms2(0.0f), _gyro_lsb_to_rads(0.0f),
      _spi_settings(IMU_SPI_FREQ, MSBFIRST, SPI_MODE0)
{}

// ---------------------------------------------------------------------------
//  Public: begin()
// ---------------------------------------------------------------------------
bool BMI088::begin()
{
    // Configure chip-select pins
    pinMode(_cs_acc, OUTPUT);
    pinMode(_cs_gyr, OUTPUT);
    digitalWrite(_cs_acc, HIGH);
    digitalWrite(_cs_gyr, HIGH);

    _spi.begin();

    delay(10);  // power-on settling

    if (!init_acc()) {
        DEBUG_SERIAL.println("[BMI088] ERROR: Accelerometer init failed");
        return false;
    }
    if (!init_gyr()) {
        DEBUG_SERIAL.println("[BMI088] ERROR: Gyroscope init failed");
        return false;
    }
    DEBUG_SERIAL.println("[BMI088] OK — both sensors initialised");
    return true;
}

// ---------------------------------------------------------------------------
//  Public: read()
// ---------------------------------------------------------------------------
bool BMI088::read(ImuData &out)
{
    out.timestamp_us = micros();

    // --- Accelerometer ---
    uint8_t abuf[6];
    acc_read_burst(BMI088_ACC::DATA_X_LSB, abuf, 6);

    int16_t raw_ax = (int16_t)((abuf[1] << 8) | abuf[0]);
    int16_t raw_ay = (int16_t)((abuf[3] << 8) | abuf[2]);
    int16_t raw_az = (int16_t)((abuf[5] << 8) | abuf[4]);

    out.ax_ms2 = raw_ax * _accel_lsb_to_ms2;
    out.ay_ms2 = raw_ay * _accel_lsb_to_ms2;
    out.az_ms2 = raw_az * _accel_lsb_to_ms2;

    // --- Temperature (from accelerometer) ---
    uint8_t tbuf[2];
    acc_read_burst(BMI088_ACC::TEMP_MSB, tbuf, 2);
    int16_t raw_temp = (int16_t)(((tbuf[0] << 3) | (tbuf[1] >> 5)));
    if (raw_temp > 1023) raw_temp -= 2048;
    out.temp_c = raw_temp * 0.125f + 23.0f;

    // --- Gyroscope ---
    uint8_t gbuf[6];
    gyr_read_burst(BMI088_GYR::DATA_X_LSB, gbuf, 6);

    int16_t raw_gx = (int16_t)((gbuf[1] << 8) | gbuf[0]);
    int16_t raw_gy = (int16_t)((gbuf[3] << 8) | gbuf[2]);
    int16_t raw_gz = (int16_t)((gbuf[5] << 8) | gbuf[4]);

    out.gx_rads = raw_gx * _gyro_lsb_to_rads;
    out.gy_rads = raw_gy * _gyro_lsb_to_rads;
    out.gz_rads = raw_gz * _gyro_lsb_to_rads;

    out.valid = true;
    return true;
}

// ===========================================================================
//  PRIVATE: Accelerometer init
// ===========================================================================
bool BMI088::init_acc()
{
    // --- Quirk 1: First SPI transaction switches ACC from I2C to SPI mode ---
    acc_read_reg(BMI088_ACC::CHIP_ID);  // dummy read — return value discarded
    delay(1);

    // --- Quirk 2: Soft reset ---
    acc_write_reg(BMI088_ACC::SOFTRESET, 0xB6);
    delay(10);

    // Dummy read again after reset (quirk 1 repeats after reset)
    acc_read_reg(BMI088_ACC::CHIP_ID);
    delay(1);

    // Verify chip ID
    uint8_t id = acc_read_reg(BMI088_ACC::CHIP_ID);
    if (id != 0x1E) {
        DEBUG_SERIAL.print("[BMI088 ACC] Bad chip ID: 0x");
        DEBUG_SERIAL.println(id, HEX);
        return false;
    }

    // --- Quirk 3: Enable accelerometer (exits suspend mode) ---
    acc_write_reg(BMI088_ACC::PWR_CTRL, 0x04);
    delay(5);   // Quirk 4: 5 ms settle after entering normal mode

    // --- Configure ODR and bandwidth ---
    // BMI088_ACC_ODR_400HZ = 0xEA → OSR2 oversampling, 400 Hz ODR
    acc_write_reg(BMI088_ACC::CONF, BMI088_ACC_ODR_400HZ);

    // --- Configure range: ±6g ---
    acc_write_reg(BMI088_ACC::RANGE, BMI088_ACC_RANGE_6G);

    // Sensitivity: full scale / (2^15 - 1)
    // ±6g range → full scale = 6 * 9.80665 m/s²
    _accel_lsb_to_ms2 = (6.0f * GRAVITY_MS2) / 32767.0f;

    // Active power mode (PWR_CONF bit[0] = 0)
    acc_write_reg(BMI088_ACC::PWR_CONF, 0x00);
    delay(1);

    DEBUG_SERIAL.print("[BMI088 ACC] Chip ID OK.  Scale = ");
    DEBUG_SERIAL.print(_accel_lsb_to_ms2 * 1000.0f, 4);
    DEBUG_SERIAL.println(" mm/s² per LSB");
    return true;
}

// ===========================================================================
//  PRIVATE: Gyroscope init
// ===========================================================================
bool BMI088::init_gyr()
{
    // Soft reset
    gyr_write_reg(BMI088_GYR::SOFTRESET, 0xB6);
    delay(30);  // Quirk 5: GYR needs 30 ms after reset

    // Verify chip ID
    uint8_t id = gyr_read_reg(BMI088_GYR::CHIP_ID);
    if (id != 0x0F) {
        DEBUG_SERIAL.print("[BMI088 GYR] Bad chip ID: 0x");
        DEBUG_SERIAL.println(id, HEX);
        return false;
    }

    // Configure range: ±500 dps
    gyr_write_reg(BMI088_GYR::RANGE, BMI088_GYR_RANGE_500);

    // ODR 400 Hz, filter BW 47 Hz
    gyr_write_reg(BMI088_GYR::BANDWIDTH, BMI088_GYR_BW_47HZ);

    // Normal power mode (LPM1 = 0x00)
    gyr_write_reg(BMI088_GYR::LPM1, 0x00);
    delay(2);

    // Sensitivity: ±500 dps over ±2^15
    _gyro_lsb_to_rads = (500.0f * (float)M_PI / 180.0f) / 32767.0f;

    DEBUG_SERIAL.print("[BMI088 GYR] Chip ID OK.  Scale = ");
    DEBUG_SERIAL.print(_gyro_lsb_to_rads * 1e6f, 4);
    DEBUG_SERIAL.println(" urad/s per LSB");
    return true;
}

// ===========================================================================
//  PRIVATE: SPI helpers — Accelerometer
//  NOTE: ACC SPI read requires ONE dummy byte after the address byte (Quirk 6)
// ===========================================================================
void BMI088::acc_write_reg(uint8_t reg, uint8_t val)
{
    _spi.beginTransaction(_spi_settings);
    digitalWrite(_cs_acc, LOW);
    _spi.transfer(reg & 0x7F);   // write: MSB = 0
    _spi.transfer(val);
    digitalWrite(_cs_acc, HIGH);
    _spi.endTransaction();
}

uint8_t BMI088::acc_read_reg(uint8_t reg)
{
    uint8_t val;
    _spi.beginTransaction(_spi_settings);
    digitalWrite(_cs_acc, LOW);
    _spi.transfer(reg | 0x80);   // read:  MSB = 1
    _spi.transfer(0x00);         // Quirk 6: mandatory dummy byte
    val = _spi.transfer(0x00);
    digitalWrite(_cs_acc, HIGH);
    _spi.endTransaction();
    return val;
}

void BMI088::acc_read_burst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    _spi.beginTransaction(_spi_settings);
    digitalWrite(_cs_acc, LOW);
    _spi.transfer(reg | 0x80);
    _spi.transfer(0x00);         // Quirk 6
    for (uint8_t i = 0; i < len; i++) buf[i] = _spi.transfer(0x00);
    digitalWrite(_cs_acc, HIGH);
    _spi.endTransaction();
}

// ===========================================================================
//  PRIVATE: SPI helpers — Gyroscope
//  NOTE: GYR has standard SPI — NO dummy byte required
// ===========================================================================
void BMI088::gyr_write_reg(uint8_t reg, uint8_t val)
{
    _spi.beginTransaction(_spi_settings);
    digitalWrite(_cs_gyr, LOW);
    _spi.transfer(reg & 0x7F);
    _spi.transfer(val);
    digitalWrite(_cs_gyr, HIGH);
    _spi.endTransaction();
}

uint8_t BMI088::gyr_read_reg(uint8_t reg)
{
    uint8_t val;
    _spi.beginTransaction(_spi_settings);
    digitalWrite(_cs_gyr, LOW);
    _spi.transfer(reg | 0x80);
    val = _spi.transfer(0x00);   // No dummy byte for GYR
    digitalWrite(_cs_gyr, HIGH);
    _spi.endTransaction();
    return val;
}

void BMI088::gyr_read_burst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    _spi.beginTransaction(_spi_settings);
    digitalWrite(_cs_gyr, LOW);
    _spi.transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) buf[i] = _spi.transfer(0x00);
    digitalWrite(_cs_gyr, HIGH);
    _spi.endTransaction();
}
