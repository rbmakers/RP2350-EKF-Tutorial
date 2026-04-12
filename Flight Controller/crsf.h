#pragma once
// =============================================================================
//  crsf.h  —  CRSF (CrossFire Serial Format) receiver parser
//             Compatible with ExpressLRS, TBS Crossfire, ELRS
//
//  Protocol details:
//    • UART: 420000 baud, 8N1
//    • Frame: [SYNC 0xC8][LEN][TYPE][PAYLOAD ...][CRC-8/DVB-S2]
//      - SYNC = 0xC8 (device address "flight controller")
//      - LEN  = payload length + 2 (type byte + CRC byte)
//      - TYPE = frame type identifier
//      - CRC  = CRC-8/DVB-S2 over bytes [TYPE .. PAYLOAD end]
//
//    Key frame types:
//      0x16  CRSF_FRAMETYPE_RC_CHANNELS_PACKED  — 16 × 11-bit RC channels
//      0x1E  CRSF_FRAMETYPE_LINK_STATISTICS     — RSSI, LQ, SNR, power
//
//    RC Channels (0x16) payload (22 bytes):
//      16 channels, each 11 bits, packed LSB-first:
//        ch[0]  = payload[0] | (payload[1] << 8)  & 0x7FF
//        ch[1]  = (payload[1] >> 3) | (payload[2] << 5)  & 0x7FF
//        ...
//      Range: 172–1811, center 992 (≈ 1500µs in PWM terms)
//
//    ELRS bind button: momentary low on CRSF_RX during power-on triggers bind
//
//  Usage:
//    CrsfReceiver rx(CRSF_SERIAL);
//    rx.begin();                        // call from Core 1 setup1()
//    rx.update();                       // call in Core 1 loop1()
//    int16_t ch = rx.channel(n);        // read channel [0..15]
//    bool armed = rx.channel(RC_CH_ARM) > CRSF_ARM_THRESH;
// =============================================================================

#include <Arduino.h>
#include "config.h"

// Frame type constants
constexpr uint8_t CRSF_SYNC_BYTE        = 0xC8;
constexpr uint8_t CRSF_FRAMETYPE_RC     = 0x16;
constexpr uint8_t CRSF_FRAMETYPE_LINK   = 0x1E;
constexpr uint8_t CRSF_MAX_FRAME_LEN    = 64;
constexpr uint8_t CRSF_NUM_CHANNELS     = 16;

// Link statistics struct (from 0x1E frame)
struct CrsfLinkStats {
    uint8_t  uplink_rssi_1;     // dBm (negative, reported as positive value)
    uint8_t  uplink_rssi_2;
    uint8_t  uplink_link_quality;  // 0–100 %
    int8_t   uplink_snr;           // dB
    uint8_t  active_antenna;
    uint8_t  rf_mode;
    uint8_t  uplink_tx_power;
    uint8_t  downlink_rssi;
    uint8_t  downlink_link_quality;
    int8_t   downlink_snr;
};

// =============================================================================
//  CrsfReceiver class
// =============================================================================
class CrsfReceiver {
public:
    explicit CrsfReceiver(HardwareSerial &serial);

    // Call from Core 1 setup1()
    void begin();

    // Call in Core 1 loop1() — processes all available bytes
    // Returns true if a new RC frame was received this call
    bool update();

    // Get latest RC channel value (raw CRSF range 172–1811)
    int16_t channel(uint8_t ch) const {
        return (ch < CRSF_NUM_CHANNELS) ? _channels[ch] : CRSF_MID;
    }

    // Get normalised channel value:
    //   throttle (ch2): 0.0 – 1.0
    //   other channels: −1.0 – +1.0  (deadband applied)
    float channel_norm(uint8_t ch) const;
    float throttle_norm() const;

    // Link quality (0–100%)
    uint8_t link_quality() const { return _link.uplink_link_quality; }
    uint8_t rssi() const         { return _link.uplink_rssi_1; }
    const CrsfLinkStats& link_stats() const { return _link; }

    // Time since last valid RC frame (ms) — use for failsafe detection
    uint32_t ms_since_last_rc() const { return millis() - _last_rc_ms; }
    bool     is_failsafe() const { return ms_since_last_rc() > RC_FAILSAFE_MS; }

    // Packet statistics
    uint32_t packet_count()   const { return _packet_count; }
    uint32_t crc_errors()     const { return _crc_errors; }

private:
    HardwareSerial &_serial;

    // Receive buffer state machine
    uint8_t  _buf[CRSF_MAX_FRAME_LEN];
    uint8_t  _buf_idx;
    enum class ParseState : uint8_t {
        WAIT_SYNC, WAIT_LEN, WAIT_DATA
    } _state;
    uint8_t  _expected_len;

    // Parsed data
    int16_t        _channels[CRSF_NUM_CHANNELS];
    CrsfLinkStats  _link;
    uint32_t       _last_rc_ms;

    // Statistics
    uint32_t _packet_count;
    uint32_t _crc_errors;

    // CRC-8/DVB-S2 lookup table (pre-computed)
    static const uint8_t _crc8_table[256];

    // Internal helpers
    bool   process_frame(const uint8_t *buf, uint8_t len);
    void   decode_rc_channels(const uint8_t *payload);
    void   decode_link_stats(const uint8_t *payload);
    uint8_t crc8(const uint8_t *data, uint8_t len) const;
};
