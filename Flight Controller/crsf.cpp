// =============================================================================
//  crsf.cpp
// =============================================================================
#include "crsf.h"
#include <string.h>

// CRC-8/DVB-S2 lookup table  (polynomial 0xD5)
const uint8_t CrsfReceiver::_crc8_table[256] = {
    0x00,0xD5,0x7F,0xAA,0xFE,0x2B,0x81,0x54,0x29,0xFC,0x56,0x83,0xD7,0x02,0xA8,0x7D,
    0x52,0x87,0x2D,0xF8,0xAC,0x79,0xD3,0x06,0x7B,0xAE,0x04,0xD1,0x85,0x50,0xFA,0x2F,
    0xA4,0x71,0xDB,0x0E,0x5A,0x8F,0x25,0xF0,0x8D,0x58,0xF2,0x27,0x73,0xA6,0x0C,0xD9,
    0xF6,0x23,0x89,0x5C,0x08,0xDD,0x77,0xA2,0xDF,0x0A,0xA0,0x75,0x21,0xF4,0x5E,0x8B,
    0x9D,0x48,0xE2,0x37,0x63,0xB6,0x1C,0xC9,0xB4,0x61,0xCB,0x1E,0x4A,0x9F,0x35,0xE0,
    0xCF,0x1A,0xB0,0x65,0x31,0xE4,0x4E,0x9B,0xE6,0x33,0x99,0x4C,0x18,0xCD,0x67,0xB2,
    0x39,0xEC,0x46,0x93,0xC7,0x12,0xB8,0x6D,0x10,0xC5,0x6F,0xBA,0xEE,0x3B,0x91,0x44,
    0x6B,0xBE,0x14,0xC1,0x95,0x40,0xEA,0x3F,0x42,0x97,0x3D,0xE8,0xBC,0x69,0xC3,0x16,
    0xEF,0x3A,0x90,0x45,0x11,0xC4,0x6E,0xBB,0xC6,0x13,0xB9,0x6C,0x38,0xED,0x47,0x92,
    0xBD,0x68,0xC2,0x17,0x43,0x96,0x3C,0xE9,0x94,0x41,0xEB,0x3E,0x6A,0xBF,0x15,0xC0,
    0x4B,0x9E,0x34,0xE1,0xB5,0x60,0xCA,0x1F,0x62,0xB7,0x1D,0xC8,0x9C,0x49,0xE3,0x36,
    0x19,0xCC,0x66,0xB3,0xE7,0x32,0x98,0x4D,0x30,0xE5,0x4F,0x9A,0xCE,0x1B,0xB1,0x64,
    0x72,0xA7,0x0D,0xD8,0x8C,0x59,0xF3,0x26,0x5B,0x8E,0x24,0xF1,0xA5,0x70,0xDA,0x0F,
    0x20,0xF5,0x5F,0x8A,0xDE,0x0B,0xA1,0x74,0x09,0xDC,0x76,0xA3,0xF7,0x22,0x88,0x5D,
    0xD6,0x03,0xA9,0x7C,0x28,0xFD,0x57,0x82,0xFF,0x2A,0x80,0x55,0x01,0xD4,0x7E,0xAB,
    0x84,0x51,0xFB,0x2E,0x7A,0xAF,0x05,0xD0,0xAD,0x78,0xD2,0x07,0x53,0x86,0x2C,0xF9,
};

CrsfReceiver::CrsfReceiver(HardwareSerial &serial)
    : _serial(serial), _buf_idx(0), _expected_len(0),
      _state(ParseState::WAIT_SYNC),
      _last_rc_ms(0), _packet_count(0), _crc_errors(0)
{
    // Initialise channels to center/min
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++)
        _channels[i] = CRSF_MID;
    _channels[RC_CH_THROTTLE] = CRSF_MIN;
    memset(&_link, 0, sizeof(_link));
}

void CrsfReceiver::begin()
{
    _serial.setTX(PIN_CRSF_TX);
    _serial.setRX(PIN_CRSF_RX);
    _serial.begin(CRSF_BAUD);
}

// ---------------------------------------------------------------------------
//  update() — call as fast as possible in Core 1 loop
// ---------------------------------------------------------------------------
bool CrsfReceiver::update()
{
    bool got_rc = false;

    while (_serial.available()) {
        uint8_t byte = (uint8_t)_serial.read();

        switch (_state) {

        case ParseState::WAIT_SYNC:
            if (byte == CRSF_SYNC_BYTE) {
                _buf[0]    = byte;
                _buf_idx   = 1;
                _state     = ParseState::WAIT_LEN;
            }
            break;

        case ParseState::WAIT_LEN:
            // LEN = number of remaining bytes (type + payload + CRC)
            // Valid range: 2 (just type+CRC) to 62
            if (byte >= 2 && byte <= CRSF_MAX_FRAME_LEN - 2) {
                _buf[1]       = byte;
                _expected_len = byte;
                _buf_idx      = 2;
                _state        = ParseState::WAIT_DATA;
            } else {
                _state = ParseState::WAIT_SYNC;
            }
            break;

        case ParseState::WAIT_DATA:
            if (_buf_idx < CRSF_MAX_FRAME_LEN) {
                _buf[_buf_idx++] = byte;
            }
            // Total frame = SYNC(1) + LEN(1) + LEN bytes
            if (_buf_idx >= 2U + _expected_len) {
                // Full frame received
                if (process_frame(_buf, _buf_idx)) {
                    if (_buf[2] == CRSF_FRAMETYPE_RC) got_rc = true;
                }
                _state = ParseState::WAIT_SYNC;
            }
            break;
        }
    }
    return got_rc;
}

// ---------------------------------------------------------------------------
//  process_frame()
// ---------------------------------------------------------------------------
bool CrsfReceiver::process_frame(const uint8_t *buf, uint8_t total_len)
{
    // buf[0]=SYNC, buf[1]=LEN, buf[2]=TYPE, buf[3..n-1]=PAYLOAD, buf[n]=CRC
    uint8_t len  = buf[1];           // LEN field
    uint8_t type = buf[2];
    uint8_t rcvd_crc = buf[1 + len]; // last byte of frame

    // CRC is computed over TYPE + PAYLOAD (not SYNC or LEN)
    uint8_t calc_crc = crc8(&buf[2], (uint8_t)(len - 1));

    if (calc_crc != rcvd_crc) {
        _crc_errors++;
        return false;
    }

    _packet_count++;
    const uint8_t *payload = &buf[3];

    switch (type) {
    case CRSF_FRAMETYPE_RC:
        decode_rc_channels(payload);
        _last_rc_ms = millis();
        return true;
    case CRSF_FRAMETYPE_LINK:
        decode_link_stats(payload);
        return true;
    default:
        return false;
    }
}

// ---------------------------------------------------------------------------
//  decode_rc_channels() — 16 channels × 11 bits, LSB first
// ---------------------------------------------------------------------------
void CrsfReceiver::decode_rc_channels(const uint8_t *p)
{
    // Unpack 22 bytes into 16 × 11-bit channels
    _channels[0]  = ((p[0]       | (p[1]  << 8)) & 0x07FF);
    _channels[1]  = ((p[1]  >> 3 | (p[2]  << 5)) & 0x07FF);
    _channels[2]  = ((p[2]  >> 6 | (p[3]  << 2) | (p[4] << 10)) & 0x07FF);
    _channels[3]  = ((p[4]  >> 1 | (p[5]  << 7)) & 0x07FF);
    _channels[4]  = ((p[5]  >> 4 | (p[6]  << 4)) & 0x07FF);
    _channels[5]  = ((p[6]  >> 7 | (p[7]  << 1) | (p[8] <<  9)) & 0x07FF);
    _channels[6]  = ((p[8]  >> 2 | (p[9]  << 6)) & 0x07FF);
    _channels[7]  = ((p[9]  >> 5 | (p[10] << 3)) & 0x07FF);
    _channels[8]  = ((p[11]      | (p[12] << 8)) & 0x07FF);
    _channels[9]  = ((p[12] >> 3 | (p[13] << 5)) & 0x07FF);
    _channels[10] = ((p[13] >> 6 | (p[14] << 2) | (p[15] << 10)) & 0x07FF);
    _channels[11] = ((p[15] >> 1 | (p[16] << 7)) & 0x07FF);
    _channels[12] = ((p[16] >> 4 | (p[17] << 4)) & 0x07FF);
    _channels[13] = ((p[17] >> 7 | (p[18] << 1) | (p[19] <<  9)) & 0x07FF);
    _channels[14] = ((p[19] >> 2 | (p[20] << 6)) & 0x07FF);
    _channels[15] = ((p[20] >> 5 | (p[21] << 3)) & 0x07FF);
}

// ---------------------------------------------------------------------------
//  decode_link_stats()
// ---------------------------------------------------------------------------
void CrsfReceiver::decode_link_stats(const uint8_t *p)
{
    _link.uplink_rssi_1         = p[0];
    _link.uplink_rssi_2         = p[1];
    _link.uplink_link_quality   = p[2];
    _link.uplink_snr            = (int8_t)p[3];
    _link.active_antenna        = p[4];
    _link.rf_mode               = p[5];
    _link.uplink_tx_power       = p[6];
    _link.downlink_rssi         = p[7];
    _link.downlink_link_quality = p[8];
    _link.downlink_snr          = (int8_t)p[9];
}

// ---------------------------------------------------------------------------
//  channel_norm() — normalised channel value with deadband
// ---------------------------------------------------------------------------
float CrsfReceiver::channel_norm(uint8_t ch) const
{
    if (ch >= CRSF_NUM_CHANNELS) return 0.0f;
    int16_t raw = _channels[ch];
    int16_t centered = raw - CRSF_MID;
    // Apply deadband
    if (abs(centered) < CRSF_DEADBAND) return 0.0f;
    if (centered > 0) centered -= CRSF_DEADBAND;
    else              centered += CRSF_DEADBAND;

    float half_range = (float)(CRSF_MAX - CRSF_MID - CRSF_DEADBAND);
    return constrain((float)centered / half_range, -1.0f, 1.0f);
}

float CrsfReceiver::throttle_norm() const
{
    // Throttle: map CRSF_MIN → 0.0,  CRSF_MAX → 1.0
    float val = (float)(_channels[RC_CH_THROTTLE] - CRSF_MIN)
              / (float)(CRSF_MAX - CRSF_MIN);
    return constrain(val, 0.0f, 1.0f);
}

// ---------------------------------------------------------------------------
//  CRC-8/DVB-S2  (table lookup)
// ---------------------------------------------------------------------------
uint8_t CrsfReceiver::crc8(const uint8_t *data, uint8_t len) const
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
        crc = _crc8_table[crc ^ data[i]];
    return crc;
}
