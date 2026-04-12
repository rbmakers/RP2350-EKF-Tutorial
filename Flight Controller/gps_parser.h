#pragma once
// =============================================================================
//  gps_parser.h  —  Minimal NMEA 0183 parser for GNSS + INS fusion
//
//  Parses two sentence types:
//    $GxRMC  — position, ground speed, heading, date/time
//    $GxGGA  — position, altitude, fix quality, satellite count
//  where 'x' may be P (GPS), N (GLONASS), L (Galileo), etc.
//
//  Output coordinate frame:
//    Latitude   : decimal degrees  (+N / −S)
//    Longitude  : decimal degrees  (+E / −W)
//    Altitude   : metres above mean sea level (MSL)
//
//  NED velocity is derived from speed-over-ground (SOG) and
//  course-over-ground (COG) provided by RMC.
//
//  Usage (from Core 1):
//    GpsParser gps(GPS_SERIAL);
//    gps.begin();
//    while(true) { gps.update(); }   // call as fast as possible
//
//  Usage (from Core 0 — read shared data):
//    GpsData fix = gps.get_fix();
//    if (fix.valid && (millis() - fix.timestamp_ms < GPS_MAX_AGE_MS)) { ... }
// =============================================================================

#include <Arduino.h>

// Maximum number of comma-separated fields in one NMEA sentence
#define NMEA_MAX_FIELDS  20
// Maximum raw sentence length (NMEA spec: 82 chars max)
#define NMEA_MAX_LEN     100

// Fix quality codes from GGA
enum class GpsFixQuality : uint8_t {
    NO_FIX    = 0,
    GPS_FIX   = 1,
    DGPS_FIX  = 2,
    RTK_FIXED = 4,
    RTK_FLOAT = 5,
};

// Structured GPS fix data — written by Core 1, read by Core 0
struct GpsData {
    // Position
    double   lat_deg;       // decimal degrees (+N)
    double   lon_deg;       // decimal degrees (+E)
    float    alt_msl_m;     // altitude above MSL (m)

    // Velocity derived from SOG + COG
    float    vel_n_ms;      // velocity North (m/s)
    float    vel_e_ms;      // velocity East  (m/s)
    float    vel_d_ms;      // velocity Down  — from GGA altitude rate (approximated 0)

    // Quality
    uint8_t  fix_quality;   // GpsFixQuality cast to uint8_t
    uint8_t  num_sats;
    float    hdop;          // horizontal dilution of precision

    // Time
    uint32_t timestamp_ms;  // millis() when this fix was parsed
    bool     valid;         // true when fix_quality >= GPS_FIX
};

// =============================================================================
//  GpsParser class
// =============================================================================
class GpsParser {
public:
    explicit GpsParser(HardwareSerial &serial);

    // Call once from Core 1 setup
    void begin(uint32_t baud = 115200);

    // Call in Core 1 loop — feeds parser one character at a time
    // Returns true when a complete, valid sentence was processed
    bool update();

    // Thread-safe snapshot of latest fix (atomic copy via seqlock in main)
    // Core 0 should use shared_state.h instead of calling this directly
    GpsData get_fix() const;

    // Total sentence count (diagnostic)
    uint32_t sentence_count() const { return _sentence_count; }

private:
    HardwareSerial &_serial;

    // Receive buffer
    char     _buf[NMEA_MAX_LEN];
    uint8_t  _buf_idx;
    bool     _in_sentence;

    // Latest parsed fix
    GpsData  _fix;

    // Statistics
    uint32_t _sentence_count;
    uint32_t _checksum_errors;

    // Internal parsing
    bool parse_sentence(const char *sentence);
    bool parse_rmc(char **fields, int n);
    bool parse_gga(char **fields, int n);

    // NMEA helpers
    bool     verify_checksum(const char *sentence);
    int      split_fields(char *sentence, char **fields, int max_fields);
    double   parse_lat(const char *val, const char *hem);
    double   parse_lon(const char *val, const char *hem);
    float    parse_float(const char *s);
    double   parse_double(const char *s);
    int      parse_int(const char *s);
};
