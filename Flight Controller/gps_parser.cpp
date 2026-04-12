// =============================================================================
//  gps_parser.cpp
// =============================================================================
#include "gps_parser.h"
#include "config.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

GpsParser::GpsParser(HardwareSerial &serial)
    : _serial(serial), _buf_idx(0), _in_sentence(false),
      _sentence_count(0), _checksum_errors(0)
{
    memset(&_fix, 0, sizeof(_fix));
}

void GpsParser::begin(uint32_t baud)
{
    _serial.begin(baud);
    // On RP2350/arduino-pico, pin assignments are via Serial1.setRX/setTX:
    // Serial1.setTX(PIN_GPS_TX);
    // Serial1.setRX(PIN_GPS_RX);
    // These must be called BEFORE .begin() if using non-default pins.
}

// ---------------------------------------------------------------------------
//  update() — called repeatedly from Core 1 loop
// ---------------------------------------------------------------------------
bool GpsParser::update()
{
    bool got_sentence = false;

    while (_serial.available()) {
        char c = (char)_serial.read();

        if (c == '$') {
            // Start of new sentence
            _in_sentence = true;
            _buf_idx     = 0;
            _buf[0]      = '\0';
        }
        else if (_in_sentence) {
            if (c == '\n' || c == '\r') {
                // End of sentence
                _buf[_buf_idx] = '\0';
                if (_buf_idx > 5) {
                    if (parse_sentence(_buf)) {
                        got_sentence = true;
                        _sentence_count++;
                    }
                }
                _in_sentence = false;
            }
            else if (_buf_idx < NMEA_MAX_LEN - 1) {
                _buf[_buf_idx++] = c;
            }
            else {
                // Buffer overflow — discard
                _in_sentence = false;
            }
        }
    }
    return got_sentence;
}

GpsData GpsParser::get_fix() const { return _fix; }

// ---------------------------------------------------------------------------
//  parse_sentence() — dispatch by sentence type
// ---------------------------------------------------------------------------
bool GpsParser::parse_sentence(const char *sentence)
{
    if (!verify_checksum(sentence)) {
        _checksum_errors++;
        return false;
    }

    // Make a mutable copy for strtok-style splitting
    char tmp[NMEA_MAX_LEN];
    strncpy(tmp, sentence, NMEA_MAX_LEN - 1);
    tmp[NMEA_MAX_LEN - 1] = '\0';

    char *fields[NMEA_MAX_FIELDS];
    int n = split_fields(tmp, fields, NMEA_MAX_FIELDS);
    if (n < 2) return false;

    // Match sentence type — accept any talker prefix (GP, GN, GL, GA, GB)
    // e.g., "$GPRMC" → fields[0] = "GPRMC"  ($ already stripped by receiver)
    const char *type = fields[0];
    int len = strlen(type);
    if (len < 5) return false;

    // Pointer past the two-char talker ID (e.g., "GP", "GN")
    const char *msg = type + 2;

    if (strncmp(msg, "RMC", 3) == 0) return parse_rmc(fields, n);
    if (strncmp(msg, "GGA", 3) == 0) return parse_gga(fields, n);
    return false;
}

// ---------------------------------------------------------------------------
//  parse_rmc() — Recommended Minimum Specific GNSS Data
//
//  $GPRMC,time,status,lat,N/S,lon,E/W,speed_knots,cog,date,magvar,mode*cs
//  Field indices (0-based, after leading '$' stripped):
//    0: sentence type   e.g. "GPRMC"
//    1: UTC time        HHMMSS.ss
//    2: status          A=active, V=void
//    3: latitude        DDMM.MMMM
//    4: N/S
//    5: longitude       DDDMM.MMMM
//    6: E/W
//    7: speed over ground (knots)
//    8: course over ground (degrees true)
//    9: date            DDMMYY
// ---------------------------------------------------------------------------
bool GpsParser::parse_rmc(char **fields, int n)
{
    if (n < 9) return false;

    // Status must be 'A' (active)
    if (fields[2][0] != 'A') {
        _fix.valid = false;
        return false;
    }

    double lat = parse_lat(fields[3], fields[4]);
    double lon = parse_lon(fields[5], fields[6]);
    float sog_ms = parse_float(fields[7]) * 0.514444f;  // knots → m/s
    float cog_deg = parse_float(fields[8]);
    float cog_rad = cog_deg * DEG2RAD;

    _fix.lat_deg   = lat;
    _fix.lon_deg   = lon;
    _fix.vel_n_ms  = sog_ms * cosf(cog_rad);
    _fix.vel_e_ms  = sog_ms * sinf(cog_rad);
    _fix.vel_d_ms  = 0.0f;   // RMC gives no vertical velocity
    _fix.timestamp_ms = millis();
    // valid flag set when GGA confirms fix quality ≥ 1
    return true;
}

// ---------------------------------------------------------------------------
//  parse_gga() — Global Positioning System Fix Data
//
//  $GPGGA,time,lat,N/S,lon,E/W,quality,numSV,HDOP,alt,M,sep,M,diffAge,diffSta*cs
//    0: type
//    1: UTC time
//    2: latitude
//    3: N/S
//    4: longitude
//    5: E/W
//    6: fix quality  0=no fix, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
//    7: number of satellites
//    8: HDOP
//    9: altitude (MSL)
//   10: M (unit)
// ---------------------------------------------------------------------------
bool GpsParser::parse_gga(char **fields, int n)
{
    if (n < 10) return false;

    int quality = parse_int(fields[6]);
    if (quality <= 0) {
        _fix.valid = false;
        return false;
    }

    _fix.lat_deg     = parse_lat(fields[2], fields[3]);
    _fix.lon_deg     = parse_lon(fields[4], fields[5]);
    _fix.alt_msl_m   = parse_float(fields[9]);
    _fix.fix_quality = (uint8_t)quality;
    _fix.num_sats    = (uint8_t)parse_int(fields[7]);
    _fix.hdop        = parse_float(fields[8]);
    _fix.timestamp_ms = millis();
    _fix.valid       = true;
    return true;
}

// ---------------------------------------------------------------------------
//  NMEA checksum:  XOR of all bytes between '$' and '*' (exclusive)
//  Format: *HH   where HH is two hex digits
// ---------------------------------------------------------------------------
bool GpsParser::verify_checksum(const char *sentence)
{
    // sentence starts AFTER '$' (stripped during receive)
    // Find '*'
    const char *star = strchr(sentence, '*');
    if (!star || strlen(star) < 3) return false;

    uint8_t calc = 0;
    for (const char *p = sentence; p != star; p++)
        calc ^= (uint8_t)*p;

    // Parse two hex chars after '*'
    char hex[3] = { star[1], star[2], '\0' };
    uint8_t provided = (uint8_t)strtol(hex, nullptr, 16);
    return calc == provided;
}

// ---------------------------------------------------------------------------
//  split_fields() — split on commas in-place, stop at '*'
// ---------------------------------------------------------------------------
int GpsParser::split_fields(char *sentence, char **fields, int max_fields)
{
    int count = 0;
    fields[count++] = sentence;

    for (char *p = sentence; *p && *p != '*' && count < max_fields; p++) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }
    // Null-terminate at '*'
    char *star = strchr(sentence, '*');
    if (star) *star = '\0';

    return count;
}

// ---------------------------------------------------------------------------
//  Parse NMEA latitude:  DDMM.MMMM + N/S
// ---------------------------------------------------------------------------
double GpsParser::parse_lat(const char *val, const char *hem)
{
    if (!val || strlen(val) < 4) return 0.0;
    double raw = parse_double(val);
    int    deg  = (int)(raw / 100.0);
    double min  = raw - deg * 100.0;
    double result = deg + min / 60.0;
    if (hem && hem[0] == 'S') result = -result;
    return result;
}

// Parse NMEA longitude: DDDMM.MMMM + E/W
double GpsParser::parse_lon(const char *val, const char *hem)
{
    if (!val || strlen(val) < 5) return 0.0;
    double raw = parse_double(val);
    int    deg  = (int)(raw / 100.0);
    double min  = raw - deg * 100.0;
    double result = deg + min / 60.0;
    if (hem && hem[0] == 'W') result = -result;
    return result;
}

float  GpsParser::parse_float (const char *s) { return s && *s ? atof(s) : 0.0f; }
double GpsParser::parse_double(const char *s) { return s && *s ? atof(s) : 0.0;  }
int    GpsParser::parse_int   (const char *s) { return s && *s ? atoi(s) : 0;    }
