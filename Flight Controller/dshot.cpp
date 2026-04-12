// =============================================================================
//  dshot.cpp  —  DSHOT600 via RP2350 PIO
// =============================================================================
#include "dshot.h"

const uint8_t DShotMotors::_motor_pins[4] = {
    PIN_MOTOR_1, PIN_MOTOR_2, PIN_MOTOR_3, PIN_MOTOR_4
};

DShotMotors::DShotMotors() : _ready(false), _pio(DSHOT_PIO), _prog_offset(0)
{
    memset(_sm, 0, sizeof(_sm));
}

// ---------------------------------------------------------------------------
//  begin()
// ---------------------------------------------------------------------------
bool DShotMotors::begin()
{
    // Check there's room in PIO instruction memory (5 instructions)
    if (!pio_can_add_program(_pio, &dshot600_program)) {
        DEBUG_SERIAL.println("[DSHOT] ERROR: No room in PIO instruction memory");
        return false;
    }

    // Load the program once; all 4 SMs share the same code
    _prog_offset = pio_add_program(_pio, &dshot600_program);

    // Initialise each SM
    for (int i = 0; i < 4; i++) {
        _sm[i] = (uint)i;   // SM0..SM3 for motors 1..4
        if (!init_sm(_sm[i], _motor_pins[i])) {
            DEBUG_SERIAL.print("[DSHOT] ERROR: SM");
            DEBUG_SERIAL.print(i);
            DEBUG_SERIAL.println(" init failed");
            return false;
        }
    }

    _ready = true;
    DEBUG_SERIAL.println("[DSHOT] OK — 4× DSHOT600 state machines started");
    return true;
}

// ---------------------------------------------------------------------------
//  init_sm()  — configure and start one PIO state machine
// ---------------------------------------------------------------------------
bool DShotMotors::init_sm(uint sm, uint pin)
{
    // Configure pin as PIO output, initial state LOW
    pio_gpio_init(_pio, pin);
    gpio_set_drive_strength((gpio_num_t)pin, GPIO_DRIVE_STRENGTH_4MA);

    // Get default SM config for our program
    pio_sm_config c = pio_get_default_sm_config();

    // Output pin: side-set pin = our motor pin
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_sideset(&c, 1, false, false);  // 1 bit, not optional, no pindirs

    // Set pin direction to output for this SM
    pio_sm_set_consecutive_pindirs(_pio, sm, pin, 1, true);

    // Clock divider: 150 MHz / 9.6154 MHz ≈ 15 + 160/256
    // Bit period = 16 cycles ≈ 1664 ns → 601 kbit/s
    sm_config_set_clkdiv_int_frac(&c,
        DSHOT_PIO_CLK_INT, DSHOT_PIO_CLK_FRAC);

    // TX FIFO: shift left (MSB first), autopull at 16-bit threshold
    // Host writes the 16-bit DSHOT packet in the upper half of a 32-bit word
    sm_config_set_out_shift(&c,
        false,   // shift_direction: false = left (MSB first)
        true,    // autopull: true
        16);     // autopull threshold: 16 bits

    // Wrap: instruction 4 wraps to instruction 0
    sm_config_set_wrap(&c,
        _prog_offset + 0,   // wrap_target (bottom)
        _prog_offset + 4);  // wrap (top)

    // Load configuration and set program counter to start
    pio_sm_init(_pio, sm, _prog_offset, &c);

    // Enable the SM
    pio_sm_set_enabled(_pio, sm, true);

    return true;
}

// ---------------------------------------------------------------------------
//  write()
// ---------------------------------------------------------------------------
void DShotMotors::write(const uint16_t values[4], uint8_t telem_mask)
{
    for (int i = 0; i < 4; i++) {
        bool telem = (telem_mask >> i) & 1;
        uint16_t packet = build_packet(values[i], telem);
        push_packet(_sm[i], packet);
    }
}

void DShotMotors::write_all(uint16_t value)
{
    uint16_t v[4] = { value, value, value, value };
    write(v);
}

void DShotMotors::disarm()
{
    write_all(DSHOT_DISARM_VAL);
}

// ---------------------------------------------------------------------------
//  build_packet()
//
//  DSHOT packet structure (16 bits):
//    [15:5]  11-bit throttle  (0 = stop/disarm, 1-47 = reserved cmds, 48-2047 = throttle)
//    [4]     telemetry request bit
//    [3:0]   CRC = (throttle XOR (throttle >> 4) XOR (throttle >> 8)) & 0x0F
//
//  The CRC is computed over the upper 12 bits (throttle + telem bit).
// ---------------------------------------------------------------------------
uint16_t DShotMotors::build_packet(uint16_t throttle, bool telemetry)
{
    // Clamp throttle to valid range
    throttle = constrain(throttle, 0, DSHOT_MAX_THROTTLE);

    // Build the 12-bit value: [11:1]=throttle, [0]=telem
    uint16_t packet = (throttle << 1) | (telemetry ? 1 : 0);

    // CRC: XOR of each nibble of the 12-bit packet
    uint8_t crc = ((packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F);

    return (packet << 4) | crc;
}

// ---------------------------------------------------------------------------
//  push_packet()
//
//  The PIO program uses `out x, 1` with autopull at 16-bit threshold.
//  We write the 16-bit DSHOT packet into the upper half of a 32-bit word
//  so that after the first 16 `out` operations, the OSR is exhausted and
//  the SM stalls — creating a natural inter-frame gap until the next write.
// ---------------------------------------------------------------------------
void DShotMotors::push_packet(uint sm, uint16_t packet)
{
    // Non-blocking push: if FIFO full, we skip (should not happen at 2 kHz)
    // The TX FIFO depth is 4 × 32-bit words; we write 1 per cycle.
    if (!pio_sm_is_tx_fifo_full(_pio, sm)) {
        // Shift packet to upper 16 bits (PIO shifts MSB first from OSR)
        pio_sm_put(_pio, sm, (uint32_t)packet << 16);
    }
}

// ---------------------------------------------------------------------------
//  normalised_to_dshot()
// ---------------------------------------------------------------------------
uint16_t DShotMotors::normalised_to_dshot(float throttle_norm)
{
    if (throttle_norm <= 0.0f) return 0;  // disarm
    // Map [0, 1] → [DSHOT_MIN, DSHOT_MAX]
    float val = DSHOT_MIN_THROTTLE
              + throttle_norm * (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE);
    return (uint16_t)constrain((int)val, DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE);
}
