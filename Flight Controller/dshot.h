#pragma once
// =============================================================================
//  dshot.h  —  DSHOT600 motor output via RP2350 PIO
//
//  DSHOT protocol overview:
//    • Digital serial protocol replacing PWM for brushless ESCs
//    • Packet: 16 bits, MSB first, transmitted at 600 kbit/s (DSHOT600)
//    • Bit encoding: start-high, time-ratio determines 0 or 1
//        '1' bit: 75% duty  → 12 cycles high + 4 cycles low  @ 9.6 MHz PIO
//        '0' bit: 37% duty  →  6 cycles high + 10 cycles low @ 9.6 MHz PIO
//    • Packet structure (16 bits):
//        [15:5]  11-bit throttle value (0=disarm, 1=reserved, 48–2047=throttle)
//        [4]     1-bit telemetry request flag
//        [3:0]   4-bit CRC = XOR of bits[15:4] nibbles
//    • No inter-byte gap needed; ESC detects packet by >2µs gap
//
//  RP2350 PIO implementation:
//    • One PIO state machine per motor (4 motors → PIO0 SM0–SM3)
//    • All 4 SMs share the same PIO program (loaded once)
//    • Each SM is configured with its own output pin (side-set)
//    • Host writes DSHOT packet (upper 16 bits of a 32-bit word) at 2 kHz
//    • SM auto-stalls between packets — natural inter-frame gap
//
//  PIO clock:  150 MHz / (15 + 160/256) ≈ 9.615 MHz
//  Bit period: 16 PIO cycles ≈ 1664 ns → 601 kbit/s (within ±2% of spec)
//
//  PIO program (5 instructions, pre-assembled):
//    Addr 0: out x, 1     side 1 [4]   ; shift bit, HIGH, 5 cycles
//    Addr 1: jmp !x, 4   side 1 [0]   ; check bit, HIGH, 1 cycle
//    Addr 2: nop          side 1 [5]   ; stay HIGH, 6 cycles (total 12 high for '1')
//    Addr 3: jmp 0        side 0 [3]   ; LOW, 4 cycles → next bit
//    Addr 4: nop          side 0 [9]   ; LOW, 10 cycles ('0' path)
//    wrap: 4→0
//
//  Quadrotor motor layout (X-frame, viewed from above):
//
//         FRONT
//      1(CW)  2(CCW)
//         ╲  ╱
//         ╱  ╲
//      3(CCW) 4(CW)
//         REAR
//
//  Mixer signs:
//    M1 = thr + roll − pitch − yaw   (front-right, CW)
//    M2 = thr − roll − pitch + yaw   (front-left,  CCW)
//    M3 = thr − roll + pitch − yaw   (rear-left,   CW)
//    M4 = thr + roll + pitch + yaw   (rear-right,  CCW)
// =============================================================================

#include <Arduino.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include "config.h"

// =============================================================================
//  DSHOT600 PIO program — pre-assembled opcodes
//  Generated from:
//    .program dshot600
//    .side_set 1
//    .wrap_target
//    out x, 1   side 1 [4]     ; 0x7421
//    jmp !x, 4  side 1 [0]     ; 0x1064
//    nop        side 1 [5]     ; 0xB542
//    jmp 0      side 0 [3]     ; 0x0300
//    nop        side 0 [9]     ; 0xA942
//    .wrap
//
//  PIO config:
//    shift_dir = LEFT (MSB first)
//    autopull  = true, threshold = 16
//    wrap_top  = 4, wrap_bot = 0
// =============================================================================
static const uint16_t dshot600_program_instructions[] = {
    0x7421u,    // addr 0: out x, 1    side 1 [4]
    0x1064u,    // addr 1: jmp !x, 4  side 1 [0]
    0xB542u,    // addr 2: nop         side 1 [5]
    0x0300u,    // addr 3: jmp 0       side 0 [3]
    0xA942u,    // addr 4: nop         side 0 [9]
};

static const struct pio_program dshot600_program = {
    .instructions = dshot600_program_instructions,
    .length       = 5,
    .origin       = -1,  // auto-allocate in PIO instruction memory
};

// =============================================================================
//  DShotMotors class — manages 4 motor outputs
// =============================================================================
class DShotMotors {
public:
    DShotMotors();

    // Initialise PIO and all 4 state machines.
    // Call once from setup() after SPI/I2C are configured.
    bool begin();

    // Write throttle values to all 4 motors.
    //   values[0..3]: DSHOT throttle (0=disarm, 48–2047=throttle, or special commands)
    //   telemetry:    set bit i to request telemetry from motor i
    void write(const uint16_t values[4], uint8_t telemetry_mask = 0);

    // Convenience: write same value to all motors (e.g., during arming check)
    void write_all(uint16_t value);

    // Stop all motors (send DSHOT disarm command = 0)
    void disarm();

    // Convert normalised throttle [0..1] to DSHOT value [48..2047]
    static uint16_t normalised_to_dshot(float throttle_norm);

    bool is_ready() const { return _ready; }

private:
    bool     _ready;
    PIO      _pio;
    uint     _prog_offset;
    uint     _sm[4];          // state machine handles

    static const uint8_t _motor_pins[4];

    // Build DSHOT packet: [throttle 11-bit][telem 1-bit][CRC 4-bit]
    static uint16_t build_packet(uint16_t throttle, bool telemetry);

    // Load packet into SM TX FIFO (shifted to upper 16 bits of 32-bit word)
    void push_packet(uint sm_idx, uint16_t packet);

    // Configure one state machine for a given output pin
    bool init_sm(uint sm_idx, uint gpio_pin);
};
