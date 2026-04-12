# FCS_NAV — Integrated Flight Control System + Navigation
## RP2354A · Quaternion EKF-INS · DSHOT600 · CRSF · GPS+Baro+Mag

---

## Overview

This project merges two systems onto a single RP2354A:

| Origin | Component |
|--------|-----------|
| INS_EKF_v2 | 16-state quaternion EKF, BMI088, BMM350, BMP580, GPS parser |
| dRehmFlight/Curio FC | Cascaded PID loops, DSHOT600 PIO, CRSF receiver, Betaflight features |

**Key integration:** The EKF quaternion completely replaces the Madgwick AHRS filter from the original flight controller. Attitude, position, and velocity estimates come from the EKF and feed the outer PID loops.

---

## Hardware

### Target: RP2354A
The RP2354A is the RP2350 die in QFN-60 with 8 MB PSRAM attached internally. For this project the PSRAM is unused — all data fits in the 520 KB SRAM.

### GPIO Map

| GPIO | Signal | Device |
|------|--------|--------|
| GP0  | GPS TX → receiver | UART0 (Serial1) |
| GP1  | GPS RX ← receiver | UART0 (Serial1) |
| GP2  | SPI SCK | BMI088 |
| GP3  | SPI MOSI | BMI088 |
| GP4  | SPI MISO | BMI088 |
| GP5  | ACC CS | BMI088 accel |
| GP6  | GYR CS | BMI088 gyro |
| GP8  | CRSF TX | UART1 (Serial2) |
| GP9  | CRSF RX | UART1 (Serial2) |
| GP10 | Motor 1 (Front-Right, CW) | DSHOT600 PIO0 SM0 |
| GP11 | Motor 2 (Front-Left, CCW) | DSHOT600 PIO0 SM1 |
| GP12 | Motor 3 (Rear-Left, CW) | DSHOT600 PIO0 SM2 |
| GP13 | Motor 4 (Rear-Right, CCW) | DSHOT600 PIO0 SM3 |
| GP20 | I2C SDA | BMM350 + BMP580 |
| GP21 | I2C SCL | BMM350 + BMP580 |
| GP22 | Status LED | — |

---

## Architecture

### Dual-Core Task Split

```
CORE 0 (Navigation + Control, 2 kHz)          CORE 1 (I/O, best-effort)
─────────────────────────────────────          ──────────────────────────
2 kHz:  BMI088 SPI read                        Continuous: CRSF UART parse
2 kHz:  Rate PID (raw gyro)                    Continuous: GPS NMEA parse
2 kHz:  DSHOT write                            @ 100 Hz:   BMM350 I2C read
500 Hz: EKF predict                            @  50 Hz:   BMP580 I2C read
500 Hz: Angle PID (EKF attitude)
500 Hz: RC read from shared_state
100 Hz: EKF mag update
 50 Hz: EKF baro update + altitude PID
 10 Hz: EKF GPS update + position PID
 10 Hz: Telemetry
```

### PID Cascade

```
Position loop  (10 Hz)  →  velocity setpoint  (m/s)
Velocity loop  (50 Hz)  →  angle setpoint     (rad)
Angle loop    (500 Hz)  →  rate setpoint      (rad/s)
Rate loop       (2 kHz) →  motor output       (DSHOT)
```

The rate loop runs at 2 kHz using **raw IMU gyro** for minimum latency. The angle loop runs at 500 Hz using the **EKF quaternion→Euler** attitude. The outer loops (altitude, position) run at lower rates using EKF NED state.

---

## EKF State → PID Mapping

```
EKF output              Used by
──────────────────────────────────────────────────
q0,q1,q2,q3 → roll, pitch, yaw    Angle loop
vel_n, vel_e           Position/velocity loop
vel_d                  Altitude velocity loop
pos_n, pos_e           Position hold
pos_d                  Altitude hold
bax,bay,baz            Subtracted from IMU in predict step
bgx,bgy,bgz            Subtracted from IMU in predict step
```

---

## DSHOT600 PIO Program

```
; PIO clock ≈ 9.615 MHz (150 MHz / 15.625)
; Bit period = 16 cycles ≈ 1664 ns → 601 kbit/s
; '1': 12 cycles HIGH + 4 cycles LOW
; '0':  6 cycles HIGH + 10 cycles LOW

Addr  Opcode  Mnemonic
 0    0x7421  out x, 1     side 1 [4]   ; shift bit, HIGH, 5 cycles
 1    0x1064  jmp !x, 4   side 1 [0]   ; check bit, HIGH, 1 cycle
 2    0xB542  nop          side 1 [5]   ; stay HIGH (total 12 for '1')
 3    0x0300  jmp 0        side 0 [3]   ; LOW 4 cycles → next bit
 4    0xA942  nop          side 0 [9]   ; LOW 10 cycles ('0' path)
              .wrap 4→0
```

Configuration: shift-left, autopull at 16-bit threshold. Host writes DSHOT packet in upper 16 bits of 32-bit word. SM stalls between packets, creating natural inter-frame gap.

### Motor Mixer (Quad X)

```
       FRONT
  M1(CW)  M2(CCW)
      ╲    ╱
      ╱    ╲
  M3(CCW) M4(CW)
       REAR

M1 = thr + roll - pitch - yaw
M2 = thr - roll - pitch + yaw
M3 = thr - roll + pitch - yaw
M4 = thr + roll + pitch + yaw
```

Desaturation: if any motor exceeds 1.0, all are scaled down. Airmode: if any motor goes negative, all are boosted to maintain attitude control authority at zero throttle.

---

## Flight Modes

| Mode | AUX2 range | Rate | Angle | Altitude | Position |
|------|-----------|------|-------|----------|----------|
| MANUAL | < 600 | RC direct | — | — | — |
| STABILIZE | 600–1000 | angle→rate | RC | — | — |
| ALT_HOLD | 1000–1400 | angle→rate | RC | EKF+baro | — |
| POS_HOLD | > 1400 | angle→rate | vel→angle | EKF+baro | EKF+GPS |
| FAILSAFE | auto | zero | level | descend | — |

### Arming Sequence

1. Switch AUX1 to low (disarm)
2. Power on — system boots DISARMED
3. Wait for GPS fix (LED blinks slowly)
4. LED solid = system ready
5. Set throttle to minimum
6. Switch AUX1 high → **ARMED** (mode determined by AUX2)
7. Arm switch low → DISARMED immediately

### Failsafe

If no CRSF packet for `RC_FAILSAFE_MS` (500 ms), the system:
1. Enters FAILSAFE mode
2. Commands level attitude, zero yaw rate
3. Descends at 0.5 m/s
4. Disarms after landing (throttle idle timeout)

---

## Cross-Core Safety Note

The `CrsfReceiver` object lives on Core 1. It is **not safe** to call its methods from Core 0 directly. All RC data is transferred via `g_shared.crsf` (seqlock). The `FCS_NAV.ino` uses `update_fc_from_shared_rc()` which reads from shared memory only.

**Recommended production refactor:** Extract a `RcInput` interface from `CrsfReceiver` that operates on a plain `int16_t[16]` channel array. `FlightController::update_from_rc()` should accept this interface rather than `CrsfReceiver&`.

---

## File Structure

```
FCS_NAV/
├── FCS_NAV.ino           Main sketch — Core 0 nav+control, Core 1 I/O
├── config.h              All pins, timing, PID gains, EKF tuning
├── pid.h                 PID controller (I-term relax, TPA, D-LPF, airmode)
├── dshot.h/.cpp          DSHOT600 via PIO (pre-assembled opcodes, 4× SM)
├── crsf.h/.cpp           CRSF/ExpressLRS parser (CRC-8/DVB-S2, 16 channels)
├── flight_control.h/.cpp Flight mode FSM, PID cascade, X-frame mixer
├── shared_state.h        Seqlock inter-core data (GPS/Baro/Mag/CRSF/EKF)
│
│   ── From INS_EKF_v2 (unchanged) ──
├── ekf_ins.h/.cpp        16-state quaternion EKF, Joseph-form covariance
├── matrix_math.h/.cpp    Fixed-size matrix library
├── bmi088_spi.h/.cpp     BMI088 SPI driver
├── bmm350.h/.cpp         BMM350 I2C magnetometer driver
├── bmp580.h/.cpp         BMP580 I2C barometer driver
└── gps_parser.h/.cpp     NMEA 0183 parser (RMC + GGA)
```

---

## Timing Budget (Core 0 @ 2 kHz)

| Operation | Rate | Est. time |
|-----------|------|-----------|
| BMI088 SPI read | 2 kHz | ~20 µs |
| Rate PID (3 axis) | 2 kHz | ~5 µs |
| DSHOT write (4 SM) | 2 kHz | ~5 µs |
| EKF predict | 500 Hz | ~150 µs |
| Angle PID | 500 Hz | ~3 µs |
| EKF mag update | 100 Hz | ~40 µs |
| EKF baro update | 50 Hz | ~15 µs |
| Altitude PID | 50 Hz | ~3 µs |
| EKF GPS update | 10 Hz | ~80 µs |
| Position PID | 10 Hz | ~5 µs |
| **Total per 500 µs tick** | | **~225 µs (45% load)** |

Measured on RP2350 @ 150 MHz. The M33 FPU executes most matrix ops at ~1 FLOP/cycle.

---

## Tuning Order

1. **Fly rate mode** (MANUAL) first. Tune `PID_RATE_*` until hover is stable.
2. **Test angle mode** (STABILIZE). Tune `PID_ANGLE_*_P`.
3. **Enable alt hold** (ALT_HOLD). Tune `PID_ALT_P/I/D` and `HOVER_THROTTLE`.
4. **Enable pos hold** (POS_HOLD). Tune `PID_VEL_*_P/I`.
5. **Adjust EKF Q/R** based on observed position noise and lag.
