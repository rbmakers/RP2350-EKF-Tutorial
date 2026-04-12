# Advanced GNSS+INS Navigation System — RP2350

**16-state Quaternion EKF with Joseph-form covariance update**

Sensors: BMI088 (SPI) · BMM350 (I2C) · BMP580 (I2C) · GNSS UART  
Target:  RP2350 (earlephilhower/arduino-pico board package)

---

## File Structure

```
INS_EKF_v2/
├── INS_EKF_v2.ino      Main sketch — dual-core setup/loop
├── config.h            ALL pin assignments, timing, and EKF tuning constants
├── matrix_math.h/.cpp  Fixed-size float matrix library (no heap)
├── bmi088_spi.h/.cpp   BMI088 accel+gyro SPI driver (6 hardware quirks handled)
├── bmm350.h/.cpp       BMM350 magnetometer I2C driver (OTP trim, normal mode)
├── bmp580.h/.cpp       BMP580 barometer I2C driver (20-bit, IIR filter)
├── gps_parser.h/.cpp   NMEA 0183 parser — RMC + GGA sentences
├── shared_state.h      Seqlock dual-core shared memory (DMB SY barriers)
└── ekf_ins.h/.cpp      16-state quaternion EKF — predict + 3 update channels
```

---

## Hardware Connections

| Signal          | RP2350 Pin | Sensor       |
|-----------------|-----------|--------------|
| SPI SCK         | GP2       | BMI088       |
| SPI MOSI        | GP3       | BMI088       |
| SPI MISO        | GP4       | BMI088       |
| ACC CS          | GP5       | BMI088 accel |
| GYR CS          | GP6       | BMI088 gyro  |
| I2C SDA         | GP20      | BMM350, BMP580 |
| I2C SCL         | GP21      | BMM350, BMP580 |
| UART1 TX → GPS  | GP0       | GPS receiver |
| UART1 RX ← GPS  | GP1       | GPS receiver |

BMM350 and BMP580 share the same I2C bus. Default addresses:
- BMM350: `0x14` (SDO = GND)
- BMP580: `0x46` (SDO = GND)

---

## EKF State Vector

```
x[16] = [ px, py, pz,           NED position   (m)
           vx, vy, vz,           NED velocity   (m/s)
           q0, q1, q2, q3,       Quaternion     (Hamilton, scalar-first)
           bax, bay, baz,        Accel bias     (m/s²)
           bgx, bgy, bgz ]       Gyro bias      (rad/s)
```

### Why Quaternion?

| Issue with Euler | Quaternion solution |
|-----------------|---------------------|
| Gimbal lock at pitch = ±90° | No singularity at any attitude |
| Discontinuity at ±180° yaw | Smooth, continuous |
| Jacobian ill-conditioned near singularity | Well-conditioned everywhere |
| Normalisation drift | Single `quat_normalise()` call |

---

## Dual-Core Architecture

```
Core 0 (Navigation — time-critical)      Core 1 (Peripherals — I/O)
────────────────────────────────────     ────────────────────────────
500 Hz gate (micros())                   GPS UART parse (as fast as bytes arrive)
BMI088 SPI read                          BMP580 I2C read @ 50 Hz
EKF predict (every cycle)                BMM350 I2C read @ 100 Hz
EKF update (GPS/Baro/Mag if new)         Write all data → shared_state (seqlock)
Write EKF output → shared_state
Serial telemetry @ 10 Hz
```

### Seqlock Protocol

All inter-core data uses a seqlock with `DMB SY` memory barriers:

```cpp
// Writer (Core 1):
shared.seq++;          // → odd  (write in progress)
__asm__("dmb sy");
shared.data = new_val;
__asm__("dmb sy");
shared.seq++;          // → even (write complete)

// Reader (Core 0):
do {
    s1 = shared.seq;  dmb_sy();
    val = shared.data;
    dmb_sy();         s2 = shared.seq;
} while ((s1 & 1) || s1 != s2);
```

The RP2350 Cortex-M33 has a weakly-ordered memory model. Without `DMB SY`,
the hardware or compiler may reorder loads/stores across the sequence counter,
silently corrupting the shared data read.

---

## EKF Mathematical Summary

### Process Model (continuous strapdown INS)

```
ṗ = v
v̇ = C_bn(q) · (f_imu − bₐ)  +  g_ned        g_ned = [0, 0, +9.80665]
q̇ = ½ · Ω(ω̃) · q                              ω̃ = ω_imu − bᵍ
ḃₐ = 0   (random walk)
ḃᵍ = 0   (random walk)
```

### Quaternion Integration (exact rotation vector method)

```
δq = [cos(|ω|dt/2),  ω̂·sin(|ω|dt/2)]
q_new = δq ⊗ q_old        (Hamilton product)
```

This is exact for constant angular velocity over `dt`, compared to the
first-order `q += q̇·dt` which accumulates normalisation error.

### C_bn from Unit Quaternion q = [q0, q1, q2, q3]

```
C_bn = ⎡ q0²+q1²−q2²−q3²    2(q1q2−q0q3)       2(q1q3+q0q2)    ⎤
       ⎢ 2(q1q2+q0q3)        q0²−q1²+q2²−q3²    2(q2q3−q0q1)    ⎥
       ⎣ 2(q1q3−q0q2)        2(q2q3+q0q1)        q0²−q1²−q2²+q3²⎦
```

### Jacobian F (16×16 non-zero blocks)

```
Block           Size    Content
(0:3, 3:6)      3×3     I·dt                   dp/dv
(3:6, 6:10)     3×4     ∂(Cbn·f̃)/∂q · dt       dv/dq  (analytical)
(3:6, 10:13)    3×3     −Cbn · dt               dv/dbₐ
(6:10, 6:10)    4×4     I + ½·Ω(ω̃)·dt          dq/dq
(6:10, 13:16)   4×3     −½·Ξ(q) · dt            dq/dbᵍ
```

### Ω and Ξ matrices

```
Ω(ω̃) = ⎡  0  −wx −wy −wz ⎤       Ξ(q) = ⎡ −q1  −q2  −q3 ⎤
        ⎢ wx    0  wz −wy ⎥              ⎢  q0  −q3   q2 ⎥
        ⎢ wy  −wz   0   wx ⎥              ⎢  q3   q0  −q1 ⎥
        ⎣ wz   wy −wx    0 ⎦              ⎣ −q2   q1   q0 ⎦
```

### Measurement Models

| Source  | p | z                    | Model         | H                          |
|---------|---|----------------------|---------------|-----------------------------|
| GPS     | 6 | [pN,pE,pD,vN,vE,vD]  | Linear        | Fixed 6×16, identity blocks |
| Baro    | 1 | pD = origin_alt − h  | Linear        | Fixed 1×16, H[IDX_PZ]=1    |
| Mag     | 3 | [bx,by,bz]_body_norm | **Nonlinear** | Jacobian ∂h/∂q computed each call |

Magnetometer predicted measurement:
```
h(q) = C_bn^T(q) · m̂_ned     (rotate NED reference field to body frame)
```

### Joseph-Form Covariance Update

```
Standard form:    P⁺ =  (I−KH) · P⁻
Joseph form:      P⁺ =  (I−KH) · P⁻ · (I−KH)ᵀ  +  K · R · Kᵀ
```

**Why Joseph form matters:**

| Property        | Standard | Joseph |
|-----------------|----------|--------|
| Symmetric P⁺    | Not guaranteed (numerical drift) | Guaranteed by construction |
| PSD P⁺          | Can go negative (filter divergence) | Guaranteed by construction |
| Sub-optimal K   | P⁺ incorrect | Correct for any K |
| Extra cost      | —        | ~2× covariance update ops |

The extra cost at 500 Hz is justified: the 16×16 products take ~110 µs total,
leaving ~1.9 ms of the 2 ms budget for state integration and sensor I/O.

---

## Sensor Update Rates and Timing Budget

```
Operation                  Rate      Core   Approx time
─────────────────────────────────────────────────────────
BMI088 SPI read            500 Hz    0      ~20 µs
EKF predict (state + P)    500 Hz    0      ~150 µs
EKF GPS update (6-state)    5 Hz     0      ~80 µs
EKF Baro update (1-state)  50 Hz     0      ~15 µs
EKF Mag update (3-state)  100 Hz     0      ~40 µs
GPS UART parse             ~5 Hz     1      non-blocking
BMP580 I2C read            50 Hz     1      ~300 µs (I2C)
BMM350 I2C read           100 Hz     1      ~300 µs (I2C)
─────────────────────────────────────────────────────────
Total Core 0 budget @ 500 Hz:   2000 µs     ~300 µs used (15%)
```

---

## Tuning Guide

Edit `config.h`. Start with defaults, then adjust.

### Q — Process Noise

```cpp
#define Q_VEL    0.1f    // Too small → lags GPS. Too large → noisy velocity.
#define Q_QUAT   5e-4f   // Too small → attitude frozen. Too large → jerky.
#define Q_BIAS_A 1e-6f   // Match IMU accel bias instability spec (datasheet).
#define Q_BIAS_G 1e-8f   // Match IMU gyro  bias instability spec.
```

### R — Measurement Noise

```cpp
#define R_GPS_POS_H  4.0f    // Set from GPS horizontal accuracy spec (CEP² / 2).
#define R_GPS_POS_V  9.0f    // Vertical accuracy is typically 1.5× horizontal.
#define R_BARO_ALT   0.25f   // BMP580 altitude noise ≈ 0.5 m 1-sigma (after IIR-4).
#define R_MAG        0.04f   // Normalised field noise; increase if local interference.
```

### Diagnostic: Filter Health

The main sketch monitors `ekf.sigma_pos_n()` and `mat_is_pd(_P, EKF_N)`.
If P goes non-positive-definite, the filter has diverged. Common causes:
- `Q` too small (overconfident model) → prediction covariance collapses
- `R_GPS` too small (trust GPS completely) → correction step corrupts P
- Wrong initial attitude (large `P0_QUAT` helps recover)

---

## Magnetic Reference Field (NED)

**Required for magnetometer update.** Update `g_mag_ned[]` in `INS_EKF_v2.ino`
for your location:

```
https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
```

The vector must be normalised (unit length). Example values:

| Location         | m_ned (N, E, D)                  |
|------------------|----------------------------------|
| Taiwan (25°N)    | [0.827, −0.065, 0.559]           |
| UK (51°N)        | [0.676, −0.017, 0.737]           |
| Sydney (34°S)    | [0.656,  0.234, −0.717]          |
| Tokyo (35°N)     | [0.651, −0.068, 0.756]           |

Hard-iron offsets (from ferromagnetic parts on your PCB) must be calibrated
and set via `config.h` or `mag.set_hard_iron(ox, oy, oz)`.

---

## Known Limitations and Future Work

1. **Flat-earth LLA→NED**: `lla_to_ned()` uses a spherical-earth approximation
   valid within ~10 km of the origin. For long-range missions, replace with
   a full WGS-84 ECEF→NED transformation.

2. **Quaternion covariance projection**: After normalisation, the quaternion
   constraint `|q|=1` is enforced on the state but not fully reflected in P.
   A rigorous approach projects P onto the constraint manifold. The
   `normalise_quaternion_state()` function applies a simplified correction.

3. **Euler-method state integration**: The predict step uses forward Euler
   for position and velocity. For high-bandwidth manoeuvres, a 4th-order
   Runge-Kutta integrator would reduce state propagation error. The quaternion
   itself uses the exact rotation vector method.

4. **No outlier rejection**: A chi-squared innovation gate
   (`yᵀ·S⁻¹·y < χ²_threshold`) should be applied before each update to
   reject multipath GPS, magnetic anomalies, or pressure spikes.

5. **Barometer temperature compensation**: BMP580 provides temperature; a
   Gauss-Markov model for thermal drift can be added to the state vector.

6. **Soft-iron calibration**: The magnetometer update uses a normalised scalar
   model. A full 3×3 soft-iron correction matrix can be pre-applied to the
   body-frame field before calling `update_mag()`.
