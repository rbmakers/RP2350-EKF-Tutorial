
# Kalman Filter & Extended Kalman Filter — A Complete Tutorial

> **Target audience:** Engineers and students with basic linear algebra background.  
> **Goal:** Build solid intuition for the statistics behind KF/EKF, then connect every equation to a physical meaning.

---

## Table of Contents

1. [Statistical Foundations](#1-statistical-foundations)
   - 1.1 Variance and Standard Deviation
   - 1.2 Covariance and the Covariance Matrix
   - 1.3 The Inverse Covariance (Precision) Matrix
   - 1.4 Gaussian (Normal) Distributions
   - 1.5 Mahalanobis Distance
2. [Kalman Filter (KF)](#2-kalman-filter-kf)
   - 2.1 The Core Problem
   - 2.2 System Model
   - 2.3 Matrix Glossary
   - 2.4 The Two-Step Algorithm
   - 2.5 Kalman Gain — The Key Insight
   - 2.6 Worked Example (1D)
3. [Extended Kalman Filter (EKF)](#3-extended-kalman-filter-ekf)
   - 3.1 Why KF Is Not Enough
   - 3.2 Linearisation via Jacobians
   - 3.3 EKF Algorithm
   - 3.4 KF vs EKF Comparison
4. [Matrix Reference Cheat Sheet](#4-matrix-reference-cheat-sheet)
5. [Tuning Guidelines](#5-tuning-guidelines)
6. [Application: Drone / INS Sensor Fusion](#6-application-drone--ins-sensor-fusion)
7. [Intuition Summary](#7-intuition-summary)

---

## 1. Statistical Foundations

Before touching filter equations, it is essential to understand the statistics that underpin every matrix in a Kalman Filter.

---

### 1.1 Variance and Standard Deviation

**Variance** σ² measures how spread out a set of measurements is around the mean:

```
σ² = E[ (x − μ)² ]
```

| Quantity | Symbol | Meaning |
|----------|--------|---------|
| Mean | μ | Expected (average) value |
| Variance | σ² | Average squared deviation from the mean |
| Standard deviation | σ | Square root of variance; same units as x |

**Intuition:** If a GPS reports position with σ = 2 m, roughly 68% of readings fall within ±2 m of truth.

---

### 1.2 Covariance and the Covariance Matrix

**Covariance** between two scalar variables x₁ and x₂:

```
cov(x₁, x₂) = E[ (x₁ − μ₁)(x₂ − μ₂) ]
```

- Positive: both tend to increase together
- Negative: one increases while the other decreases
- Zero: no linear relationship

For a state vector **x** = [x₁, x₂, …, xₙ]ᵀ, the **covariance matrix** Σ collects all pairwise covariances:

```
         ┌ σ₁²        cov(x₁,x₂)  ···  cov(x₁,xₙ) ┐
Σ  =     │ cov(x₂,x₁) σ₂²         ···  cov(x₂,xₙ) │
         │      ⋮                  ⋱        ⋮   │
         └ cov(xₙ,x₁) cov(xₙ,x₂) ···  σₙ²         ┘
```

**Key properties:**
- Σ is always **symmetric**: cov(xᵢ, xⱼ) = cov(xⱼ, xᵢ)
- Σ is always **positive semi-definite**: all eigenvalues ≥ 0
- Diagonal elements = individual variances
- Off-diagonal elements = how variables co-vary

**Intuition:** Σ is a complete map of *uncertainty* in your state estimate.

---

### 1.3 The Inverse Covariance (Precision) Matrix

The **precision matrix** (or **information matrix**) is defined as:

```
Λ = Σ⁻¹
```

| Concept | Matrix | Meaning |
|---------|--------|---------|
| Uncertainty | Σ | "How wrong could I be?" |
| Precision | Σ⁻¹ | "How confident am I?" |

**Scalar example:**

```
σ² = 4  →  Σ⁻¹ = 1/4 = 0.25   (low precision, high uncertainty)
σ² = 0.01 →  Σ⁻¹ = 100          (high precision, low uncertainty)
```

**Key properties of Λ = Σ⁻¹:**

1. **Encodes confidence, not spread** — large diagonal entry = tightly constrained variable
2. **Reveals conditional independence** — if Λᵢⱼ = 0, then xᵢ and xⱼ are *conditionally independent* given all other variables
3. **Natural weighting in fusion** — in sensor fusion, more precise sensors receive higher weight proportional to Σ⁻¹

**Why this matters for filtering:**

The Kalman Gain formula contains `(HPHᵀ + R)⁻¹`, which is precisely an inverse covariance — it weights the correction by how reliable the measurement is relative to the prediction.

---

### 1.4 Gaussian (Normal) Distributions

The Kalman Filter is the **optimal estimator** when noise is Gaussian. A multivariate Gaussian is:

```
p(x) ∝ exp( −½ (x − μ)ᵀ Σ⁻¹ (x − μ) )
```

Notice:
- The **covariance Σ** shapes the distribution (wide = uncertain)
- The **precision Σ⁻¹** appears in the exponent — it directly weights the "distance" from the mean

**Key property:** The product of two Gaussians is also a Gaussian. This is why combining a prediction (one Gaussian) with a measurement (another Gaussian) yields a new, sharper Gaussian — the foundation of KF.

---

### 1.5 Mahalanobis Distance

The term inside the Gaussian exponent is called the **Mahalanobis distance**:

```
d²_M = (x − μ)ᵀ Σ⁻¹ (x − μ)
```

Compare with ordinary Euclidean distance:
```
d²_E = (x − μ)ᵀ I (x − μ)      ← treats all directions equally
d²_M = (x − μ)ᵀ Σ⁻¹ (x − μ)   ← scales by uncertainty
```

**Intuition:**
- A 5 m GPS error is *small* if GPS variance is 25 m² (Mahalanobis ≈ 1.0)
- A 5 m GPS error is *large* if GPS variance is 0.01 m² (Mahalanobis = 500)

The Mahalanobis distance normalises deviations by the expected spread — used in the KF innovation step.

---

## 2. Kalman Filter (KF)

### 2.1 The Core Problem

You want to know the **true state** of a system (position, velocity, attitude…), but you only have:
- A **noisy model** of how the system evolves
- **Noisy sensor measurements**

The Kalman Filter finds the **optimal minimum-variance estimate** by fusing both sources, weighting each by its uncertainty.

---

### 2.2 System Model

The KF assumes:

**State transition (process) model:**
```
xₖ = A xₖ₋₁ + B uₖ + wₖ       wₖ ~ N(0, Q)
```

**Measurement model:**
```
zₖ = H xₖ + vₖ                  vₖ ~ N(0, R)
```

Where:
- **xₖ** — state vector at time k (what we want to know)
- **zₖ** — measurement vector (what the sensor reports)
- **uₖ** — control input (e.g., commanded thrust)
- **wₖ** — process noise (model imperfection)
- **vₖ** — measurement noise (sensor imperfection)

---

### 2.3 Matrix Glossary

| Matrix | Name | Size | Meaning | How to Obtain |
|--------|------|------|---------|---------------|
| **A** | State transition | n×n | How state evolves from k−1 to k | Physics / kinematics model |
| **B** | Control input | n×m | Maps control input u to state change | System input model |
| **Q** | Process noise covariance | n×n | Uncertainty in the model | IMU noise density; tuned |
| **H** | Measurement matrix | p×n | Maps state space → measurement space | Sensor observation model |
| **R** | Measurement noise covariance | p×p | Sensor uncertainty | Sensor datasheet / calibration |
| **P** | State (error) covariance | n×n | Current estimate uncertainty | Initialised; propagated by filter |
| **K** | Kalman gain | n×p | Optimal blending weight | Computed each timestep |

*(n = state dimension, m = control dimension, p = measurement dimension)*

---

### 2.4 The Two-Step Algorithm

The filter cycles between **Predict** and **Update** at every timestep.

#### Step 1 — Predict

Propagate state forward using the model:

```
x̂ₖ⁻ = A x̂ₖ₋₁ + B uₖ          ← predicted state (a priori)
Pₖ⁻  = A Pₖ₋₁ Aᵀ + Q           ← predicted covariance
```

- `Aᵀ` rotates the covariance ellipsoid through the model transformation
- `Q` is *added* because model errors grow uncertainty

#### Step 2 — Update (Correct)

Fuse the prediction with the new measurement:

```
Innovation:       yₖ = zₖ − H x̂ₖ⁻           ← residual (how surprised we are)
Innovation cov:   Sₖ = H Pₖ⁻ Hᵀ + R          ← total uncertainty in measurement space
Kalman gain:      Kₖ = Pₖ⁻ Hᵀ Sₖ⁻¹           ← optimal weight
State update:     x̂ₖ = x̂ₖ⁻ + Kₖ yₖ          ← corrected state (a posteriori)
Cov update:       Pₖ = (I − Kₖ H) Pₖ⁻         ← corrected covariance
```

**Numerical tip:** The *Joseph form* is more numerically stable for the covariance update:
```
Pₖ = (I − Kₖ H) Pₖ⁻ (I − Kₖ H)ᵀ + Kₖ R Kₖᵀ
```

---

### 2.5 Kalman Gain — The Key Insight

```
Kₖ = Pₖ⁻ Hᵀ (H Pₖ⁻ Hᵀ + R)⁻¹
```

Think of K as a *dial* between two extremes:

```
         K → 0                           K → H⁻¹
    (ignore sensor)                  (trust sensor fully)

         ↑                                 ↑
    R is large                        R is small
  (noisy sensor)                  (precise sensor)
  or Pₖ⁻ is small              or Pₖ⁻ is large
  (model confident)              (model uncertain)
```

| Condition | Kalman Gain | Effect |
|-----------|-------------|--------|
| R ↓ (precise sensor) | K increases | Larger correction from measurement |
| R ↑ (noisy sensor) | K decreases | Rely more on prediction |
| P ↑ (uncertain model) | K increases | Trust sensor more |
| P ↓ (confident model) | K decreases | Trust model more |

**Fundamental relationship:** K is a ratio of *precisions* — it's the inverse covariance doing its work.

---

### 2.6 Worked Example (1D)

**Scenario:** Estimating position of a robot. One step shown.

**Given:**
- Prior estimate: x̂ = 10 m, P = 4 m²
- Measurement: z = 11 m, R = 1 m²
- No motion (A = 1, Q = 0 for simplicity)

**Predict:**
```
x̂⁻ = 10 m        P⁻ = 4 m²
```

**Update:**
```
y = 11 − 10 = 1 m
S = 4 + 1 = 5 m²
K = 4 / 5 = 0.8
x̂ = 10 + 0.8 × 1 = 10.8 m
P  = (1 − 0.8) × 4 = 0.8 m²
```

**Result:** The filter moved from 10 m toward 11 m (measurement), landing at 10.8 m — weighted toward the more precise measurement. Uncertainty dropped from 4 m² to 0.8 m².

---

## 3. Extended Kalman Filter (EKF)

### 3.1 Why KF Is Not Enough

The KF assumes **linear** system and measurement models. Real systems are almost never linear:

| Real System | Nonlinearity |
|-------------|-------------|
| Drone attitude | Euler angle kinematics, quaternion integration |
| IMU integration | Trigonometric rotation matrices |
| GPS → local frame | Coordinate transformations |
| Barometric altitude | Nonlinear pressure model |

Using KF on these systems would propagate errors incorrectly because `A` would misrepresent the true dynamics.

---

### 3.2 Linearisation via Jacobians

The EKF replaces linear matrices A and H with **local linear approximations** — the Jacobian matrices of the nonlinear functions.

**Nonlinear models:**
```
xₖ = f(xₖ₋₁, uₖ) + wₖ         ← nonlinear process function
zₖ = h(xₖ) + vₖ                ← nonlinear measurement function
```

**Jacobian of f evaluated at current estimate x̂:**
```
        ∂f₁/∂x₁  ∂f₁/∂x₂  ···  ∂f₁/∂xₙ
F  =    ∂f₂/∂x₁  ∂f₂/∂x₂  ···  ∂f₂/∂xₙ      evaluated at x̂ₖ₋₁
            ⋮                        ⋮
        ∂fₙ/∂x₁  ∂fₙ/∂x₂  ···  ∂fₙ/∂xₙ
```

**Jacobian of h:**
```
        ∂h₁/∂x₁  ∂h₁/∂x₂  ···  ∂h₁/∂xₙ
H  =    ∂h₂/∂x₁  ∂h₂/∂x₂  ···  ∂h₂/∂xₙ      evaluated at x̂ₖ⁻
            ⋮                        ⋮
        ∂hₚ/∂x₁  ∂hₚ/∂x₂  ···  ∂hₚ/∂xₙ
```

**Physical meaning:** The Jacobian is the *best linear fit* to the nonlinear function at the current operating point. Errors grow when the state moves far from that point (highly nonlinear regions).

---

### 3.3 EKF Algorithm

#### Predict

```
x̂ₖ⁻ = f(x̂ₖ₋₁, uₖ)              ← propagate using nonlinear function
Pₖ⁻  = Fₖ Pₖ₋₁ Fₖᵀ + Q          ← propagate covariance using Jacobian F
```

#### Update

```
Innovation:       yₖ = zₖ − h(x̂ₖ⁻)            ← nonlinear predicted measurement
Innovation cov:   Sₖ = Hₖ Pₖ⁻ Hₖᵀ + R
Kalman gain:      Kₖ = Pₖ⁻ Hₖᵀ Sₖ⁻¹
State update:     x̂ₖ = x̂ₖ⁻ + Kₖ yₖ
Cov update:       Pₖ = (I − Kₖ Hₖ) Pₖ⁻
```

**Key difference from KF:**
- State propagation uses the true nonlinear `f(·)` and `h(·)`
- Covariance propagation uses the Jacobians `F` and `H`
- Jacobians are **recomputed at every timestep**

---

### 3.4 KF vs EKF Comparison

| Feature | Kalman Filter (KF) | Extended Kalman Filter (EKF) |
|---------|-------------------|------------------------------|
| System model | Linear: xₖ = Axₖ₋₁ | Nonlinear: xₖ = f(xₖ₋₁) |
| Measurement model | Linear: zₖ = Hxₖ | Nonlinear: zₖ = h(xₖ) |
| Covariance propagation | A P Aᵀ + Q | F P Fᵀ + Q (Jacobian F) |
| Innovation | zₖ − H x̂ₖ⁻ | zₖ − h(x̂ₖ⁻) |
| Optimality | Exact (for linear Gaussian) | Approximate (first-order) |
| Computational cost | Low | Moderate (Jacobian computation) |
| Typical applications | Linear tracking, simple 1D systems | Robotics, drones, SLAM, INS |
| Divergence risk | Low | Can diverge for strongly nonlinear systems |

---

## 4. Matrix Reference Cheat Sheet

### Complete Matrix Table

| Matrix | Full Name | Role | Definition / Calculation | Design Source |
|--------|-----------|------|--------------------------|--------------|
| **A** (or **F** in EKF) | State transition | Predicts how state evolves | Physics equations; Jacobian of f in EKF | System dynamics model |
| **B** | Control input matrix | Maps uₖ → state | Derived from system input model | Often omitted if no control |
| **Q** | Process noise covariance | Model uncertainty; uncertainty grows | Derived from noise density; tuned empirically | IMU noise specs; dynamics uncertainty |
| **H** | Measurement matrix | Maps state → sensor space | Sensor observation model; Jacobian of h in EKF | What the sensor physically measures |
| **R** | Measurement noise covariance | Sensor uncertainty | Sensor datasheet; measured variance | Calibration data |
| **P** | State error covariance | Current estimation uncertainty | Initialised large; updated each cycle | Filter propagation |
| **K** | Kalman gain | Fusion weight | K = P⁻ Hᵀ (HP⁻Hᵀ + R)⁻¹ | Computed automatically |
| **S** | Innovation covariance | Total uncertainty in measurement space | S = H P⁻ Hᵀ + R | Intermediate computation |
| **y** | Innovation / residual | How surprising the measurement is | y = z − H x̂⁻ | Used in state update |
| **I** | Identity matrix | Neutral element | Standard n×n identity | Fixed |

---

### Flow Diagram

```
                ┌──────────────────────────────────────┐
                │              PREDICT                 │
                │                                      │
                |  x̂ₖ₋₁, Pₖ₋₁ ──►  x̂ₖ⁻ = A x̂ₖ₋₁ + B uₖ │
                │  Pₖ⁻  = A Pₖ₋₁ Aᵀ + Q                │
                └─────────────────┬────────────────────┘
                                  │  x̂ₖ⁻, Pₖ⁻
                                  ▼
                ┌──────────────────────────────────────┐
                │              UPDATE                  │
                │                                      │
         zₖ ──► │  y = zₖ − H x̂ₖ⁻                      │
                │  S = H Pₖ⁻ Hᵀ + R                    │
                │  K = Pₖ⁻ Hᵀ S⁻¹                      │
                │  x̂ₖ = x̂ₖ⁻ + K y                      │
                │  Pₖ  = (I − KH) Pₖ⁻                  │
                └─────────────────┬────────────────────┘
                                  │  x̂ₖ, Pₖ
                                  ▼
                         (next timestep)
```

---

### Relationships Among Matrices

```
Q  ──────────────────────►  Pₖ⁻  ◄──── A, Pₖ₋₁
                                │
                        ┌───────┘
                        │
           H ──────────►│──────────────► S ◄──── R
                        │                │
                        └──────► K ◄─────┘
                                 │
                     x̂ₖ⁻ ──────►│──────────► x̂ₖ
                         y ──────┘
```

---

## 5. Tuning Guidelines

Tuning Q and R is the primary engineering task when deploying a KF/EKF. P is usually initialised large and converges quickly.

### R — Measurement Noise Covariance

R is the easier matrix to set. Use sensor specifications:

| Sensor | Typical Variance |
|--------|-----------------|
| GPS position | 1–25 m² (depends on signal quality) |
| Barometer altitude | 0.1–1 m² |
| Magnetometer heading | 0.01–0.1 rad² |
| Optical flow | depends on altitude |

**Rule:** Set R from real measurement variance. **Never set R = 0** (singular system).

### Q — Process Noise Covariance

Q is harder. Common approaches:

1. **From sensor noise density:** For an IMU with accelerometer noise density `n` [m/s²/√Hz], over Δt seconds:
   ```
   Q_accel ≈ n² × Δt
   ```

2. **From physical reasoning:** Unmodelled forces (wind, vibration) contribute residual acceleration. Estimate their typical magnitude.

3. **Empirical tuning:** Start with a diagonal Q. Increase diagonal entries if the filter is too slow to track; decrease if output is too noisy.

### P — Initial State Covariance

```
P₀ = diag(σ²_pos, σ²_vel, σ²_att, ...)
```

- **Large P₀** → fast initial convergence, first few estimates may be noisy
- **Small P₀** → slow adaptation, risk of wrong initial lock
- **Typical practice:** P₀ = diagonal with values ≈ initial uncertainty squared

### Tuning Heuristics Summary

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Estimate lags behind truth | Q too small or R too small | Increase Q |
| Estimate is too noisy | Q too large | Decrease Q |
| Sensor ignored | R too large | Decrease R |
| Estimate diverges | Q too small (overconfident model) | Increase Q or re-check A/F |
| Slow convergence | P₀ too small | Initialise P₀ larger |

---

## 6. Application: Drone / INS Sensor Fusion

### Typical State Vector

```
x = [ px, py, pz,        ← position (m)
      vx, vy, vz,        ← velocity (m/s)
      φ, θ, ψ,           ← roll, pitch, yaw (rad)
      bax, bay, baz,     ← accelerometer bias (m/s²)
      bgx, bgy, bgz ]    ← gyroscope bias (rad/s)
```

State dimension n = 15 (typical EKF for a drone INS)

### Sensor Roles

| Sensor | Update Rate | Role in Filter | Notes |
|--------|------------|----------------|-------|
| IMU (accel + gyro) | 500–2000 Hz | **Prediction** step | High rate; integrates attitude & velocity |
| GPS | 1–10 Hz | Position/velocity **update** | Absolute reference; latency matters |
| Barometer | 50–100 Hz | Altitude **update** | Good vertical; drifts with weather |
| Magnetometer | 50–100 Hz | Yaw **update** | Susceptible to magnetic interference |
| Optical flow | 100–400 Hz | Velocity **update** (indoor) | Requires height knowledge |

### Sensor Fusion Logic

```
┌─────────────────────────────────────────────────────────────┐
│  IMU @ 1 kHz                                                │
│  ──────────► EKF Predict every 1 ms                         │
│              (f integrates IMU, F = kinematic Jacobian)     │
└─────────────────────────────┬───────────────────────────────┘
                              │
              ┌───────────────┼───────────────┐
              ▼               ▼               ▼
         GPS @ 5 Hz     Baro @ 50 Hz    Mag @ 50 Hz
         EKF Update     EKF Update      EKF Update
         (pos, vel)     (alt)           (yaw)
```

### Why Inverse Covariance Matters Here

At fusion time, the Kalman Gain effectively computes:

```
K ∝ P⁻¹_prediction × (P_prediction + R_sensor)⁻¹
```

- When the **IMU drifts** (P grows large) → K increases → **GPS correction dominates**
- When **GPS is noisy** (R large) → K decreases → **IMU dead reckoning trusted more**
- The filter autonomously adjusts trust based on the ratio of uncertainties

This is the inverse covariance working as a *reliability weight* in real time.

---

## 7. Intuition Summary

### The Big Picture

```
╔══════════════════════════════════════════════════════════════╗
║                  WHAT IS A KALMAN FILTER?                    ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║   You have TWO sources of information:                       ║
║                                                              ║
║   1. A MODEL: "Physics says I should be here"                ║
║      → Encoded in A (or f), with uncertainty Q               ║
║                                                              ║
║   2. A SENSOR: "My instrument reads this"                    ║
║      → Encoded in H (or h), with uncertainty R               ║
║                                                              ║
║   The KF asks: "Given both, what is my BEST estimate?"       ║
║                                                              ║
║   Answer: Weight each source by its PRECISION (Σ⁻¹)          ║
║   The more precise source receives more weight.              ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
```

### One-Sentence Meanings

| Matrix | One-sentence meaning |
|--------|----------------------|
| **A / F** | "This is how I think the world moves" |
| **B** | "This is how my commands affect the world" |
| **Q** | "This is how wrong my motion model might be" |
| **H** | "This is what my sensor actually measures" |
| **R** | "This is how noisy my sensor is" |
| **P** | "This is how uncertain I am right now" |
| **K** | "This decides which source to believe more" |
| **Σ⁻¹** | "This is how confidently I weight information" |

### Memory Anchor

```
PREDICT:    A, B  →  where are we going?
            Q     →  how much do we doubt the model?

UPDATE:     H     →  what does the sensor see?
            R     →  how much do we doubt the sensor?
            P     →  how uncertain are we right now?
            K     →  the judge: model vs sensor
```

---

*Document prepared as an engineering tutorial for KF/EKF applications in embedded systems and drone sensor fusion.*

*References: Welch & Bishop (2006), "An Introduction to the Kalman Filter"; Maybeck (1979), "Stochastic Models, Estimation and Control"; PX4 EKF2 source documentation.*
