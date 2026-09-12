# RocketNav — Estimator Design

## State vector and error state

Nominal state (mechanized): position `p` (NED, m), velocity `v` (NED, m/s), attitude quaternion `q` (body→NED, Hamilton, scalar-first), gyro bias `bg` (rad/s, body), accel bias `ba` (m/s², body), baro altitude bias `bb` (m). The NED frame is a WGS-84 tangent frame anchored at the first valid GNSS fix; output is converted back to geodetic (lat/lon as int32 1e-7 deg internally — float32 cannot represent that resolution at mid-latitudes).

Error state (16): `δx = [δp δv δθ δbg δba δbb]`. `δθ` is a **global (NED-frame)** rotation-vector error: `q_true = δq(δθ) ⊗ q̂`, i.e. `R_true = (I+[δθ]×) R̂`. Injection: `q̂ ← δq(δθ̂) ⊗ q̂` (left multiply), then the error resets to zero with the covariance reset Jacobian `G_θ = I + ½[δθ̂]×` applied to the attitude block. Global error is chosen because (a) heading error is exactly `δθ_D`, giving the magnetometer a `[0 0 1]` row that structurally cannot touch roll/pitch, and (b) it makes `F` nilpotent (below).

## Process model and Jacobian

Mechanization per IMU sample (`ω = ω_m − bg`, `f = a_m − ba`, `R = R(q̂)`):
`ṗ = v`, `v̇ = R f + g`, `q̇ = ½ q ⊗ [0, ω]`, biases constant + random walk. `g = [0 0 +g₀]` with `g₀` from WGS-84 Somigliana gravity at the anchor latitude/height (0.006 m/s² site error double-integrates to meters over a flight).

Error dynamics (continuous):

```
δṗ = δv
δv̇ = −[Rf]× δθ − R δba − R n_a
δθ̇ = −R δbg − R n_g
δḃg = n_wg ,  δḃa = n_wa ,  δḃb = n_wb
```

The dependency chain is `p ← v ← (θ, ba)`, `θ ← bg`, no cycles, so `F⁴ = 0` and the discrete transition matrix is the **exact** matrix exponential in closed form (`A = −[Rf]×`, `B = −R`):

```
Φ = I +  [p←v] I·dt            [v←θ] A·dt        [v←ba] B·dt      [θ←bg] B·dt
      +  [p←θ] A·dt²/2         [p←ba] B·dt²/2    [v←bg] (A·B)·dt²/2
      +  [p←bg] (A·B)·dt³/6
```

No truncation error is introduced at any rotation rate; the residual is only the zero-order hold of `f` and `R` across one 2.5 ms step. `P ← Φ P Φᵀ + Q_d` runs at the full IMU rate (dense 16×16, ~3% CPU at 216 MHz).

## Discrete process noise

Continuous PSDs → first-order discrete diagonal (`Q_d = G Q_c Gᵀ dt`; rotation of isotropic noise by `R` is identity-preserving, and Van Loan cross terms are O(dt²) — negligible at 400 Hz):

| Block | Q_d entry | Source |
|---|---|---|
| δv | (σ_a·k_vib)²·dt | ICM-45686 DS-000489 Table 2: 110 µg/√Hz at ±32 g |
| δθ | (σ_g·k_vib)²·dt | DS-000489 Table 1: 0.0038 dps/√Hz |
| δbg | σ_wg²·dt | derived from ZRO tempco ±0.005 dps/°C (Table 1) |
| δba | σ_wa²·dt | derived from zero-g tempco ±0.15 mg/°C (Table 2) |
| δbb | σ_wb²·dt | ~1.8 m/h weather drift |

`k_vib` inflates the bench densities for unmodeled flight vibration; accel saturation multiplies the δv term by `SAT_Q_MULT²` for that step (graceful degradation, not divergence). A tiny δp floor keeps P numerically healthy.

## Measurement models

All updates: Joseph form `P ← (I−KH)P(I−KH)ᵀ + KRKᵀ`, explicit symmetrization, NIS `ν'S⁻¹ν` gated against χ² (99%): 11.34 (3 dof), 9.21 (2), 6.63 (1). float32 throughout.

**GNSS position** (NAV-PVT geodetic → NED): `ν = z − (p̂ − v̂τ)`, `H = [I 0…]`. τ is the fix latency at I2C read time (no PPS wired — TIMEPULSE only drives an LED — so latency is modeled explicitly as a constant and the prediction is back-propagated by it). `R = diag(hAcc², hAcc²)` floored at 0.5 m — **horizontal only (2-dof)**: the vertical channel is baro+IMU exclusively, GNSS altitude is logged raw but never fused. **GNSS velocity**: `ν = z − (v̂ − â_ned·τ)` on N/E, `H` on δv_N/δv_E, `R = sAcc²·I₂` floored at 0.1 m/s. After an outage > 2 s, the next 3 fixes carry `R×25` and must pass their gates before normal weighting resumes (reacquisition gating); dead reckoning through the outage is the propagation itself with its honest covariance growth.

**Barometer**: `z = h_baro(P; P₀,T₀)` (hypsometric with the alignment-averaged reference — temperature dependence handled by construction), model `h = −p_D + bb`, so `H` has −1 at `p_D`, +1 at `bb`. `R = σ_base² + (k_dyn·|v̂|²)² + A·exp(−(M−1)²/2σ_M²)` where `M = |v̂|/√(γR_air·T)` — dynamic-pressure and transonic static-port error as smooth state-driven R inflation; rejection is only ever by innovation statistics. No flight-phase logic exists in this path. The sample is `baro_latency_s` (~20 ms of conversion window + register hold) old and is compared against the state back-propagated by that latency, mirroring the GNSS compensation. The vertical channel does not wait for GNSS: `p_D`/`v_D` mechanize and take baro updates from the end of alignment (FS_WAIT_FIX and FS_ATT_ONLY); horizontal states stay pinned until the anchor, and unaided p/v covariance is capped by PSD-preserving row/column scaling. The anchor initializes ONLY the horizontal states: `p_D`/`v_D`/`bb` continue untouched, so altitude is continuous through the anchor by construction, and `vd_off` (p_D at anchor) maps the pad datum into the origin frame for geodetic output. With no second vertical reference, `bb` is pinned near zero — the baro itself defines the vertical datum, and port/weather errors are covered by the R model rather than split into an unobservable bias.

**Magnetometer** (`MAG_CONTINUOUS`): the **full 3-axis vector update** `ν = R̂ m_body − B_ref`. With the global attitude error, `R̂ m_body = (I − [δθ]×) B_ref`, so `H = [B_ref]×` on δθ — constant, rank 2: it constrains every attitude axis except rotation about the field line, the exact vector twin of the gravity tilt update (whose null space is rotation about Down). Since the field and gravity are not collinear, the pair fully determines attitude; critically, the mag pins roll/pitch even while the vehicle is handled, when the stillness-gated gravity/ZARU aids are off. `R = σ_m² I₃` per axis. Field-magnitude (±15%) and inclination (±10°) gates run first, 3-dof χ² (11.34) after — a direction-distorted field must slip past all three to touch attitude. **B_ref, the gates' yardstick and the innovation reference, is captured at alignment**: when the mag seeded the heading, the align-window field mean expressed in the just-initialized frame becomes the run reference (its azimuth equals the configured declination by construction; magnitude and inclination are the measured local values), so in an iron-distorted environment the vector update holds attitude to the boot-consistent field instead of being rejected against — or fighting toward — a site model the local field never matches. Without a mag-seeded init the configured site vector stands. Two boot-repeatability rules feed this: the align mag mean is restricted to the same window that passed the stillness screen (samples taken while the vehicle was still being handled used to survive window restarts and seeded a different heading every boot), and the `$magcal` hard-iron result persists in the F722's last flash sector (RAM-only calibration silently vanished at reset — the single largest boot-to-boot heading delta). The estimator itself is a least-squares sphere fit (`magfit.h`): min/max midpoints were abandoned because one glitched sample corrupts a midpoint permanently and, worse, an axis never tumbled through the field keeps min ≈ max, so the Earth field itself (47.8 µT vertical here) leaked into that axis's "offset" — a flat tabletop spin poisoned z by ~48 µT. The fit averages over every sample, ignores out-of-range glitches, and axes whose observed swing stays under 30 µT hold their previous offset and are flagged to the ground station. Residual heading error after a good hard-iron fit is the (unmodeled, identity) soft-iron term — the next lever if it matters. A `$cal` re-align recaptures the reference, e.g. after moving from bench to pad. Heading = body-roll for a near-vertical vehicle stays the channel's flagship contribution (unobservable to GNSS); the scalar-bearing information of the old formulation is contained in the vector form. `MAG_INIT_ONLY` stops updating at origin anchor; `MAG_OFF` initializes heading 0 with 1 rad σ and the heading covariance grows without divergence.

**Gravity tilt refinement** (every post-align state, stillness-gated by gyro+accel statistics — in flight the gate simply never passes): `z = R̂(−f̂/|f̂|) − e_D`, rows N/E only, `H = [e_D]×` rows 1–2 — rank-2, cannot touch heading. **Zero-angular-rate update** (same stillness gate, same cadence): a measured-still gyro reads exactly its bias, so `z = ω_raw`, prediction `b̂g`, `H = I` on δbg with a 3-dof χ² gate — the gate is the windup protection: a slow real rotation that slips past the stillness screen is rejected once `P_bg` has converged instead of being absorbed into the bias. Together these stop pad attitude drift at its source, independent of the magnetometer.

## Initialization

2.5 s static window → tilt from the accel mean, heading from the tilt-compensated mag mean (mode-dependent), `bg` from the gyro mean (earth rate ≪ bias floor, absorbed). Stillness requires per-axis gyro σ < 0.02 rad/s AND accel σ < 0.35 m/s² AND `||f̄|−g| < 0.5` — a moving vehicle restarts the window; after 20 s the filter accepts a degraded init with 3× covariance and flags it. Initial σ: roll/pitch 2°, yaw 5° (mag) / 57° (no mag), bg 0.01 rad/s (DS ZRO ±0.4 dps), ba 0.2 m/s² (DS ±20 mg), p/v from the anchor fix's reported accuracies, bb 5 m. The origin anchors on the first fix with fixType 3, gnssFixOK, ≥6 SV, hAcc ≤ 5 m.

## Tuning table

| Symptom | Turn |
|---|---|
| Position lags GNSS steps / NIS_gps ≪ 1 | raise `VIB_ACCEL_MULT` (or GNSS R floors down) |
| Position jitters at GNSS rate / NIS_gps ≫ 3 | lower `VIB_ACCEL_MULT`, raise `GPS_POS_R_FLOOR` |
| Attitude noisy at rest | lower `VIB_GYRO_MULT` |
| Heading wanders with mag on | check `MAG_HARD_UT`/`MAG_SOFT` cal, then lower `MAG_R_FLOOR` |
| Mag rejected constantly (kmg=0) | recalibrate iron; widen `MAG_NORM_TOL_FRAC`/`MAG_INCL_TOL_RAD` |
| Baro rejected through transonic | raise `TRANSONIC_VAR` or `TRANSONIC_SIGMA_M` |
| Baro fights GNSS altitude slowly | raise `BARO_BIAS_RW` (bb tracks weather faster) |
| Biases converge then wander | lower `GYRO_BIAS_RW`/`ACCEL_BIAS_RW` |
| Sluggish bias convergence after power-on | raise `INIT_BG_STD`/`INIT_BA_STD` |
| Altitude noisy at speed | raise `BARO_R_DYN_K` |
