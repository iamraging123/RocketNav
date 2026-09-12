# Plan: canard linkage calibration (servo angle ≠ canard angle)

## Problem

The servos drive the canards through mechanical linkages, so the map from
servo command to true canard deflection is, per fin:

- **a gain ≠ 1** — horn-length ratio: 10° of servo is not 10° of canard;
- **an offset** — servo 90° (1500 µs) is not canard 0°: assembly tolerance
  puts every fin's neutral somewhere slightly different;
- **mild nonlinearity and asymmetry** — pushrod/horn geometry is roughly
  sinusoidal, so up-gain and down-gain differ a little;
- **real mechanical limits** — the linkage binds before the servo's own
  900–2100 µs range does.

Today's firmware model (`us = center_us + δ · us_per_deg`, constants
guessed) captures none of this per real fin. Everything downstream cares:
the PID commands *canard* degrees (the 6DOF's `C_l_δ` is defined in canard
degrees), telemetry `cdef` claims canard degrees, and the ±10° authority
clamp is meaningless if the linkage turns it into ±4° or ±23°.

## The model

Per fin: a **monotonic calibration table of up to 5 points
`(canard_deg, servo_us)`**, applied by piecewise-linear interpolation in the
deflection→pulse direction.

- 2 points = plain linear (gain + offset). 3 points adds independent
  up/down gains. 5 points captures the linkage curve. The calibration flow
  is identical for all of them.
- The **table's end points are the mechanical limits** found on the bench —
  they become the hard per-fin clamp, replacing the generic
  `SERVO_MIN/MAX_US` for calibrated fins.
- Uncalibrated fins fall back to a 2-point table built from today's
  compile-time constants — behavior unchanged until calibrated.
- Effective control authority = `min(CTL_DEFL_MAX, smallest |table end|
  over the four fins)`, reported at boot so a mechanically limited fin
  can't silently promise deflection it can't deliver.

`$ang` (raw servo angle on any channel) stays exactly as is — it's the
*servo-side* debug tool. The calibration lives one layer up, where canard
degrees are minted.

## Calibration procedure (bench, per fin — ~3 minutes each)

Tools: a straightedge (canard-zero reference against the body) and a phone
inclinometer app or printed protractor for measuring canard angles.
Precondition: the channel map is verified first (`$ang scan` — a cal saved
against the wrong channel is worse than none).

1. **Select the fin** in the viewer's new *Linkage cal* panel → the servo
   drives to the current center guess.
2. **Find neutral**: jog with ±10/±50 µs buttons until the canard is flush
   at 0° against the straightedge → *Mark center* (records the µs).
3. **Find the limits**: jog toward each end until just before the linkage
   binds → *Mark min* / *Mark max*. Measure the canard angle at each and
   type it in — these are the table's end points.
4. **Optional mid points** (recommended): the panel drives the servo
   halfway between center and each limit; measure the canard angle, type
   it, *Save point*. Now the table has 5 points and the linkage curve is
   captured.
5. **Save**: monotonicity and span are validated (≥ 8° each side
   recommended, warning below that), then the whole table set is written
   to flash — the same ~2 s freeze rules as `$magcal`, bench-only.
6. **Verify**: the panel's check mode commands −10/−5/0/+5/+10° *canard*
   and the fin should measure within ~1° at each — that's the acceptance
   test, per fin.

## Firmware changes

- **`servos.cpp/.h`**: per-fin `LinkageCal { uint8_t n; float deg[5];
  float us[5]; }`; `setDeflDeg()` interpolates through it (clamped at the
  table ends); default tables synthesized from the current constants.
  Cal-mode helpers: drive-to-µs, current-µs readout for the marking flow.
- **`cfgstore` v2**: the flash record grows to
  `{ magic "RNV2"; mag_hard[3]; LinkageCal[4]; crc }`. The loader accepts
  v1 (mag only, linkage defaults) and v2; the saver always writes v2, so
  the first linkage save silently migrates the record and the mag cal is
  preserved (the whole struct is cached in RAM and rewritten together —
  one sector, one erase per save).
- **`$lcal` command family** (IDLE only, refused while armed):
  `start <fin>`, `jog <±us>`, `goto <us>`, `center`, `min`, `max`,
  `point <measured_deg>`, `check <deg>`, `save`, `show`, `clear <fin|all>`,
  `abort`. Every step answers with a msg so the flow works over the
  console (and later LoRa) too.
- **Control**: authority clamp derived from the calibrated tables at boot
  and after `$lcal save`; boot log line
  `cfg: linkage cal 4/4 fins, authority ±8.5 deg` (or `defaults - run
  $lcal`).
- **Telemetry/SCHEMA**: `cdef` semantics upgraded to *true canard degrees*
  (documented); `$lcal` rows; no new st fields required.

## Viewer changes

*Linkage cal* panel in the Ctrl tab, replacing nothing (sits under Servo
angle): fin selector, jog buttons, live µs, the Mark buttons, a single
"measured canard °" input reused for limits/points/check, a stepper that
walks the 6-step procedure in order with the current step highlighted, and
Save/Show/Clear. All of it is just `$lcal` command wiring — no new
telemetry parsing.

## Tests (host, extending `pca_test`)

- Interpolation exactness on a known 5-point asymmetric table (including
  between-point values and both signs).
- Clamping at table ends; monotonicity validation rejects a bad table.
- Default-table equivalence: uncalibrated fin produces byte-identical
  I2C writes to today's linear map.
- v1→v2 record migration logic (pure struct/CRC part).
- Authority derivation = smallest table end.

## Phases

| # | Deliverable | Proof |
|---|---|---|
| 1 | Linkage table in `servos` + default synthesis | pca_test: equivalence + interpolation checks green |
| 2 | cfgstore v2 (+migration) + `$lcal` family | `$lcal show` round-trips a table across a power cycle |
| 3 | Viewer panel + guided flow | full cal of one fin from the browser only |
| 4 | Control authority wiring + SCHEMA/boot msgs | boot line reports per-fin authority |
| 5 | Bench acceptance | each fin: commanded ±5/±10° canard measures within ~1°; survives power cycle |

## Owner inputs before phase 1

1. Fix the channel map first (`$ang scan` results → `SERVO_CH`).
2. Rough linkage ratio (does 10° of servo move the canard more or less
   than 10°?) — only to pick sane jog step defaults.
3. Confirm 5-point tables are wanted (vs plain 3-point); the flow supports
   both, this only sets how many the guided stepper asks for.
