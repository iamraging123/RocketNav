# Handoff: magnetometer subsystem redesign

## Mission

Redesign the magnetometer path in RocketNav end-to-end — driver usage, sampling,
calibration (algorithm + user procedure), filter integration, and persistence.
The current implementation is algorithmically defensible and host-test-green,
but it has repeatedly failed the owner on real hardware (symptom log below), and
the owner has asked for a clean rebuild: "redesign the magnetometer because it's
just not working." Nothing in the current mag pipeline is sacred except the hard
constraints listed at the bottom. **Diagnose on hardware with raw data before
rebuilding — the symptom log contains one unresolved observation that could
invalidate every software-level theory so far.**

## Hardware facts

- **MCU**: STM32F722RETx. Build with
  `arduino-cli compile --fqbn STMicroelectronics:stm32:GenF7:pnum=GENERIC_F722RETX .`
  from `firmware/RocketNav/` (stm32duino core 2.12.0; currently 111,748 B flash,
  18,440 B RAM).
- **Sensor**: ST **LIS3MDL** 3-axis magnetometer, **I2C address 0x1C** (SDO/SA1
  grounded, CS tied high for I2C mode).
- **Bus**: **I2C1 @ 400 kHz — SDA = PB7, SCL = PB6** (the Arduino variant's Wire
  pins; the bus-clear recovery routine toggles exactly these). Shared with:
  ICM-45686 IMU @ 0x68 (400 Hz reads), BME280 baro @ 0x77, SAM-M8Q GNSS @
  0x42 — and, per PLAN_LORA_CONTROL.md, a PCA9685 servo-PWM controller
  @ 0x40 driving 4 canard servos is being added to this same bus: servo
  current pulses and servo magnets are prime mag-interference suspects.
- **No DRDY/interrupt wire from the mag.** Freshness comes from polling the
  STATUS register (ZYXDA bit) at 100 Hz (`MAG_POLL_HZ`), sensor ODR 80 Hz.
- **Driver**: Adafruit_LIS3MDL 1.2.5 + Adafruit BusIO 1.17.4. Current config:
  ultra-high-performance mode, 80 Hz, continuous, **±4 gauss** full scale
  (6842 LSB/gauss → 0.14620 µT/LSB, `kLis3mdlUtPerLsb = 100.0f/6842.0f`;
  saturation at ±400 µT).
- **Mount rotation (sensor→body)**, applied immediately after LSB scaling:
  `R_MOUNT_MAG = [ 0 0 -1 ; -1 0 0 ; 0 1 0 ]`.
- **Telemetry/command serial**: 921600 Bd NDJSON out; ground commands arrive on
  the same serial as `$cmd` lines (see SCHEMA.md "Command input").
- **Possible interference sources on the board**: WS2812B LED strips on PC10
  (2 LEDs) and PC13 (4 LEDs) — current-carrying wiring whose routing relative to
  the mag is unknown; USB cable position; anything steel on the bench. A servo
  with magnets is expected on the vehicle eventually.
- **Site field** (Champaign IL, IGRF-14, Aug 2026): NED
  `{20.15, −1.23, 47.77}` µT, |B| = 51.86 µT, declination −3.50°, inclination
  +67.09° down. **Only ~20 µT of the field is horizontal — every µT of
  uncorrected horizontal iron costs ~3° of heading.** This site punishes
  compassing.

## Repo map (mag-relevant)

- `firmware/RocketNav/sensors.cpp/.h` — `beginMag()` (config above),
  `pollMag()` pipeline in order: ZYXDA gate → read → stuck detector (200
  identical raw triplets latches a fault; watchdog `service()` reinits with
  backoff) → µT scaling → mount remap → magcal collection (raw, pre-subtract) →
  hard-iron subtract (`mag_hard_ram_[3]`) → soft-iron matrix multiply
  (**currently identity**) → |m| ∈ [5, 200] µT validity gate → output.
  Plus `magCalStart/Service/Progress/TakeResult`.
- `firmware/RocketNav/magfit.cpp/.h` — pure least-squares sphere fit for hard
  iron (no Arduino deps, host-tested): double accumulators, first-sample
  centering, [5,200] µT sample gate, per-axis spread reporting, ≥200 samples.
- `firmware/RocketNav/cfgstore.cpp/.h` — hard-iron persistence in **flash
  sector 7 (0x08060000, last 128 KB)**, CRC'd record, HAL erase/program. The
  sector erase **freezes the whole chip 1–2 s** (single-bank flash).
- `firmware/RocketNav/eskf.cpp/.h` — 16-state ESKF. Mag enters exactly twice:
  1. **Alignment**: mag samples averaged during the final still window only,
     gated on |m| within ±15% of site |B|; ≥20 accepted samples → initial yaw
     via tilt-compensated bearing vs site declination (`mag_init_used_`), else
     wide no-mag prior (yaw 0, σ≈1 rad).
  2. **Run**: full 3-axis vector update `ν = R̂ m_b − B_run`, `H = [B_run]×` on
     δθ (rank 2, null along field line), `R = (0.5 µT)² I₃`. Pre-gates:
     magnitude ±15%, inclination ±10°, then χ²(3 dof) 11.34. **B_run is
     captured at alignment from the measured field** (azimuth = site
     declination by construction); site vector is the fallback when alignment
     had no mag. `$cal` recaptures.
- `firmware/RocketNav/RocketNav.ino` — all config constants; command handlers
  `$magcal` / `$magclr` / `$cal`; magcal progress + result messaging in the
  10 Hz service slot.
- `SCHEMA.md` — telemetry contract. Mag fields in `st` records: `mag` [3] µT
  (calibrated, body), `img` (mag innovation N-component, µT), `nmg` (NIS),
  `kmg` (accepted), `hmag`/`dmag` (health/drops). **Every emitted field must be
  documented here.**
- `viewer/index.html` — single-file Web Serial ground station. Its parser is
  contract-locked to `viewer/parse_test.js` (`node parse_test.js` → 31/31).
  Ctrl tab has "Mag cal (rotate 30 s)" and "Clear stored mag cal" buttons; all
  `msg` records land in the console and startup-log panel.
- `test/` — host tests, built with zig (not on PATH):
  `%LOCALAPPDATA%\Microsoft\WinGet\Packages\zig.zig_Microsoft.Winget.Source_8wekyb3d8bbwe\zig-x86_64-windows-0.16.0\zig.exe`
  - `zig c++ -std=c++17 -O2 -I../firmware/RocketNav -o eskf_test.exe eskf_test.cpp ../firmware/RocketNav/eskf.cpp ../firmware/RocketNav/nav_frames.cpp` → 91 checks
  - same pattern for `magfit_test.cpp` (+ `magfit.cpp`) → 10 checks

## Symptom log (chronological, all on real hardware)

1. **Startup heading not repeatable** boot-to-boot at the same orientation.
   Software fixes shipped: align mag mean restricted to the verified-still
   window; boot-captured run reference; flash-persisted calibration. Verified
   in host simulation only.
2. After those fixes: heading **repeatable but angle-dependent** — pointing
   50° reads 50°, pointing 350° reads ~290° (−60°). Classic hard-iron
   sinusoid; consistent with an uncalibrated board at this site.
3. **`$magcal` produced no output after the ack**, ever. Two structural holes
   found in the then-current build (completion required a fresh mag sample
   after the 30 s deadline; flash save ran before any result message) — both
   closed: completion is now time-based in the service path, progress
   heartbeats at 10/20 s report `n / spreads / |m|`, results are flushed to
   the wire before the erase, save confirmed after. **This fail-visible build
   has NOT been confirmed on hardware yet.**
4. **Unresolved owner observation** (earlier session, their words): rotating
   the board leaves the raw XYZ µT readings "at those exact XYZ uT values".
   This was interpreted at the time as "readings are stable/noise-free", but
   it can equally mean **the readings do not change with rotation** — a stuck
   sensor, a saturated axis, a driver returning cached data, or a wrong
   register map. That would explain every downstream symptom at once
   (arbitrary headings, cal sweeps with no spread). **Nobody has ever
   verified on hardware that the three components actually trace sinusoids
   during a slow 360° rotation. Do this first.**

## Diagnosis checklist (before any redesign)

1. Flash the current build. Startup log should show `mag LIS3MDL @0x1C: OK`
   and either `cfg: stored mag hard-iron …` or `cfg: no stored mag cal`.
2. Watch the `mag` field of `st` records while rotating the board slowly
   through a full turn, level: X/Y must trace ±20 µT sinusoids, Z ≈ 48 µT
   roughly constant, |m| roughly constant. **If the components barely move,
   the problem is sensor/driver/remap — fix that before touching anything
   else** (WHO_AM_I check, raw register dump, ZYXDA semantics, BusIO read
   path, ±4 gauss saturation, axis remap).
3. Run `$magcal`: heartbeats at 10/20 s must appear, `n` growing ~80/s,
   spreads growing on all three axes during a tumble, |m| inside [5,200].
4. If the stream dies right after `magcal: saving to flash - board freezes
   ~2 s` → the sector-7 erase kills this board; redesign persistence (defer
   to an explicit `$magsave`, erase-less journal slots, or RTC backup
   registers).
5. |m| at rest far from ~52 µT (say <30 or >80) → big iron or scale/range
   problem: check for magnets/speakers/steel near the sensor and the LED
   wiring runs.

## Redesign scope

Free to replace wholesale:
- Adafruit driver → a minimal hand-written register driver (repo precedent:
  the ICM-45686 driver here is hand-written; the owner prefers owned code).
- Sampling scheme (rates, freshness, fault handling).
- Calibration algorithm and UX: continuous background hard-iron tracking,
  ellipsoid (soft-iron) fit, guided per-axis procedure with live viewer
  feedback are all on the table.
- Filter gating strategy and reference handling.
- Persistence mechanism.

Hard constraints (do not violate):
- 16-state ESKF architecture stays; GLOBAL-NED attitude-error convention and
  Joseph-form updates as in `eskf.cpp`; float32 state.
- No external AHRS/fusion/calibration libraries — owned code only.
- No flight-phase / pyro / SD / radio / control logic.
- SCHEMA.md documents every emitted field; `viewer/parse_test.js` must stay
  green (33/33 as of the 2026-08-29 build below) — extend it when the
  schema grows.
- Plain-ASCII telemetry text; no TODO/FIXME/placeholder markers anywhere.
- Verify every round: arduino-cli compile + zig host tests + node parse_test.js.

## Acceptance criteria

- Same orientation, power cycle → heading repeats within 2°.
- After the documented calibration procedure: heading error < 5° at
  0/45/90/…/315° with the board level.
- Calibration cannot end silently: live progress, explicit success/failure,
  a failed or thin sweep changes nothing and says so.
- Mag outage (unplugged mid-run, bus fault) degrades gracefully: filter keeps
  running on the remaining aids, health flags show it, heading uncertainty
  grows honestly instead of freezing a stale value.

---

## 2026-08-29 diagnostic build (round 1 of the redesign)

### Prime suspect identified in the old read path

The LIS3MDL requires SUB(7) = 1 in the I2C sub-address to auto-increment
across a multi-byte read ("to read multiple bytes it is necessary to assert
the most significant bit of the subaddress field" — DocID023312; ST's own
STdC examples and Pololu's driver both OR in 0x80). **Adafruit BusIO only
applies its `AD8_HIGH_TOREAD_AD7_HIGH_TOINC` address policy on the SPI
branch** — `Adafruit_BusIO_Register::read()` hands the raw sub-address to
`write_then_read()` on I2C (BusIO 1.17.4, Adafruit_BusIO_Register.cpp:223).
So `Adafruit_LIS3MDL::read()` clocked 6 bytes from 0x28 without the bit.
Without auto-increment the part serves OUT_X_L for all six bytes → x = y =
z = (b | b<<8): rotation-shaped garbage that collapses the three axes onto
one line — consistent with symptom 4 and with every heading pathology
downstream. Not yet PROVEN on this silicon (the Adafruit lib has a large
I2C user base, so the chip may tolerate it) — `$magdiag` below settles it
in one command.

### What changed (all verified: compile 112,332 B / 18,432 B RAM,
eskf_test 91/91, magfit_test 10/10, parse_test 33/33)

- **Owned driver `lis3mdl.h/.cpp`** replaces Adafruit_LIS3MDL (repo style =
  icm45686). Every multi-byte read ORs in 0x80. UHP / 80 Hz / ±4 gauss /
  continuous as before, plus **BDU set** (CTRL5, the old stack never set it
  — axis words could tear across an 80 Hz update mid-burst). `begin()`
  soft-resets, writes CTRL1–5, then **verifies the config with a 5-byte
  auto-increment burst readback** — five distinct values can only match if
  the address pointer really advances, so a part that can't do burst reads
  now fails loudly at boot instead of lying quietly forever.
- **Atomic poll**: one 7-byte burst STATUS→OUT_Z_H; ZYXDA and its sample
  come from the same transaction. Bus errors now count (like the IMU path)
  and fault→reinit after 50 consecutive; before, a mag bus failure read as
  an eternally-quiet ZYXDA. Axis-at-range-limit sets the `hmag` sat bit (8).
- **`mgr` telemetry field** (st records): raw field, scaled + mount-rotated
  only, BEFORE hard/soft iron and the [5,200] µT gate — updates on every
  fresh register read, so the ground sees the sensor move even while gating
  freezes `mag`. SCHEMA.md updated; viewer has a "mag raw" chart (x/y/z +
  |m|) and a "mag raw µT" Sensors-table row; CSV grew to 93 cols
  (mgr_x/y/z); parse_test extended 31→33 checks.
- **`$magdiag` command**: register-level probe that deliberately bypasses
  the driver (raw Wire, works even when begin() failed). Emits WHO_AM_I,
  CTRL1–5 vs intended `7C 00 00 0C 40`, STATUS, the six output registers
  read three ways — burst WITHOUT 0x80 (the old library's exact transaction
  shape), burst WITH 0x80, and six single-byte reads (ground truth) — plus
  a 12-poll ZYXDA/data-change liveness count and one `VERDICT` line:
  wrong WHO_AM_I → wiring/address; ZYXDA never set → config dead; data
  frozen while sampling → wedged silicon; noinc burst collapsed to three
  identical words while inc shows a vector → **the auto-inc bug was the
  root cause and is now fixed**; all healthy → rotate and watch `mgr`.

### Bench procedure (owner, ~3 minutes)

1. Flash `firmware/build/RocketNav.ino.bin` (ST-LINK was enumerated on USB
   during this session but reported "No target connected" — plug the probe
   into the board, or flash however you usually do). Programming touches
   sectors 0–4 only; a stored magcal in sector 7 survives.
2. Boot log must show `mag LIS3MDL @0x1C: OK`. If it shows NOT FOUND where
   it used to say OK, the new config readback is failing — run `$magdiag`
   and read the verdict; that is signal, not regression.
3. Send `$magdiag`, keep the 5 msg lines. The VERDICT line names the fault
   class directly.
4. Rotate the board slowly through a full level turn watching the **mag
   raw** chart (or the `mgr` field): X/Y must trace ±20 µT sinusoids, Z
   ≈ 48 µT and |m| ≈ constant. This is the decisive test from the symptom
   log — raw now, so no gate can mask it.
5. If sinusoids are clean: `$magcal` tumble (watch 10/20 s heartbeats +
   spread), then `$cal`, then the 8-heading check against the acceptance
   criteria.

### Deliberately NOT touched this round (waiting on the rotation data)

Calibration algorithm/UX, ESKF gating and reference handling, and flash
persistence are unchanged: whether they need redesign depends on what steps
3–5 show (e.g. if the auto-inc bug was the root cause, every $magcal ever
run fitted garbage and simply redoing it on good data may land inside the
5° criterion before any soft-iron work is justified).

### Session-side hardware notes

- COM5 was held open by another process at probe time (likely a viewer tab
  still attached) — the diagnosis could not be run from this side. COM3 and
  COM9 are ESP32 projects, COM15 caps at 115200; the RocketNav link is
  almost certainly COM5.
- `ST-LINK_CLI.exe` (STM32 ST-LINK Utility) is the flasher present on this
  machine; STM32CubeProgrammer is NOT installed, so `arduino-cli upload`
  (which shells out to CubeProgrammer on this core) will not work as-is.
  `ST-LINK_CLI -c SWD -P firmware\build\RocketNav.ino.hex -V -Rst` is the
  working incantation once the probe sees the target.
