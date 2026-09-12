# RocketNav Serial Schema (version 1)

Newline-delimited JSON at **921600 baud**, decimated to 50 Hz (configurable),
independent of the filter rate. Records never block: when the link backs up,
whole records are dropped and `dtx` increments (visible as `seq` gaps).
Any field can be JSON `null` when its source is unavailable; numbers are
never NaN/inf. Record type is the `t` field.

## Record `"hdr"` — once at startup

| key | type | units | description |
|---|---|---|---|
| `t` | string | — | `"hdr"` |
| `us` | uint64 | µs | monotonic timestamp |
| `fw` | string | — | firmware version |
| `sch` | int | — | schema version (this document: 1) |
| `imu` | 0/1 | — | ICM-45686 present at boot |
| `mag` | 0/1 | — | LIS3MDL present at boot |
| `bar` | 0/1 | — | BME280 present at boot |
| `gps` | 0/1 | — | SAM-M8Q present at boot |
| `iodr` | int | Hz | configured IMU ODR |
| `afs` | int | g | accel full scale |
| `gfs` | int | dps | gyro full scale |
| `modr` | int | Hz | mag ODR |
| `mmode` | string | — | `"cont"` / `"init"` / `"off"` |
| `gmode` | string | — | GPS usage: `"auto"` (att-only if no fix by the deadline; a later fix upgrades to nav) / `"off"` / `"req"` |
| `bhz` | int | Hz | baro poll rate |
| `ghz` | int | Hz | requested GNSS nav rate |
| `ohz` | int | Hz | telemetry output rate |

## Record `"msg"` — startup/status lines (boot and state transitions only)

| key | type | units | description |
|---|---|---|---|
| `t` | string | — | `"msg"` |
| `us` | uint64 | µs | monotonic timestamp |
| `txt` | string | — | human-readable status line (plain ASCII, no escapes) |

Emitted at boot (one per sensor probe) and on filter-state transitions
(calibration start/done, GPS determination, origin anchor). Never periodic.

## Record `"cal"` — once, when the startup calibration window completes

The 3 s static alignment doubles as the calibration pass: the gyro offset is
the averaged stationary rate (removed from all subsequent output), and the
initial attitude comes from the averaged gravity vector plus the mag heading.

| key | type | units | description |
|---|---|---|---|
| `t` | string | — | `"cal"` |
| `us` | uint64 | µs | monotonic timestamp |
| `bg` | [3] | deg/s | averaged gyro offset, body axes |
| `eul` | [3] | deg | initial attitude [roll, pitch, yaw] |
| `alq` | 0/1 | — | 1 = clean static window, 0 = degraded (was moving) |
| `dur` | float | s | averaging duration |

## Record `"org"` — once, when the NED origin anchors

| key | type | units | description |
|---|---|---|---|
| `t` | string | — | `"org"` |
| `us` | uint64 | µs | monotonic timestamp |
| `lat` | double | deg | origin latitude (7 dp) |
| `lon` | double | deg | origin longitude (7 dp) |
| `alt` | float | m | origin altitude, MSL |
| `hae` | float | m | origin height above ellipsoid |
| `g0` | float | m/s² | site gravity (WGS-84 Somigliana) |
| `p0` | float | Pa | baro reference pressure (alignment mean) |
| `t0` | float | °C | baro reference temperature |

## Record `"st"` — state, 50 Hz

Attitude fields (`q`, `eul`, `sa`, `bg`) are meaningful from `fst ≥ 2`;
position/velocity fields are non-null only in `fst = 3`. The baro-damped
vertical channel (`ral`, `rvs`, `sra`, `srv`) is non-null in `fst` 2, 3
and 4 — altitude and vertical rate never wait for GNSS.

| key | type | units | description |
|---|---|---|---|
| `t` | string | — | `"st"` |
| `us` | uint64 | µs | monotonic timestamp of this record |
| `seq` | uint32 | — | record counter (gaps = dropped records) |
| `fst` | int | — | filter state: 0 init, 1 aligning, 2 wait-fix, 3 run, 4 attitude-only (GPS-less; in auto GPS mode a later valid fix anchors and moves 4 → 3) |
| `alq` | 0/1 | — | 1 = clean static alignment, 0 = degraded (was moving) |
| `lat` | double\|null | deg | estimated latitude (7 dp) |
| `lon` | double\|null | deg | estimated longitude (7 dp) |
| `alt` | double\|null | m | estimated altitude MSL — baro-referenced: the vertical channel fuses baro+IMU only, GNSS altitude is logged raw (`galt`) but never fused |
| `p` | [3]\|null | m | NED position from origin [N,E,D]; D is the pad/baro-datum vertical — GNSS observes N/E only |
| `v` | [3]\|null | m/s | NED velocity [N,E,D] |
| `q` | [4] | — | attitude quaternion body→NED [w,x,y,z] |
| `eul` | [3] | deg | Euler ZYX [roll, pitch, yaw] |
| `sp` | [3]\|null | m | 1σ position uncertainty [N,E,D] |
| `sv` | [3]\|null | m/s | 1σ velocity uncertainty [N,E,D] |
| `sa` | [3] | deg | 1σ attitude uncertainty [about N, E, D]; D = heading |
| `bg` | [3] | rad/s | estimated gyro bias (body) |
| `ba` | [3] | m/s² | estimated accel bias (body) |
| `bb` | float\|null | m | baro altitude bias — pinned ≈0 (the baro defines the vertical datum) |
| `ral` | float\|null | m | vertical-channel altitude above the pad baro reference (−p_D + bb) |
| `rvs` | float\|null | m/s | vertical-channel vertical rate, up positive |
| `sra` | float\|null | m | 1σ of `ral` |
| `srv` | float\|null | m/s | 1σ of `rvs` |
| `acc` | [3]\|null | m/s² | calibrated specific force fed to the filter (body) |
| `gyr` | [3]\|null | rad/s | calibrated angular rate fed to the filter (body) |
| `mag` | [3]\|null | µT | calibrated magnetic field fed to the filter (body) |
| `mgr` | [3]\|null | µT | raw magnetic field: scaled + mount-rotated only, before hard/soft-iron, validity gates and the filter — updates on every fresh register read (rotation diagnostics); null until the first read succeeds |
| `pa` | float\|null | Pa | pressure fed to the filter |
| `tc` | float\|null | °C | air temperature fed to the filter |
| `gfix` | int\|null | — | raw GNSS fix type (3 = 3D) |
| `gsv` | int\|null | — | satellites used |
| `ghac` | float\|null | m | reported horizontal accuracy (hAcc) |
| `gvac` | float\|null | m | reported vertical accuracy (vAcc) |
| `gsac` | float\|null | m/s | reported speed accuracy (sAcc) |
| `glat` | double\|null | deg | raw GNSS latitude (7 dp) |
| `glon` | double\|null | deg | raw GNSS longitude (7 dp) |
| `galt` | float\|null | m | raw GNSS altitude MSL |
| `gage` | float\|null | s | age of the last PVT at record time |
| `igp` | [3]\|null | m | last GNSS position innovation [N,E,0] — horizontal-only update |
| `igv` | [3]\|null | m/s | last GNSS velocity innovation [N,E,0] — horizontal-only update |
| `img` | float\|null | µT | last mag vector innovation, N component (full 3-axis update) |
| `ibr` | float\|null | m | last baro altitude innovation |
| `ngp` | float\|null | — | NIS of last GNSS position update (χ², 3 dof) |
| `ngv` | float\|null | — | NIS of last GNSS velocity update (3 dof) |
| `nmg` | float\|null | — | NIS of last mag update (3 dof, vector); null if field-gated |
| `nbr` | float\|null | — | NIS of last baro update (1 dof) |
| `kgp` | 0/1\|null | — | last GNSS position update accepted |
| `kgv` | 0/1\|null | — | last GNSS velocity update accepted |
| `kmg` | 0/1\|null | — | last mag update accepted |
| `kbr` | 0/1\|null | — | last baro update accepted |
| `himu` | int | bitmask | IMU health (bit values below) |
| `hmag` | int | bitmask | mag health |
| `hbar` | int | bitmask | baro health |
| `hgps` | int | bitmask | GNSS health |
| `dimu` | uint32 | — | IMU samples dropped before the filter |
| `dmag` | uint32 | — | mag samples dropped |
| `dbar` | uint32 | — | baro samples dropped |
| `dgps` | uint32 | — | GNSS fixes rejected at fix-level validation |
| `dtx` | uint32 | — | telemetry records dropped (link backpressure) |
| `ei2c` | uint32 | — | I2C transaction errors |
| `ri2c` | uint32 | — | I2C bus-clear/reset events |
| `grz` | float\|null | Hz | measured GNSS nav rate (runtime-verified) |
| `fhz` | float | Hz | achieved filter propagation rate |
| `lmx` | uint32 | µs | max loop period since the previous record |
| `lre` | 0/1 | — | LoRa radio (SX1278) present at boot |
| `rssi` | float\|null | dBm | RSSI of the last uplink frame heard by the rocket; null until one arrives |
| `snr` | float\|null | dB | SNR of the last uplink frame |
| `ltx` | uint32 | — | LoRa frames transmitted |
| `lrx` | uint32 | — | uplink command frames accepted |
| `lcrc` | uint32 | — | air frames rejected (CRC/shape) |
| `cmode` | int | — | control mode: 0 idle, 1 armed, 2 active, 3 safe, 4 bench (ground roll-hold test) |
| `cdef` | [4] | deg | canard deflection commands — TRUE canard degrees, mapped through the per-fin linkage calibration |

### Health bitmask (himu/hmag/hbar/hgps)

| bit | value | meaning |
|---|---|---|
| 0 | 1 | present (begin() succeeded) |
| 1 | 2 | fresh (valid sample inside the staleness window) |
| 2 | 4 | stale (present but no valid samples lately) |
| 3 | 8 | saturated (latest sample at full scale: accel for `himu`, a field axis at the ±4 gauss range limit for `hmag`) |
| 4 | 16 | stuck (N identical consecutive raw samples) |
| 5 | 32 | fault (bus/init failure; reinit with backoff in progress) |

## Record `"lcal"` — linkage-cal snapshot (on demand)

Emitted for all four fins plus the staged session points at boot, after
every `$lcal` action that changes anything, and on `$lcal dump` — the
viewer's point list and linkage graph render from these.

| key | type | units | description |
|---|---|---|---|
| `t` | string | — | `"lcal"` |
| `us` | uint64 | µs | monotonic timestamp |
| `fin` | int | — | fin index 0-3; −1 with `src` 2 = no active session |
| `src` | int | — | 0 = synthesized default table, 1 = calibrated table applied to the servos (normally flash-stored; after a `flash save FAILED` reply it is RAM-only until reboot — `$lcal flash` shows what actually persisted), 2 = staged points of the active `$lcal` session (entry order) |
| `n` | int | — | number of points (0-5) |
| `deg` | [n] | deg | canard angles |
| `pus` | [n] | µs | matching servo pulses |

## Command input (ground → vehicle)

The telemetry serial is bidirectional: the ground station may send
newline-terminated ASCII lines starting with `$`. Every command is answered
with a `"msg"` record whose `txt` starts `cmd:`, so replies ride the normal
stream into any log. Malformed or unknown input answers `cmd: unknown`;
overlong lines are dropped and the parser resyncs at the next newline.

| command | effect |
|---|---|
| `$ping` | replies `cmd: pong` (link check) |
| `$cal` | full navigation restart: alignment re-averages the gyro offset and attitude, the baro reference is re-collected, and the origin re-anchors on the next valid fix |
| `$led <a\|b\|all> <r> <g> <b>` | sets a WS2812B strip color (a = 2 px on PC10, b = 4 px on PC13; channels 0–255, out-of-range values clamp) |
| `$magcal` | 30 s hard-iron calibration: tumble the vehicle through **all** orientations. Progress msgs at 10/20 s report live sample count, per-axis spread and \|m\|. A least-squares sphere fit estimates the offset (outlier-tolerant); any axis whose field swing stayed under 30 uT is **held at its previous value** and flagged in a `magcal: thin coverage` msg (a flat spin cannot separate iron from the Earth field on the vertical axis). The sweep closes on time even if the sensor faults mid-cal. Result and coverage msgs are flushed to the wire, then the offset is **saved to on-chip flash** (survives reset and power cycles; the save freezes the chip ~1-2 s, announced by a `saving to flash` msg) and confirmed with a `saved to flash` msg. Send `$cal` afterwards to re-align on the corrected field |
| `$magclr` | zeroes the stored mag calibration (the config record is rewritten - the linkage calibration is preserved; ~1-2 s stall); send `$cal` afterwards |
| `$lcal start <fin>` → `jog <±us>` / `goto <us>` → `point <measured_deg>` (**1 point = offset-only**, keeps the default gain; 2-5 = measured curve; `point 0` = flush canard) → `save` → `check <deg>`; also `undo`, `del <idx>`, `edit <idx> <deg>`, `load` (copy the fin's active table into the stage for editing), `show`, `dump` (emit `lcal` records), `flash` (non-destructive report of what is physically stored in the flash sector — run after a power cycle to confirm a save persisted), `clear <fin\|all>`, `abort` | per-fin **linkage calibration** (IDLE only): builds a monotonic table of up to 5 (canard°, servo µs) points measured on the bench, validated and saved to flash (~2 s freeze). Table ends are the mechanical limits and set the per-fin clamp; control authority becomes min(compile clamp, smallest table end), reported at boot (`cfg: linkage cal n/4 fins - authority …`). Angles must be inside ±89° and pulses inside 500–2500 µs (the same bounds the flash loader enforces, so every saved table survives a reboot); `point` is refused while a servo sweep/scan/fin-test is driving the fins (the staged pulse would not be the one holding the fin). Uncalibrated fins keep the compile-time linear map |
| `$servo <0-3> <us>` \| `$servo center` \| `$servo off` \| `$servo sweep` | bench servo overrides via the PCA9685 (IDLE only): raw pulse to one canard (index strictly 0–3, pulse strictly 900–2100 µs — malformed arguments are rejected, never driven), all to center, pulses stopped (limp), or a 6 s triangle sweep of all four |
| `$cang <fin 0-3> <deg>` \| `$cang <fin> off` \| `$cang center` \| `$cang test` \| `$cang alloff` | **canard angle** through the linkage calibration (IDLE only): `<deg>` is TRUE canard deflection, so `0` = the calibrated neutral (fin straight), not the raw servo center. `off` releases one fin (limp, **latched** — it stays released until that fin is driven again or `center`/a bench mode takes the outputs back), `center` sends all four canards to neutral, `test` runs an all-fin functional sweep (+10 -> -10 -> 0 deg, calibrated), `alloff` releases all four. Calibrate the fin (`$lcal`) first, else `0` is just the 1500 us default center |
| `$ang <ch 0-15> <deg 0-180>` \| `$ang <ch> off` \| `$ang alloff` \| `$ang scan` | servo-angle debug on ANY of the 16 PCA9685 channels (IDLE only). Standard hobby mapping: 0-180° → 1000-2000 µs, 90° = 1500 µs center. `off` releases one channel (pulses stop, servo limp), `alloff` releases all 16. `scan` wiggles every channel in turn (60→120→90°, ~1.2 s each, ~19 s total) with an `ang: scan - wiggling ch N` msg per channel — watch which servo moves to learn the true header→channel map, then fix `SERVO_CH`. Channels 0-3 are assumed to be the canards |
| `$arm` → `$arm yes` | two-step arming: `$arm` opens a 5 s window, `$arm yes` inside it enters ARMED (requires aligned filter, fresh IMU, PCA9685 present). Launch detect (sustained \|a\| > 3 g) then activates the roll controller |
| `$disarm` | any state → IDLE, canards centered. Also the abort during ARMED/ACTIVE/BENCH |
| `$rolltest` | **ground roll-hold test** (IDLE → BENCH): runs the real roll PID immediately, no arming and no launch detect. Same preconditions as `$arm` (aligned filter, fresh IMU, PCA9685 present). Rate mode → canards fight an imposed spin; angle mode → holds the roll it started in. The flight SAFE triggers (tilt / descent / 20 s timeout) are disabled so you can handle the airframe freely; an IMU fault still drops it to SAFE. `$disarm` to stop |
| `$ctl <kp_rate> <ki_rate> <kp_ang>` | live controller gains (RAM; echoed as `ctl:` msg ×1000) |
| `$sframe <hz>` | live servo PWM frame rate, 24–333 Hz (IDLE only, RAM; boot default 50). The frame period is the dominant command→pulse latency: 50 Hz ⇒ up to 20 ms, digital servos at 200–333 Hz cut it ~4×. The write cadence and the write deadband rescale with it. A frame change rescales the counts of **all 16 channels**, so channels 4–15 (anything parked by `$ang`) are released (limp) rather than left outputting wrong pulses; the canards are rewritten at the new frame immediately. Reply msg reports the achieved (prescale-quantized) frame. If the reprogram sequence dies mid-way on a bus fault the chip is reported absent (health flags it, arming refuses) and the 10 Hz recovery probe re-initializes it. Analog servos may buzz or heat at high rates — watch them |
| `$ctlmode rate\|angle` | rate damping (gyro only) vs roll-angle hold (uses estimated roll; target captured at ACTIVE entry) |
| `$lora 0\|1` / `$lora?` | mute/unmute the downlink; status msg with tx/rx/crc counters and last uplink RSSI/SNR. **Boots muted** (`LORA_TX_AT_BOOT 0`, bench default - an antenna-less Ra-02 must never transmit); RX runs regardless, so uplink commands still arrive while muted |
| `$sens?` | one-line health msg: present/fresh per sensor, PCA9685, radio, I2C error counters |

`$cal`, `$magcal` and `$magclr` are refused while ARMED, ACTIVE or BENCH (the
magcal flash save freezes the chip ~2 s and a re-alignment while the canards
are live is never acceptable) - `$disarm` first. They are allowed in SAFE.

## LoRa link

Rocket Ra-02 (SX1278, SPI1: PA5/PA6/PA7, NSS PA4, RESET PC4, DIO0 PB0) ⇄
base station ESP32-C3 + Ra-02 (`basestation/`). Air profile (both ends,
`linkcodec.h`): 433.5 MHz, SF7, BW 500 kHz, CR 4/5, sync 0x4B, 17 dBm
PA_BOOST. The rocket transmits one 53-byte state frame every 250 ms (~27 ms
airtime, ~11% duty) and message frames on alternate slots when queued; it
listens (RX-continuous) the rest of the time. The base station only
transmits after hearing a frame, so uplinks never collide with the rocket's
own TX. Uplink `C` frames carry a `$command` line (sent twice, deduplicated
by sequence on the rocket) into the same command handler as USB.

The base station emits **the same NDJSON records** on its USB serial that
the rocket emits on its own - a slim `st` subset (attitude, rates, specific
force, velocity, `ral`, geodetic fix, health, control state) at 4 Hz plus
`rssi`/`snr`/`loss` measured at the base, and forwarded `msg` records — so
the viewer connects to either port unchanged. `$base?` typed in the console
reports base-side link statistics without transmitting.
| `$magdiag` | register-level magnetometer probe, independent of the driver and its init state (~60 ms stall): replies with msg records carrying WHO_AM_I, CTRL_REG1–5 vs the intended config, STATUS, the six output registers read three ways (burst without the I²C auto-increment bit — the shelf-driver transaction shape, burst with it, and single-byte reads), a 12-poll ZYXDA/data-change liveness count, and one `magdiag: VERDICT …` line naming the likely fault class |
| `$zero` | zeroes the sensor-drop and I²C error/reset counters |
| `$rst` | replies `cmd: rebooting`, drains the stream, resets the MCU |
