# Plan: LoRa telemetry link + active canard control

Owner has expanded the project scope: the long-standing "no radio / no control
logic" boundary is lifted for exactly these two systems. The navigation filter
itself stays phase-free — arming and flight-phase detection live only in the
new control layer.

---

## Part 1 — LoRa link (rocket Ra-02 ⇄ base station ESP32-C3 + Ra-02)

### 1.0 The governing constraint: bandwidth

The USB stream is ~55 kB/s (1.1 kB NDJSON × 50 Hz). LoRa at practical flight
settings delivers **2–3 kB/s at best** — the NDJSON stream cannot go over the
air. Therefore:

- **USB (unchanged)**: full 50 Hz NDJSON for bench and pad work.
- **LoRa**: compact **binary frames** — a 4 Hz downlink snapshot, opportunistic
  text-message frames, and a command uplink. The base station translates
  binary → NDJSON so **the viewer needs no protocol changes**: it just sees a
  slower, slimmer stream from a different serial port.

### 1.1 Hardware (from the owner's netlists)

Rocket (STM32F722RETx, SPI1 — default variant pins PA5=SCK / PA6=MISO /
PA7=MOSI, all currently unused in firmware):

| Signal | MCU pin | Ra-02 (SX1278) |
|---|---|---|
| SCK | PA5 (SPI1) | SCK |
| MISO | PA6 (SPI1) | MISO |
| MOSI | PA7 (SPI1) | MOSI |
| NSS | PA4 (GPIO, active low) | NSS |
| RESET | PC4 (GPIO, active low) | RESET |
| DIO0 | PB0 (GPIO input) | DIO0 (TxDone/RxDone IRQ) |

Base station (ESP32-C3):

| Signal | ESP32-C3 pin | Ra-02 pin |
|---|---|---|
| MOSI | GPIO5 | 14 |
| MISO | GPIO6 | 13 |
| SCK | GPIO7 | 12 |
| NSS | GPIO0 | 15 |
| RESET | GPIO4 | 4 |
| DIO0 | GPIO1 | 5 |
| 3V3/GND | — | 3 / 2 (100n+10n decoupling present) |

DIO1–DIO5 unconnected on both ends → **everything must run off DIO0 +
IRQ-flag register polls** (no RxTimeout via DIO1, etc.). Ra-02 TX path is
PA_BOOST → power settings must use PA_BOOST, 2–17 dBm.

### 1.2 Radio profile

| Profile | SF | BW | CR | 56 B airtime | Use |
|---|---|---|---|---|---|
| **A (default)** | 7 | 500 kHz | 4/5 | ~27 ms | flight, ~11% duty at 4 Hz |
| B (long-range fallback) | 8 | 250 kHz | 4/5 | ~113 ms | 1 Hz beacon if link degrades |

433.5 MHz (off the 433.92 remote-control pileup), explicit header, CRC on,
custom sync word 0x4B, preamble 8, TX 17 dBm PA_BOOST. Link budget at SF7/500:
sensitivity ≈ −117 dBm; at 5 km LOS, FSPL ≈ 99 dB → ~35 dB margin. Range is
not the constraint; airtime is.

Regulatory note (US): 433 MHz is the 70 cm amateur band — operating this as
telemetry is the ham-license path (callsign in the beacon; an ID string field
is reserved in the frame). Keep duty ≤ ~10%.

### 1.3 Protocol

All frames: `[0x4B][type][seq u8][payload…][crc16]`, max 64 B.

- **`T` downlink state frame @ 4 Hz** (~56 B): ms-since-boot u32; status u8
  (filter state, fix, arm/phase); euler ×3 i16 (0.01°); gyro ×3 i16
  (0.1 dps); accel ×3 i16 (0.01 g); vel NED ×3 i16 (0.1 m/s); alt AGL i16
  (0.5 m); lat/lon i32 (1e-7°); alt MSL i16; sats u8; hacc u8 (0.1 m, cap);
  health u8; canard deflections ×2 i8 (0.5°) + ctl mode u8 (zeros until Part
  2 ships); frame counter feeds the base station's loss statistic.
- **`M` message frame** (opportunistic, queued): the existing `msg` text
  records, truncated to 48 B — cal/magcal/nav events reach the field screen.
- **`C` uplink command frame**: ASCII `$cmd …` line ≤ 48 B + rolling seq.
  Feeds the **same `execCommand()`** as USB (refactor it to take a source
  tag); every command still answers with a `msg` (→ `M` frame). Duplicate
  seq = duplicate delivery → dropped.
- **MAC**: rocket is TX-master on a fixed 250 ms cadence; after each TxDone
  it sits in RX-continuous until the next slot. Base transmits only after
  hearing a downlink frame (listen-after-talk) → no collisions with the
  rocket's own TX, no time sync needed.

### 1.4 Rocket firmware changes

- **`sx1278.h/.cpp` — owned driver, portable** (repo precedent: hand-written
  drivers). Register-level LoRa init, FIFO tx/rx, IRQ-flag poll, RSSI/SNR,
  runs on an injected pin/SPI shim so the identical file compiles on
  stm32duino and ESP32 Arduino.
- **`link.h/.cpp`** — frame pack/parse, downlink scheduler, msg queue, uplink
  → execCommand. Pure logic where possible (host-testable pack/parse/CRC).
- **Loop integration**: new cooperative slot (`next_radio_`, COST_RADIO_US
  ≈ 200): poll DIO0 GPIO (free) → IRQ-flags SPI read only when set or when a
  TX slot is due. All SPI transactions bounded well under the IMU slack
  window. **No interrupts** — consistent with the poll-everything scheduler.
- **Snapshot refactor**: `buildStateRecord` and the frame packer both read
  one `TelemSnapshot` struct filled in the 50 Hz telemetry slot (no double
  sampling).
- **New USB telemetry fields** (SCHEMA + parse_test): `lre` (radio present),
  `rssi`, `snr` (last uplink), `ltx`/`lrx`/`lcrc` counters. The viewer's
  Comms tab was built for exactly these — it auto-populates.
- Config block: `LORA_FREQ_HZ 433.5e6`, `LORA_SF 7`, `LORA_BW 500k`,
  `LORA_TX_DBM 17`, `LORA_DOWNLINK_HZ 4`, pins as §1.1.
- New commands: `$lora?` (status/counters msg), `$lora 0|1` (mute radio).

### 1.5 Base station firmware (`basestation/` — new sketch, ESP32-C3)

Transparent bridge, no UI of its own:

1. RX LoRa frame → decode → emit **NDJSON** on USB CDC: `T` → a slim `st`
   record (same field names, missing fields simply absent — the viewer
   already tolerates that) **plus `rssi`, `snr`, `loss`, `lqi` measured at
   the base**; `M` → `msg` records; emits its own `hdr` record with
   `mode:"lora"`, `ohz:4` on connect.
2. Viewer console input `$…` lines → `C` frames (with retry-on-no-ack ×2).
3. Base-local diagnostics as `msg` records (`base: …`).

Viewer connects to the ESP32's port instead of the rocket's — zero viewer
protocol work. Arduino core: `esp32:esp32:esp32c3`, USB CDC on boot.

### 1.6 Viewer changes (small)

- Comms tab: bind rssi/snr/loss/rate charts and Link table to the new fields
  (placeholders already exist); "LINK: USB | LoRa" badge from `hdr.mode`.
- Staleness thresholds keyed off `hdr.ohz` (4 Hz ≠ stale at 300 ms).
- Ctrl tab already sends `$…` — works over either link unchanged.

### 1.7 LoRa acceptance

- Bench, both radios: 30 min soak — downlink loss < 2%, every `$ping` from
  the viewer answered via RF, RSSI/SNR plotted live.
- USB unplugged entirely: field screen (attitude, altitude, map, console)
  fully live at 4 Hz through the base station.
- Walk test ≥ 300 m: link holds, loss stat honest.

---

## Part 2 — Active canard control (PID)

### 2.0 Scope and safety posture

Roll axis first, alone: **roll-rate damping, then roll-angle hold**. Pitch/yaw
stay passive (fins). Rate control needs only the gyro — it works even while
the magnetometer redesign (HANDOFF_MAG.md) is unfinished; absolute-heading
roll targets wait for a healthy mag. The ESKF remains exactly as is — the
control layer consumes its outputs and never feeds anything back into it.

Minimal arming/phase machine, control-layer only:

`IDLE` →(`$arm`, 2-step)→ `ARMED` →(launch detect: |a_x| > 3 g for 100 ms)→
`ACTIVE` →(tilt > 60° OR descending OR t > 20 s)→ `SAFE` (canards centered,
disarmed). `$disarm` from anywhere. On any sensor-health failure of the IMU →
`SAFE`. `$magcal`/`$magclr`/`$cal` refused while ARMED/ACTIVE (the flash-save
freeze and re-alignment must never run mid-flight).

### 2.1 New firmware modules

- **`pca9685.h/.cpp`** — owned driver for the PCA9685BS, the 16-channel
  12-bit I2C PWM controller that drives all 4 canard servos. It sits on the
  **shared I2C1 sensor bus** (PB6/PB7 @ 400 kHz, default address 0x40 —
  confirm straps). No WHO_AM_I: the boot probe is a MODE1 read
  (`pwm PCA9685 @0x40: OK / NOT FOUND` in the startup log). Init sets
  auto-increment + prescale (PWM frame rate); API: `setServoUs(ch, us)`,
  `centerAll()`, `pulsesOff()` (FULL_OFF bit → pulses stop, servos limp).
  Write-failure counter surfaces in telemetry health.
- **`servos.h/.cpp`** — mapping layer on top: canard index → PCA channel,
  per-fin center/direction/throw, hard deflection clamp ±15°, slew limit
  400°/s, `center()` on SAFE and at boot. Bench commands (IDLE only):
  `$servo <n> <us>`, `$servo center`, `$servo off`.

  Actuation numbers that shape the design (servos are now bus traffic):
  - **Bus cost**: one auto-increment write updates all 4 channels (17 bytes
    ≈ 425 µs at 400 kHz). Commands latch at `SERVO_WRITE_HZ` (default 50 —
    writing faster than the PWM frame is pointless) and only when a channel
    moved ≥ 1 LSB → ~2% bus load, one bounded ~500 µs transaction per cycle
    in its own cooperative slot (`next_servo_`, `COST_SERVO_US 600`). The
    400 Hz IMU cadence is untouched.
  - **Resolution**: 12 bits across a 20 ms frame = 4.88 µs/LSB ≈ 205 steps
    over a 1000–2000 µs throw ≈ 0.15° of canard for a typical linkage —
    adequate.
  - **Latency**: 50 Hz write + 50 Hz PWM frame ⇒ up to ~40 ms command →
    pulse. At a 2–4 rad/s roll crossover that is 5–9° of phase — tolerable
    but real. If the servos tolerate a faster frame (digital servos:
    200–333 Hz), `SERVO_FRAME_HZ`/`SERVO_WRITE_HZ` config cuts it to
    ~10 ms. **The 6DOF gain validation must model this transport delay
    either way.**
  - **Failure mode**: the PCA9685 free-runs, so on a wedged bus the servos
    **hold their last commanded deflection** (no flail, no slam). Sustained
    write failures while ACTIVE: keep computing, flag health, resume
    cleanly on bus recovery (the integrator anti-windup covers the outage).
    The sensors module's bus-clear waveform (SCL toggling with SDA high)
    contains no START condition — the PCA9685 ignores it.
  - **SAFE behavior**: hold center (aerodynamically neutral) by default;
    `pulsesOff()` after a config delay is available for ground safing.
- **`control.h/.cpp`** — pure, host-testable (like eskf/magfit):
  - 400 Hz tick from the IMU slot, output latched at servo rate.
  - Rate mode: PI on roll-rate error (gyro x, bias-corrected from ESKF).
  - Angle mode: P on roll-angle error (ESKF roll) cascaded onto the rate
    loop; integrator clamped (back-calculation anti-windup).
  - Mixer: signed distribution to N canards (config matrix).
  - All gains/limits in the config block; live-tunable via
    `$ctl kp ki kd …` (RAM) for bench work.
- **Phase machine** in `control.cpp` (not in the filter, not in sensors).

### 2.2 Gain derivation path (the 6DOF asset)

`~\canardRocket6DOF` (Simulink, verified sign conventions, MATLAB MCP
available) supplies roll inertia and canard torque effectiveness vs airspeed.
Sequence: extract I_x and C_l_δ → classical loop shaping for the rate loop at
expected max-q → validate in the 6DOF with the exact discrete PID + slew +
clamp → port the same constants into `control.cpp` config → replay the same
step scenarios in `control_test.cpp` (simple 1-DOF roll plant) and require
matching response envelopes.

### 2.3 Telemetry / viewer / schema

- `st` additions: `cmode` (IDLE/ARMED/ACTIVE/SAFE), `cdef` [N] (deg),
  `crll`/`ctgt` (rate & target), saturation flag. SCHEMA + parse_test.
- Downlink `T` frame carries mode + 2 deflections (already reserved).
- Viewer Ctrl tab: ARM (2-step, red, like Reboot), DISARM, mode badge in the
  topbar (ACTIVE = inverted), deflection readouts; gains form → `$ctl`.
- Every phase transition emits a `msg` (`ctl: ARMED`, `ctl: ACTIVE @ t+…`).

### 2.4 Control acceptance

- Host: `control_test.cpp` — step responses within 6DOF-matched envelopes,
  anti-windup holds under saturation, mixer signs, full phase-machine
  transition table including sensor-fault → SAFE.
- Bench: `$arm` refused without 2-step; hand-rotate board in ACTIVE (bench
  jig) → canards oppose roll with correct sign at sane deflection; kill
  switch (`$disarm`) instant; USB pull mid-ACTIVE → SAFE via health;
  unplugging the PCA9685 mid-ACTIVE → health flags it, servos hold, and
  recovery on reconnect is clean.
- 6DOF: gust + spin-up disturbance rejected, deflections stay off the clamps
  ≥ 80% of flight.

---

## Phases (each ends compiled + host-tests green + parse_test green + SCHEMA current)

| # | Deliverable | Proof |
|---|---|---|
| 0 | Owner inputs (below) + pin audit already done (PA4/5/6/7, PB0, PC4 free) | — |
| 1 | `sx1278` driver + rocket radio slot + `$lora?` + rssi/snr in `st` | two desk radios exchange test frames; counters in viewer |
| 2 | `link` protocol + base-station bridge sketch | viewer runs entirely over LoRa on desk |
| 3 | Viewer Comms tab live + link-mode staleness | walk test, loss stat |
| 4 | `pca9685` + `servos` modules + bench commands | `$servo` sweep moves all 4 canards |
| 5 | `control` module + host tests + 6DOF gains | control_test green, 6DOF traces |
| 6 | Phase machine + ARM UI + full dress rehearsal (LoRa + control, motor-less) | end-to-end bench log |

## Phase 0 — needed from the owner

1. **Servo/PWM hardware**: PCA9685 address straps (0x40 assumed); whether
   /OE is wired to a GPIO or tied to GND (a GPIO gives a hardware kill for
   all pulses — worth knowing); which of the 16 channels drive which canard;
   servo model (analog/digital — maximum PWM frame rate it tolerates); V+
   servo supply rail (source/BEC current headroom — 4 servos stalling
   together is the worst case); linkage direction and mechanical throw per
   fin.
2. **Antennas** on both Ra-02s (never TX unloaded — PA damage risk).
3. Ham callsign (or confirm none) for the beacon ID field.
4. Confirm the PCB routed SPI1 on PA5/PA6/PA7 (variant defaults assumed).
5. ESP32-C3 board variant (bare module vs devkit) for the base sketch's
   USB-CDC settings.

## Standing constraints that still apply

Owned drivers only (no RadioLib/LoRa libs); SCHEMA.md documents every emitted
field; `viewer/parse_test.js` stays green and grows with the schema; no
TODO/FIXME markers; plain-ASCII messages; ESKF architecture untouched and
phase-free; every round verified with arduino-cli + zig host tests + node
parse_test.
