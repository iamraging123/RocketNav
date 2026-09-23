# RocketNav — Full Context Handoff

Written 2026-09-13 for the next Claude session taking over this project.
Read this whole page before touching anything. Deeper history lives in the
files listed in section 11; this page is the map and the current truth.

---

## 0. Read-this-first (current state in one screen)

- **What it is:** STM32F722 flight firmware for a high-power rocket — 16-state
  ESKF navigation (GNSS/IMU/mag/baro), LoRa telemetry + ESP32-C3 base station,
  4-canard active roll control, and a single-file Web Serial ground station.
- **Repo:** `C:\Users\lgrue_borstjl\RocketNav` → GitHub
  `https://github.com/iamraging123/RocketNav` (public, branch `main`).
- **Live site (GitHub Pages):** project page
  `https://iamraging123.github.io/RocketNav/site/`, ground station
  `https://iamraging123.github.io/RocketNav/viewer/`.
- **Last verified build (2026-09-13):** firmware 145,492 B flash / 20,072 B
  RAM; host tests eskf 91, magfit 10, link 20, cfgcodec 16, control 35,
  pca 67, viewer parse 41 — **all green**.
- **Committed and pushed 2026-09-20 at the owner's request:** MG90S latency
  defaults, the one-point offset-cal fix, the LoRa bring-up round (compact
  `$lora?` reply; base station USB-matching null rules and a real
  listen-after-talk uplink - cued 30 ms after each downlink frame, blind
  after 400 ms), the viewer's LoRa bar and link-selector polish (section
  6), the site page refresh, and this file. The standing rule is unchanged:
  do not commit or push unless the owner asks.
- **LoRa link VERIFIED on the bench 2026-09-20** (first RF test ever): rocket
  → base 4 Hz state frames, seq 1→52, 0 % loss, 0 CRC errors; base → rocket
  `$ping` answered `cmd: pong` over RF; `$lora?` answered over RF. RSSI at
  the base −21…−23 dBm / SNR ~5 dB with the radios ~1 m apart (near
  front-end saturation - real numbers come from the walk test). The base
  station on COM6 was flashed by arduino-cli that day; the rocket ran the
  firmware as of 2026-09-13 (LoRa code unchanged since 08-30).
- Rocket reflashed by the owner on 2026-09-20 with all of the above (the
  `hdr` line was confirmed); base station on COM6 flashed the same day.
  The offset-cal bench confirmation (section 9.1) is still owed.
- **2026-09-21 (evening), uncommitted, NOT yet on the air:** the two-profile
  range scheme of PLAN_LORA_RANGE.md is implemented end to end (codec,
  driver, rocket MAC, base station, viewer, SCHEMA) - see 5.6 and PLAN §7b.
  The `T` frame is now 54 bytes; the base still parses the old 53-byte one,
  so **flash the base first, then the rocket, in any order that suits**.
  The viewer also gained the LoRa declutter (`uo` class), link-quality
  colours, the euler→quaternion rebuild, and the freeze fix (section 6).
  Bench procedure to run next: PLAN §6 steps 1–3.
- **Top open items:** section 9. The most urgent: confirm the owner is
  flashing the RIGHT sketch (section 2.3 — a wrong-sketch upload already
  happened once), confirm the offset-cal fix on the bench, watch the MG90S
  servos at the new 100 Hz frame rate.

---

## 1. The owner and how to work with them

- Hands-on rocketry/embedded engineer. Iterates fast, types tersely (typos
  are normal — read intent), reports bench results in short bursts, often
  pastes raw serial output. Fine with dense technical explanations.
- **Every change round must end with an explicit reflash/refresh callout**
  (their standing request):
  - anything under `firmware/RocketNav/` → **reflash the rocket**
  - anything under `basestation/` → **flash the ESP32-C3 base station**
  - `viewer/index.html` only → hard-refresh the page (Ctrl+F5); for the
    GitHub Pages copy it also needs commit + push (~30 s redeploy)
  - docs/tests only → nothing to do
  - remind when relevant: flash-stored calibrations (mag hard-iron, linkage
    tables, flash sector 7) **survive re-uploads**.
- **Owned code only.** Hand-written drivers are the house style (ICM-45686,
  LIS3MDL, PCA9685, SX1278). No AHRS/fusion/calibration/radio libraries.
- **Viewer taste:** dense "engineer-brutalist" — square corners (no
  border-radius), one monospace font (Consolas stack), maximum data per
  pixel, no placeholder/explainer prose, charts in RGB, dark + light themes.
  They have rejected several redesigns within a day; keep the viewer's
  engine/presentation split clean so reskins stay cheap.
- When a request is ambiguous about WHICH element to swap/remove, restate the
  change in one line before building (a past misread cost a round).
- They use the **Arduino IDE** for uploads now (also arduino-cli available).
- Git attribution: every commit message must end with the session
  attribution footer provided by the harness.

---

## 2. Machine, toolchain, flashing

### 2.1 Environment
- Windows 11, PowerShell primary (Git Bash also available). No `make`.
- **Never pass patch scripts through bash heredocs** (backslash escapes and
  newlines get mangled; lines after a heredoc terminator run unconditionally).
  Use the Write/Edit tools for all file changes. This bit past sessions 5+
  times.

### 2.2 Build and test commands (exact, verified)
Firmware (from anywhere):
```
arduino-cli compile --fqbn STMicroelectronics:stm32:GenF7:pnum=GENERIC_F722RETX --output-dir C:\Users\lgrue_borstjl\RocketNav\firmware\build C:\Users\lgrue_borstjl\RocketNav\firmware\RocketNav
```
stm32duino core 2.12.0. Output hex/bin in `firmware/build/` (gitignored).

Host tests — zig 0.16 is the only native C++ compiler and is NOT on PATH:
```
$zig = "$env:LOCALAPPDATA\Microsoft\WinGet\Packages\zig.zig_Microsoft.Winget.Source_8wekyb3d8bbwe\zig-x86_64-windows-0.16.0\zig.exe"
# run from RocketNav\test
& $zig c++ -std=c++17 -O2 -I../firmware/RocketNav -o eskf_test.exe eskf_test.cpp ../firmware/RocketNav/eskf.cpp ../firmware/RocketNav/nav_frames.cpp
& $zig c++ -std=c++17 -O2 -I../firmware/RocketNav -o magfit_test.exe magfit_test.cpp ../firmware/RocketNav/magfit.cpp ../firmware/RocketNav/nav_frames.cpp
& $zig c++ -std=c++17 -O2 -Imock -I../firmware/RocketNav -o link_test.exe link_test.cpp ../firmware/RocketNav/linkcodec.cpp
& $zig c++ -std=c++17 -O2 -I../firmware/RocketNav -o cfgcodec_test.exe cfgcodec_test.cpp ../firmware/RocketNav/cfgcodec.cpp ../firmware/RocketNav/linkage.cpp
& $zig c++ -std=c++17 -O2 -I../firmware/RocketNav -o control_test.exe control_test.cpp ../firmware/RocketNav/control.cpp
& $zig c++ -std=c++17 -O2 -Imock -I../firmware/RocketNav -o pca_test.exe pca_test.cpp ../firmware/RocketNav/pca9685.cpp ../firmware/RocketNav/servos.cpp ../firmware/RocketNav/linkage.cpp
node ..\viewer\parse_test.js
```
(Some test-file header comments show older build lines — e.g. pca_test's
omits `linkage.cpp` and will fail to link. Use the lines above.)
`M_PI` is absent under zig — tests define a local `kPi`.

Viewer checks: syntax-check both `<script>` blocks (extract and
`new Function()` them in node), then run Demo mode in Chrome. To serve
locally: `python -m http.server 8137` from the repo root →
`http://localhost:8137/viewer/` (the Chrome extension refuses `file://`).
Kill the python server afterwards; zombie servers on 8137 have caused
false 404s — curl for a 200 before blaming the page.

### 2.3 Flashing — and the WRONG-SKETCH HAZARD
- The owner uploads from the **Arduino IDE**. The correct sketch is
  **`C:\Users\lgrue_borstjl\RocketNav\firmware\RocketNav\RocketNav.ino`** —
  it is OUTSIDE the IDE sketchbook.
- The sketchbook `Documents\Arduino\` contains two OLD dead projects with
  near-identical names: **`RocketNavF7`** and **`RocketNavEKF`**. On
  2026-09-12 the owner accidentally flashed RocketNavF7 and reported "not
  starting up". Tell-tale: its stream looks like `{"r":"st","t":86175487,
  "n":...,"hl":{...},"lpx":...,"flt":0}`. The real firmware emits
  `{"t":"hdr",...}` then `{"t":"st","us":...,"seq":...}`. If the owner pastes
  output with `"r":"st"`, `drec`, `lpx`, or `flt`, it is the wrong binary —
  not a firmware bug. (Renaming those folders to `*_OLD` was offered, not
  done.)
- Right-sketch check in the IDE: tabs include `lis3mdl.cpp`, `servos.cpp`,
  `control.cpp`. Board: Generic STM32F7 series → Generic F722RETx.
- Also installed: STM32 ST-LINK Utility CLI
  (`C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe -c SWD -P <hex> -V -Rst`).
  Whether STM32CubeProgrammer is installed now is unknown (it was not on
  2026-08-29; `arduino-cli upload` needs it on this core).
- Serial (verified 2026-09-20): rocket telemetry UART = **COM5** (FTDI
  FT232, 921600 Bd; the STM32 itself never enumerates a COM port). **Base
  station ESP32-C3 = COM6** (Espressif native USB, VID 303A) - Claude can
  flash it: `arduino-cli upload -p COM6 --fqbn
  esp32:esp32:esp32c3:CDCOnBoot=cdc --input-dir <build> basestation`.
  **COM9 is a separate ESP32-C3 LED-controller device - NOT the base station,
  never flash or reset it** (it enumerates identically; only the port number
  tells them apart). COM3 is an ESP32-S3 display project; COM14/15 Bluetooth.

### 2.4 GitHub
- Account `iamraging123` (gh CLI logged in, token has repo/workflow scopes).
- Pages builds from `main` at repo root; root `index.html` redirects to
  `site/`. Public repo is required for free Pages.
- `.gitignore`: `firmware/build/`, `test/*.exe`, `test/*.pdb`.
- `.gitattributes`: `viewer/replay.jsonl -text` — parse_test slices that
  file at fixed BYTE offsets; newline conversion would break the test.
- Do not push without the owner's say-so. One commit exists (initial).

---

## 3. Hardware

| Item | Detail |
|---|---|
| MCU | STM32F722RETx, 216 MHz, single-bank flash (512 KB) |
| I2C1 @ 400 kHz | SDA PB7, SCL PB6 — shared by ALL of the below |
| IMU | TDK ICM-45686 @0x68, 400 Hz ODR, polled at 440 Hz (no DRDY wire), ±32 g / ±2000 dps, UI LPF ODR/4 |
| Magnetometer | ST LIS3MDL @0x1C, UHP 80 Hz, ±4 gauss, no DRDY (STATUS poll 100 Hz) |
| Baro | Bosch BME280 @0x77, polled 25 Hz |
| GNSS | u-blox **SAM-M8Q** @0x42 (DDC). NOT an M10: SparkFun v3 lib cannot talk to it; uses SparkFun u-blox GNSS Arduino Library **v2 (2.2.28)** |
| Servo PWM | PCA9685BS @0x40 (address straps unverified), on the same I2C bus |
| Servos | **4× MG90S** (analog, metal gear, ~0.1 s/60°), canards on PCA ch 0–3 (map assumed; `$ang scan` finds the real one) |
| LoRa | Ai-Thinker Ra-02 (SX1278) on SPI1: SCK PA5, MISO PA6, MOSI PA7, NSS PA4, RESET PC4, DIO0 PB0 (DIO1–5 not connected) |
| Base station | ESP32-C3 + Ra-02: MOSI 5, MISO 6, SCK 7, NSS 0, RESET 4, DIO0 1 |
| LEDs | WS2812B strip a: PC10 ×2; strip b: PC13 ×4 (bit-banged) |
| Telemetry UART | `Serial` at 921600 Bd |

Mount rotations (sensor → body; body X = nose):
- IMU: `R_MOUNT_IMU = [0 0 -1; 1 0 0; 0 -1 0]` — verified; do NOT change
  without a two-orientation static check through a verified display.
- Mag: `R_MOUNT_MAG = [0 0 -1; -1 0 0; 0 1 0]`.

Site field (Champaign IL, IGRF-14): NED {20.15, −1.23, 47.77} µT,
|B| 51.86 µT, declination −3.50°, inclination +67.09°. Only ~20 µT is
horizontal: every µT of uncorrected horizontal iron ≈ 3° of heading.

Radio profile: 433.5 MHz, SF7 / BW 500 kHz / CR 4/5, sync 0x4B, preamble 8,
17 dBm PA_BOOST. **Downlink boots MUTED** (`LORA_TX_AT_BOOT 0`) — an
antenna-less Ra-02 must never transmit. `$lora 1` unmutes.

---

## 4. Repo map

```
RocketNav/
  HANDOFF.md            this file
  HANDOFF_MAG.md        magnetometer redesign brief + round-1 addendum + bench procedure
  PLAN_LORA_CONTROL.md  LoRa link + canard control plan (implemented)
  PLAN_LORA_RANGE.md    range budget, profile/airtime tables, two-profile
                        recovery design + walk-test plan (2026-09-21, NOT implemented)
  PLAN_LINKAGE_CAL.md   linkage calibration plan (implemented)
  DESIGN.md             ESKF math: states, exact Phi, Q, every measurement model
  SCHEMA.md             telemetry + command CONTRACT (every emitted field documented)
  README.md, index.html root landing + Pages redirect
  firmware/RocketNav/
    RocketNav.ino       config block (ALL tunables), scheduler, command handlers, boot
    eskf.h/.cpp         16-state ESKF (pure, host-tested)
    nav_frames.h/.cpp   geodetic/NED/quat/DCM helpers
    sensors.h/.cpp      acquisition, validation, stuck/stale detection, mag cal, bus recovery
    icm45686.h/.cpp     owned IMU driver
    lis3mdl.h/.cpp      owned mag driver (+ $magdiag register probe)
    magfit.h/.cpp       least-squares sphere fit for hard iron (pure)
    cfgcodec.h/.cpp     flash record codec v1/v2 (pure, host-tested)
    cfgstore.h/.cpp     HAL flash I/O, sector 7 (0x08060000)
    linkage.h/.cpp      servo-µs <-> canard-deg tables (pure)
    pca9685.h/.cpp      owned PWM driver
    servos.h/.cpp       canard mapping, bench modes, write-on-change scheduler
    control.h/.cpp      roll controller + arming/phase machine (pure)
    sx1278.h/.cpp       owned LoRa driver (portable stm32duino/ESP32)
    linkcodec.h/.cpp    LoRa air codec + CRC (pure)
    link.h/.cpp         rocket-side LoRa MAC
    telemetry.h/.cpp    NDJSON writer with non-blocking ring buffer
    leds.h/.cpp         WS2812B bit-bang
  basestation/          ESP32-C3 bridge sketch; sx1278.* and linkcodec.* are
                        COPIES of the rocket files — keep them in sync
  test/                 host tests + mock/{Arduino.h,Wire.h} recording I2C mock
  viewer/index.html     single-file ground station (~2.4 kB lines)
  viewer/parse_test.js  node contract test over the viewer's PARSE-CORE
  viewer/replay.jsonl   canned stream for parse_test (byte-offset sensitive)
  site/index.html       project page (1960s spec-sheet design)
```

---

## 5. Firmware architecture

### 5.1 Scheduler (RocketNav.ino `loop()`)
Cooperative, poll-everything, no interrupts. The IMU poll owns the loop
(440 Hz). Between IMU due-times exactly ONE secondary task runs, and only if
its worst-case cost fits the remaining slack (margin 150 µs). Priority order
(earlier wins when due): **telemetry (700 µs) → service 10 Hz (600) → radio
250 Hz (400) → GNSS 100 Hz (1200) → mag 100 Hz (500) → baro 25 Hz (800) →
servo (700)**. Telemetry and the watchdog service deliberately outrank sensor
I/O: a sick bus turns Wire ops into long timeouts that would otherwise starve
the stream exactly when its health fields are the only diagnostic.
`pollCommands()` (USB) and `link::popCommand()` (LoRa uplink) feed the same
`execCommand()` every pass.

The IMU branch also runs `control.tick()` at the sample rate
(dt = 1/IMU_ODR_HZ — NOT the poll period) and pushes deflections to
`servos.setDeflDeg()` only in ACTIVE, BENCH, or SAFE (IDLE belongs to bench
commands).

The 10 Hz service slot: sensor watchdogs/reinit, orchestrate() (startup
messages, baro reference, GPS auto decision), LEDs, **PCA9685 recovery
probe with backoff**, magcal progress/completion + flash save.

### 5.2 Sensors pipeline
raw counts → SI → mount rotation → (mag) hard-iron subtract → soft-iron
(identity) → validity gates → filter. Every sensor has stuck detection,
staleness timeouts, and reinit with exponential backoff; `busClear()` clocks
out a wedged slave after 10 consecutive I2C errors. Health bitmask per sensor
(present 1, fresh 2, stale 4, sat 8, stuck 16, fault 32).

### 5.3 ESKF (see DESIGN.md for the math)
- 16 error states `[δp δv δθ δbg δba δbb]`, float32, Joseph-form updates,
  **GLOBAL (NED-frame) attitude error** — hard constraint, do not change.
- F is nilpotent (F⁴ = 0) → the closed-form Φ is the EXACT exponential.
- Filter states: 0 init, 1 align, 2 wait-fix, 3 run, 4 attitude-only
  (GPS auto mode: no fix within 20 s → att-only; a later fix upgrades 4 → 3).
- GNSS fuses **horizontal only** (2-dof); altitude is baro+IMU exclusively;
  `bb` is pinned (the baro defines the vertical datum).
- Aids: GNSS pos/vel (latency-compensated), baro, full 3-axis mag vector
  update (`H = [B_ref]×`, B_ref captured at alignment), stillness-gated
  gravity tilt + zero-angular-rate (ZARU) updates.
- The filter is **phase-free**: no flight-phase logic ever goes in eskf or
  sensors. Control consumes filter outputs and never feeds back.

### 5.4 Roll control (control.h/.cpp)
- Cascade: inner **rate PI** on gyro roll rate (always); optional outer
  **angle P** turning roll error into a rate command. `angle_mode` boots
  FALSE (rate damping = first-flight configuration, no mag dependence).
- Conditional-integration anti-windup, slew limit, per-canard mix signs.
- Phases: `IDLE(0) → ARMED(1)` via 2-step `$arm`/`$arm yes` within 5 s →
  `ACTIVE(2)` on |a_x| > 3 g for 100 ms → `SAFE(3)` on tilt > 60°, descending
  (v_D > 3 m/s), or 20 s. `BENCH(4)` = ground roll-hold test via `$rolltest`
  (same law, no launch gate, flight SAFE triggers disabled; IMU fault still
  → SAFE). `$disarm` from anywhere → IDLE. **The cmode integer is on the
  wire: append new modes, never reorder.**
- Current defaults (RAM-tunable with `$ctl`): `CTL_KP_RATE 0.08`,
  `CTL_KI_RATE 0.05`, `CTL_KP_ANG 8.0`, rate cmd max 180 dps,
  `CTL_DEFL_MAX 10°`, `CTL_SLEW_DPS 600` (= MG90S top speed). Gains are bench
  placeholders; the flight pass comes from the Simulink 6DOF at
  `C:\Users\lgrue_borstjl\canardRocket6DOF` (verified sign conventions).
- Authority = min(CTL_DEFL_MAX, smallest linkage-table end) — ONE helper,
  `applyAuthority()`, used at boot and after every linkage change.

### 5.5 Servos, linkage, persistence
- `SERVO_FRAME_HZ 100` (MG90S analog envelope; spec point is 50, digital-only
  range 200–333). `SERVO_WRITE_PER_FRAME 4` → writes every 2.5 ms, only on
  change ≥ ~0.9 LSB. Command→pulse worst case ≈ 15 ms electrical.
- Pulse guard 900–2100 µs. `SERVO_US_PER_DEG {10,-10,10,-10}`,
  `CTL_MIX_SIGN {1,1,1,1}` — **signs not yet bench-verified**.
- Per-fin linkage table: up to 5 (canard°, servo µs) points, piecewise-linear,
  table ends = mechanical limits. 1 staged point = offset-only table (default
  gain through the measured neutral). Uncalibrated fins synthesize a 2-point
  default. `lcal_fins_` bitmask = which fins have an applied calibrated table.
- Flash: sector 7, record `RNV2` {mag hard-iron[3], FinRec[4], CRC32},
  whole-record rewrite (mag and linkage never clobber each other); loader
  migrates `RNV1`. **Every save freezes the whole chip ~1–2 s** (single-bank
  erase) — refused while ARMED/ACTIVE/BENCH. `$lcal flash` reads the physical
  sector past the cache (proof of persistence after a power cycle).
- Bench modes in `Servos`: sweep (6 s), channel scan (`$ang scan`, 19 s),
  fin test (`$cang test`, +10/−10/0°), per-fin release, all-off.

### 5.6 LoRa
**Two profiles since 2026-09-21 (PLAN_LORA_RANGE.md §4, §7b):** FLIGHT =
SF7/500k, 54-byte `T` frame at 4 Hz; RECOVERY = SF11/125k, 27-byte `B`
beacon every 10 s. The rocket switches itself to RECOVERY when SAFE + still
30 s, or 20 s after the base's `K` keepalives (5 s) stop - never in ACTIVE,
never muted; `$lora rec|flight` forces. The base follows by listening
(10 s quiet → recovery, 30 s → scan 10 s/10 s), stays on FLIGHT until it has
ever heard the rocket, sends a command's repeat on the other profile when
unsure, and `$base flight|rec|auto` pins/frees it. Modem switch =
`Sx1278::setModem()`; LowDataRateOptimize is automatic above 16 ms symbols.
**On the air 2026-09-21 (bench):** `$lora rec` over RF → base followed
after its 10 s rule → beacons decoded every 10 s at SF11 (rssi −26, snr
12.8) → `$lora flight` back. The base's blind uplink repeat scales with
the profile (`blindMs()`: 400 ms FLIGHT, 2.5 s RECOVERY) because on SF11 a
640 ms command frame plus the rocket's immediate reply must clear before
the repeat, or the repeat talks over the reply (that lost the first pong).
Also verified the same evening: the rocket's keepalive-loss rule (base pinned
to recovery → rocket moved itself to RECOVERY ~20 s after its last
keepalive → base decoded its beacons), the base's scan (found a rocket that
came back on FLIGHT), and `$lora flight` over SF11. Still owed from PLAN
§6: the walk test and the ground test (the only ones that need distance).
Rocket is TX-master every 250 ms (54-byte binary `T` state frame; `M` text
frames alternate slots from a 4-deep msg queue); base listens-after-talk and
sends `C` uplink command frames (twice; rocket dedupes by seq). **The uplink
cue is RxDone + 30 ms (`UPLINK_CUE_MS`), not RxDone itself:** the rocket
only learns of its own TxDone at its next 250 Hz radio poll and re-arms RX
then, so an uplink started the instant its frame ends is lost (bench: 6
pings → 1 delivery). No downlink cue within 400 ms (`UPLINK_BLIND_MS`,
rocket muted) → the frame goes blind, so `$lora 1` reaches a silent rocket.
Never move the cue earlier without re-running the 6-ping test. **DIO0 is
advisory:** `Sx1278::poll()` also reads RegIrqFlags every 20 ms
(`kFlagPollMs`), because on 2026-09-20 the base station went completely
deaf (TX fine, rx 0 for minutes) with the symptoms of a loose DIO0 wire on
GPIO1. On 2026-09-21 the `dio0 N timer M` counters in `$base?` (how each
reception was discovered) read `dio0 39 timer 0`, so the base's DIO0 net
is fine and the fallback is insurance. The deaf episode coincided with an
RF-path collapse instead: the rocket logged the base's uplink at
**−106 dBm / SNR −11.8 dB** (normally −20 dBm on the bench), and the next
run showed 2 CRC errors in 36 frames where every earlier run had 0 -
i.e. an intermittent antenna connection (u.FL seating / spring-antenna
joint) on one of the Ra-02s while the owner was handling the boards.
Watch the LoRa bar's rssi and air-loss before blaming firmware. `$base
regs` shows the DIO0 level next to the raw flag register. The base
station translates everything to the same NDJSON the viewer speaks and stamps
`hdr.mode:"lora"`. Viewer topbar has a USB|LoRa selector; LoRa display is
4 Hz by design — judge responsiveness over USB.

### 5.7 Telemetry records (SCHEMA.md is authoritative)
NDJSON, 50 Hz `st` records (~1.25 kB each, ~62 kB/s of the 92 kB/s link),
plus `hdr` (boot), `msg` (status/replies, plain ASCII, **no JSON escaper —
never pass dynamic strings with quotes/backslashes**), `cal` (alignment
result), `org` (origin anchor), `lcal` (linkage tables; key `pus` not `us`).
Recent `st` additions: `mgr` (raw mag), `lre/rssi/snr/ltx/lrx/lcrc`,
`cmode`, `cdef[4]`.

### 5.8 Command set (all documented in SCHEMA.md "Command input")
`$ping $cal $magcal $magclr $magdiag $zero $led $rst $sens? $lora 0|1
$lora? $arm $arm yes $disarm $rolltest $ctl <kpr> <kir> <kpa>
$ctlmode rate|angle $sframe <hz> $servo ... $cang ... $ang ... $lcal ...`
Servo/canard/lcal/sframe commands are IDLE-only. `$cal/$magcal/$magclr`
refused unless IDLE or SAFE.

---

## 6. Ground station (viewer/index.html)

- Single file, no build, Web Serial (Chrome desktop). Works from `file://`
  and from GitHub Pages. Network used only for Esri satellite tiles and the
  site page's Google Fonts.
- Layout: left column 3D attitude (+ STL upload) over pad-anchored satellite
  map; right column tabs **Main | Ctrl | Comms**: readout cards, charts
  (altitude, attitude, gyro, mag raw, accel dial), dense tables (Filter,
  Sensors, Health, GNSS, Device, startup log), console with typed command
  line. Ctrl tab: servo/canard/linkage-cal panels, control (arm/rate/angle/
  gains/roll test), system commands.
- **LoRa bar (2026-09-20):** a second topbar row, shown when the selector
  is LoRa or the stream's `hdr.mode` is `lora`: downlink LIVE/QUIET/MUTED?
  badge (LIVE = frame within 2 s; MUTED? before the first frame, since the
  rocket boots muted), Unmute/Mute/Ping/Rocket link/Base buttons
  (`$lora 1/0`, `$ping`, `$lora?`, `$base?` - disabled until connected),
  rssi/snr/air-loss/frame-rate/ping-RTT badges. Ping RTT is timed from
  `$ping` to the `cmd: pong` msg on any link (shared with the Ctrl-tab
  Ping). The Connect button names the link ("Connect USB"/"Connect LoRa");
  USB mode filters the chooser to FTDI 0x0403, LoRa mode to Espressif
  0x303A, both fall back to the full chooser on cancel. Comms Link table
  gained "air loss (base)" (the base station's `loss`) and "ping round
  trip". Badges/device panel tolerate the base hdr (no `mmode`/`gmode`,
  `schema` instead of `sch`). `sendCmd` logs a note past 48 chars on LoRa.
- **LoRa records are a SUBSET of the USB record (2026-09-21 freeze):** the
  base station's `st` has no `q/sa/bg/ba/mag/mgr/pa/tc/glat/glon/galt/gage/
  grz/fhz/lmx/dimu../dtx/ei2c/lre/ltx/lrx/lcrc`. A USB record always carries
  every key (null when unknown), so `r.x === null ? "—" : r.x.toFixed()`
  guards let `undefined` through and `updateDom` threw on `r.glat.toFixed`
  - and because `frame()` armed the next `requestAnimationFrame` LAST, that
  one exception killed the render loop: "the page froze, I can't move the
  model". Rules now: `frame()` arms the next frame FIRST and wraps the body
  in try/catch (first error goes to the command log as "viewer draw
  error"); field guards are `== null` (null or undefined); raw `${r.x}`
  interpolations go through `nv()`; `loraStreamNow()` (hdr.mode or a
  `loss` key on the last record) decides LoRa presentation, because the
  base's boot-time hdr is often torn (the ESP32-C3 reboots when its port
  opens) - the base also repeats its hdr every 10 s now. The read loop backs
  off on data-less reader failures instead of re-arming in a hot loop.
  Test recipe: inject real base lines with `handleLine()` in Chrome, click
  every tab, call each draw function inside try/catch, and scan the DOM for
  "undefined"/"NaN". The 3-D model and compass read `q`, which the air
  frame lacks: the st handler rebuilds it with `eulToQuat(r.eul)` (the
  firmware's `quat_from_euler` ZYX construction, round-trip verified
  against its `quat_to_euler`), so LoRa attitude renders like USB.
- **Link-quality colours (2026-09-21, owner request):** rssi / margin /
  snr / air-loss badges on the LoRa bar and the Comms Link rows carry
  `good` (green, `--c2`) / `warn` (amber, `--c4`) / `bad` (red) classes
  from `gradeRssi/gradeSnr/gradeLoss`. Bands for the SF7/500 kHz profile:
  floor `LORA_FLOOR_DBM −117`; RSSI margin ≥ 20 dB green, 10–20 amber,
  < 10 red; SNR ≥ 0 green, −5..0 amber, below red (the packet SNR estimate
  saturates near +6 dB on a strong signal, so bench SNR of 5 dB at −21 dBm
  is normal, not weak); air loss < 1 % green, < 5 % amber, else red. The
  `margin` badge is the one number to watch on a walk test.
- **LoRa declutter (2026-09-21, owner request):** elements the air frame
  can never fill carry class `uo` (USB-only) and `body.lora .uo` hides
  them: Filter / Calibration-table / Origin / Linkage-cal panels, the mag
  raw chart, the baro-bias card, the mode / filter-Hz / gnss-Hz badges, and
  the sensor/health/GNSS-raw/device/link rows for mag, baro, biases, drops,
  i2c, raw fix, loop max, dtx, 1σ attitude. `body.lora` follows the STREAM
  when connected (a USB stream shows everything even with LoRa selected)
  and the selector when idle. The Ctrl-tab Calibration COMMAND panel stays
  (its commands work over RF). When a future frame carries a field, remove
  its `uo` tag - that is the whole contract. Likewise `fo` (flight-only:
  gyro chart, accel dial, vertical-rate / speed / vel-NED cards, acc and
  gyro sensor rows) hides under `body.recovery`, set while `hdr.prof` is
  `recovery`, because the beacon carries no rates, accel or velocity.
- **Chart history (2026-09-22):** `histSlider` (log, 5 s–1 h) sets every
  chart's `win`; `rn_hist` in localStorage remembers it; clicking the
  `history` badge returns to auto = 10 s, or 10 min on the 0.1 Hz recovery
  beacon. Ring capacity is 16 384 points (~5 min at 50 Hz USB, ~68 min at
  4 Hz). The record-rate window also stretches to three frame periods so
  the beacon reads 0.1 Hz instead of "—". The base's "downlink quiet"
  heartbeat now uses 25 s on RECOVERY (5 s on FLIGHT) so a 10 s beacon is
  not reported as quiet.
- **PARSE-CORE contract:** the code between `PARSE-CORE-BEGIN` and
  `PARSE-CORE-END` in script block 1 is extracted verbatim by
  `parse_test.js`. Keep the markers intact. Schema growth = extend
  `CSV_COLS`/`stToCsvRow` + `replay.jsonl` + parse_test checks.
- `sendCmd` chains all writes on one promise with `releaseLock` in `finally`
  (a leaked writer lock once killed every later command, Disarm included).
- Canvas rule: a canvas gets EITHER a fixed CSS height OR `contain: size` —
  never a bare percentage height in a content-sized track (resize feedback
  loop).
- Automation gotchas: the viewer is rAF-driven — in a background Chrome tab
  panels freeze at "—" while data still arrives (not a bug; a screenshot
  forces a frame). `resize_window` cannot resize a maximized window. Never
  trigger `prompt()`/`alert()` during browser automation.

---

## 7. Invariants — do not break

1. SCHEMA.md documents every emitted field and every command; parse_test
   stays green and grows with the schema.
2. ESKF: 16 states, global-NED attitude error, Joseph form, float32,
   phase-free. No flight-phase logic in eskf/sensors.
3. Owned drivers/code only. No TODO/FIXME/placeholder markers anywhere.
4. `msg` text is plain ASCII literals or snprintf of numbers — no escaper.
5. `ctl::Mode` values are wire values: append only.
6. `basestation/sx1278.*` and `basestation/linkcodec.*` must match the
   firmware copies byte-for-byte in behavior.
7. `linkage::finalize()` bounds (|deg| < 90, 500 < µs < 2500) MIRROR
   cfgcodec `finSane()` — anything that finalizes must survive reboot.
   Offset tables are pulled inside ±89° (`synthOffset`).
8. `Servos::takeOwnership()`: every path that takes the outputs (center,
   testUs, sweep/scan/fin-test start, off) cancels the other bench modes and
   releases the scan's wiggled channel. `releaseCanard` latches a fin limp.
9. `Pca9685::setFrameHz` commits `frame_hz_` (the µs→counts basis) only after
   the whole sleep→prescale→wake sequence succeeds; left asleep ⇒ report the
   chip absent. A frame change turns the whole bank off, then rewrites the
   canards.
10. `viewer/replay.jsonl` is byte-offset sensitive (`-text` in git).
11. Every round: arduino-cli compile + all zig suites + parse_test, and the
    reflash/refresh callout to the owner.

---

## 8. Verification baseline (2026-09-20)

| Check | Result |
|---|---|
| firmware | 147,276 B flash, 20,120 B RAM |
| base station (esp32c3, CDCOnBoot=cdc) | 309,044 B flash, 14,680 B RAM |
| eskf_test | 91/91 |
| magfit_test | 10/10 |
| link_test | 20/20 |
| cfgcodec_test | 16/16 |
| control_test | 35/35 |
| pca_test | 67/67 |
| viewer parse_test | 41/41 |

If a count drops, something regressed; if it grows, you added checks.

---

## 9. Open items / status board

1. **One-point fin offset calibration — FIXED today, needs reflash + bench
   confirm.** Symptom: `$lcal point 0` at 1200 µs then `$lcal save` →
   "table invalid". Cause: regression from the 2026-09-12 review-fix pass —
   the offset table spans the full 900–2100 µs range, so a 1200 µs neutral
   put one end at exactly 90°, which the new boot-safety bound rejects.
   Fix: `synthOffset` pulls overlong ends in along the same line (neutral and
   gain unchanged). Regression checks added to pca_test. Bench confirm:
   `$lcal start <fin>` → jog flush → `$lcal point 0` → `$lcal save` (expect
   "offset-only, default gain") → power cycle → `$lcal flash`.
2. **MG90S at 100 Hz frame — unconfirmed.** Owner should run servos a few
   minutes and touch-check. Buzz or warmth → `$sframe 50` live, then drop
   `SERVO_FRAME_HZ` back to 50.
3. **Roll-hold responsiveness — awaiting feedback.** Owner reported the
   BENCH response as slow. Done so far: write cadence 20 ms → 2.5 ms, frame
   50 → 100 Hz, slew 400 → 600 °/s, dt bug fixed, gains 0.04/0.05/3 →
   0.08/0.05/8. Suggested live test: `$ctlmode rate` then
   `$ctl 0.2 0.05 8`. Rate mode damps (won't return to an orientation);
   angle mode holds. If higher gain feels right, bake it into the defaults.
4. **Wrong-sketch incident.** Confirm the board runs this firmware (first
   line `{"t":"hdr","fw":"1.0.0",...}`). Renaming
   `Documents\Arduino\RocketNavF7` / `RocketNavEKF` was offered, not done.
5. **Magnetometer redesign — round 1 shipped, bench data never came back.**
   Owned LIS3MDL driver (the Adafruit/BusIO stack never set the I2C
   auto-increment bit 0x80 for multi-byte reads — prime suspect for "readings
   don't change with rotation"), raw `mgr` field + "mag raw" chart,
   `$magdiag` register probe with a VERDICT line. NOT yet reported by the
   owner: `$magdiag` output, the slow level-360° rotation test (X/Y must
   trace ±20 µT sinusoids, |m| ≈ 52 µT), a fresh `$magcal` tumble, the
   8-heading accuracy check. Calibration algorithm, ESKF mag gating, and
   persistence were deliberately left alone pending that data. Full brief,
   acceptance criteria, and bench procedure: `HANDOFF_MAG.md`. Servo current
   and MG90S magnets on the same board are new interference suspects.
6. **Unverified hardware assumptions (verify before any flight):** PCA9685
   address 0x40, canard channel map `SERVO_CH {0,1,2,3}`, `SERVO_US_PER_DEG`
   signs, `CTL_MIX_SIGN`, antennas on both Ra-02s, flight gains from the
   6DOF, the MG90S servo rail's current headroom.
7. **Uncommitted changes** (section 0) — commit/push only when asked.
8. **LoRa link — bench-verified 2026-09-20, field acceptance still open.**
   Both directions work (section 0). Remaining from PLAN_LORA_CONTROL §1.7:
   30-min soak with loss < 2 %, walk test ≥ 300 m with an honest loss stat,
   and the viewer connected to COM6 in LoRa mode with USB unplugged (the
   Web Serial port picker cannot be automated - owner's check). Bench
   numbers (RSSI −21 dBm, SNR 5 dB at ~1 m) are saturation numbers; judge
   the link from the walk test. Bring-up findings fixed the same day:
   `$lora?` reply now fits the 48-byte msg frame; base station nulls `v`
   outside RUN and `ral`/`rvs` outside WAIT_FIX/RUN/ATT_ONLY like USB does.
   Note the rocket's `rx` counter counts both deliveries of each twice-sent
   uplink (2 commands → r4): by design, not loss. The base's uplink was
   rewritten the same day to true listen-after-talk with a 30 ms cue delay
   (section 5.6); the final 6-ping run delivered 6/6 with 0 % downlink loss
   and a blind `$lora 1` unmuted a silent rocket. Bench script pattern for
   re-testing: open COM5 and COM6 together, `$lora 1` on COM5, count `st`
   records on COM6, send `$ping` ×6 on COM6, `$lora 0` on COM5.

---

## 10. Hard-won lessons (condensed)

- Validate frames through raw numbers, never through an unverified renderer:
  a viewer camera sign bug once got correct IMU mounts "fixed" into wrong
  ones. A mirrored symptom implicates the display layer, never a det=+1
  rotation chain.
- "Attitude drifts on the bench" usually means the bias random walk (Q) is
  too stiff, not a missing measurement.
- "Fuse X with Y" requests on this EKF usually meant a state-machine gap
  (fusion missing in some filter state), not missing math.
- |a| ≈ g is not stillness — gate on gyro AND accel statistics.
- A task's cost gate must be checked against the ACHIEVABLE slack
  (~1400 µs after a 440 Hz IMU poll), or the task starves forever.
- ICM-45686: never burst-read across FIFO_DATA (0x12–0x14); DRDY must be
  enabled as an INT1 source even though INT1 is not wired.
- LIS3MDL: multi-byte I2C reads need sub-address bit 7 set.
- PCA9685: integer prescale quantizes the frame — convert µs through the
  ACHIEVED frame (25e6 / (4096·(PRE+1))), not the requested one.
- Host-test harness: compare raw counts, not µs round-trips; clear mock
  transaction logs at the TOP of a loop iteration.
- Synthetic test noise must sit above the filter's R floors or NIS bands lie.
- Never subsample triangles to decimate a mesh (it makes a wireframe) —
  the viewer uses vertex clustering.
- Mag calibration: min/max midpoints are poisoned by one glitch and by
  un-tumbled axes (a flat spin leaks the 48 µT vertical field into z); the
  sphere fit holds axes whose swing stays under 30 µT.

---

## 11. Where the deeper history lives

- **Project memory** (chronological design log, every round's decisions):
  `C:\Users\lgrue_borstjl\.claude\projects\C--Users-lgrue-borstjl\memory\project_rocketnav_repo.md`
- `HANDOFF_MAG.md` — magnetometer brief, symptom log, bench procedure
- `DESIGN.md` — estimator math and measurement models
- `SCHEMA.md` — telemetry and command contract
- `PLAN_LORA_CONTROL.md`, `PLAN_LINKAGE_CAL.md` — design intent for the
  radio, control, and linkage subsystems
- Related but separate projects (do not confuse): `Documents\Arduino\
  RocketNavF7`, `Documents\Arduino\RocketNavEKF`, `Documents\Arduino\AHRS_F7`,
  KittyV1 firmware/ground station, `C:\Users\lgrue_borstjl\canardRocket6DOF`
  (the Simulink 6DOF used for control gains).
