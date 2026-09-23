# Plan: LoRa range — what the code can buy, what it cannot, and how far it reaches

Written 2026-09-21 after the first bench verification of the link (rocket
Ra-02 ⇄ ESP32-C3 base station, both SX1278). Numbers below are datasheet
typicals plus one bench measurement; treat every range figure as an
order-of-magnitude estimate until the walk test in section 6 replaces the
assumptions with measured path loss.

---

## 0. Where the link stands today

| Item | Value |
|---|---|
| Profile (both ends, `linkcodec.h`) | 433.5 MHz, **SF7, BW 500 kHz**, CR 4/5, explicit header, CRC on, preamble 8, sync 0x4B |
| TX power | 17 dBm PA_BOOST (50 mW), no PA_DAC high-power mode |
| Downlink | 53-byte `T` frame every 250 ms (4 Hz), ~26 ms airtime, ~10 % duty; `M` message frames alternate slots |
| Uplink | `C` command frames, listen-after-talk, twice per command |
| Receiver floor at this profile | **−116 dBm** (SX1278 datasheet, SF7/500 kHz); demodulator SNR limit −7.5 dB |
| Bench measurement | −21 dBm at ~1 m, 0 % loss, 0 CRC errors. Free-space at 1 m would be about −8 dBm, so the two spring antennas plus mismatch cost **~13 dB** together |

Working assumptions for every estimate below: 17 dBm out, **10 dB total
antenna/mismatch loss** (both ends, rounded down from the bench 13 dB), and
a **15 dB fade margin in flight** (spinning airframe, polarization mismatch,
pattern nulls). Free-space path loss at 433.5 MHz is
`FSPL = 85.2 + 20·log10(d_km)` dB.

---

## 1. Link budget: the physics that sets the ceiling

Range in free space follows from one line:

```
allowed path loss = P_tx − L_ant − S_rx − M_fade
d_km = 10^((allowed − 85.2) / 20)
```

Every 6 dB doubles the distance. The levers, ranked by dB per unit of
effort:

| Lever | dB available | Where it lives |
|---|---|---|
| Antennas (tuned ¼-wave / dipole on the rocket, elevated ground-plane or Yagi at the base) | **+5 to +15** | hardware |
| Spreading factor / bandwidth (SF7/500 → SF12/125) | **+20** | code, both ends, costs airtime |
| PA high-power mode (17 → 20 dBm) | +3 | code, duty and heat constraints |
| Coding rate 4/5 → 4/8 | +1 to +2 effective (burst errors), −60 % airtime | code |
| Preamble 8 → 12 | +0.3, better sync at the floor | code |
| Base antenna height (ground link only) | +6 per doubling of height | hardware |

The antenna line is first on purpose: it is the biggest lever, it costs no
airtime, and the bench number says the current antennas throw away 13 dB.

---

## 2. Modem profiles: sensitivity vs airtime

Airtime for explicit header, CR 4/5, preamble 8 (12.25 symbols),
`T_sym = 2^SF / BW`. "Beacon" is the proposed 22-byte recovery frame of
section 4. Sensitivity from the SX1276/7/8 datasheet typicals.

| Profile | T_sym | 53 B frame | 22 B beacon | Sensitivity | Gain vs now | Max cadence at 10 % duty (53 B) |
|---|---|---|---|---|---|---|
| **SF7 / 500 k (now)** | 0.26 ms | 26 ms | 12 ms | −116 dBm | 0 | 4 Hz |
| SF8 / 500 k | 0.51 ms | 46 ms | 22 ms | −119 dBm | +3 | 2 Hz |
| SF9 / 500 k | 1.0 ms | 82 ms | 43 ms | −122 dBm | +6 | 1 Hz |
| SF9 / 125 k | 4.1 ms | 329 ms | 206 ms | −129 dBm | +13 | 0.3 Hz |
| SF10 / 125 k | 8.2 ms | 617 ms | 370 ms | −132 dBm | +16 | 0.16 Hz |
| SF11 / 125 k ‡ | 16.4 ms | 1.31 s | 0.74 s | −133 dBm | +17 | one per 13 s |
| SF12 / 125 k ‡ | 32.8 ms | 2.47 s | 1.32 s | −136 dBm | +20 | one per 25 s |

‡ needs `LowDataRateOptimize` (RegModemConfig3 bit 3) whenever T_sym > 16 ms.
The driver currently writes 0x04 (AGC only) — a hard requirement for any
SF11/SF12 profile, section 5.

Bandwidth below 125 kHz is off the table: the Ra-02 has a plain crystal
(±10–20 ppm ≈ ±9 kHz at 433 MHz) and the SX1278 tolerates ±25 % of BW of
frequency error. 125 kHz leaves margin; 62.5 kHz is marginal; 31.25 kHz
fails without a TCXO.

---

## 3. Range estimates

### 3.1 In the air (line of sight, rocket above ground clutter)

`allowed = 17 − 10 − 15 − S`; `d = 10^((allowed − 85.2)/20)` km.

| Profile | Allowed path loss | LOS range with 15 dB fade margin | LOS range, zero margin |
|---|---|---|---|
| SF7 / 500 k (now) | 108 dB | **~14 km** | ~77 km |
| SF8 / 500 k | 111 dB | ~20 km | |
| SF9 / 500 k | 114 dB | ~28 km | |
| SF9 / 125 k | 121 dB | ~60 km | |
| SF12 / 125 k | 128 dB | ~140 km | |

Read this the honest way: the **current profile already outranges any
flight this vehicle will make** (a few km apogee, landing within a few km).
In the air the link does not fail on distance. It fails on **fades**: the
spinning airframe swings the antenna pattern and polarization through
10–20 dB nulls lasting tens of milliseconds. Against fades, short frames at
a high cadence win — a 26 ms SF7 frame slips between nulls and the next one
is 250 ms away, while a 2.5 s SF12 frame is guaranteed to straddle several
and be lost. **Do not move the flight profile to a high SF for range.**

### 3.2 On the ground (the case that actually limits you)

After landing the rocket's antenna is ~0.3 m off the ground and the base
antenna ~1.5 m in someone's hand. Beyond a few metres the two-ray ground
model applies: `PL = 40·log10(d_m) − 20·log10(h1·h2)`, i.e. 12 dB per
doubling instead of 6, plus vegetation. With 10 dB antenna loss and a 6 dB
margin (`allowed = 1 − S`):

| Profile | Base antenna at 1.5 m | Base antenna raised to 5 m |
|---|---|---|
| SF7 / 500 k (now) | **~560 m** | ~1.0 km |
| SF9 / 125 k | ~1.2 km | ~2.2 km |
| SF12 / 125 k | ~1.8 km | ~3.3 km |

Two conclusions: the ground link is where a slow profile pays (×3 in
distance), and raising the base antenna is worth as much as five
spreading-factor steps. A landed rocket 2 km out in a field needs both.

---

## 4. Recommended design: two profiles, deterministic switching

Keep one fast profile for flight and add one slow profile for recovery.
Never adapt continuously in flight: an adaptive scheme that loses the
"switch now" message desynchronizes the two radios exactly when the link is
weakest, and there is no ack channel to notice.

### 4.1 Flight profile (the display link)

**SF8 / 500 kHz at 2 Hz**, or stay at SF7 / 500 kHz at 4 Hz.
- SF8 buys +3 dB for 18 % duty at 4 Hz or 9 % at 2 Hz.
- Keep the 53-byte `T` frame; nothing else changes.
- Recommendation: stay at **SF7 / 4 Hz** until the walk test says
  otherwise. The field screen's 4 Hz update is worth more than 3 dB you
  are not short of.

### 4.2 Recovery profile (the find-the-rocket link)

**SF12 / 125 kHz, CR 4/8, preamble 12, 22-byte `B` beacon every 10 s.**
- Beacon payload: ms u32, flags u8 (fix, phase, battery-ok), lat i32,
  lon i32, alt MSL i16, AGL i16, roll/pitch/yaw i8 ×3 (2° steps), health
  u8 → 20 B + 5 B framing = 25 B, ~1.4 s airtime, 14 % duty at one per
  10 s. (Tighten to 22 B by dropping AGL if wanted.)
- Position is the point. Attitude at 2° is a courtesy.
- +20 dB sensitivity, and the rocket listens on SF12 between beacons so
  `$ping`/`$lora flight` still reach it.

### 4.3 Switching rules (no negotiation, both ends can derive them alone)

Rocket → recovery when ANY of:
1. control phase SAFE for 30 s with the IMU reporting stillness (landed);
2. no base **keepalive** heard for 20 s (see below), in any phase but
   ACTIVE (never switch during boost/coast — the fade argument above);
3. `$lora rec` over USB or RF. `$lora flight` returns.

Base station:
1. Listens on the flight profile. No frame for 10 s → switch to recovery
   and listen there.
2. Beacon heard → stay in recovery, emit NDJSON `st` from it (the viewer
   already tolerates subset records as of 2026-09-21), and note
   `base: profile recovery` on the console.
3. Nothing heard on recovery for 30 s → alternate 10 s on each profile
   until something arrives (a scan, so a wrong guess can never strand it).
4. Uplinks always go out on whatever profile the base is currently
   listening on; the rocket listens on its current profile, so both agree
   whenever the base has heard the rocket at all.

Base keepalive: a 6-byte `K` frame every 5 s, cued like any uplink. The
rocket only uses it to reset a "base heard" timer. Cost: ~5 ms airtime every
5 s. This is the one addition that lets the rocket *know* it has been lost
instead of guessing.

### 4.4 Optional: +20 dBm

`RegPaDac = 0x87` and OCP trim to 140 mA give 20 dBm on PA_BOOST. The
datasheet allows it only at ≤ 1 % duty with a good antenna match. That rules
it out for the 4 Hz flight link (10 % duty) and makes it marginal for the
recovery beacon (14 %). Take the 3 dB only if the beacon cadence is relaxed
to one per 30 s, and only with a measured antenna. Probably not worth it.

---

## 5. Implementation steps (rocket, base, viewer, tests)

| # | Change | Files | Effort | Risk |
|---|---|---|---|---|
| 1 | `LowDataRateOptimize` auto-set when `T_sym > 16 ms`; OCP left default | `sx1278.cpp` (both copies) | 10 min | none — required for SF11/12 |
| 2 | `Sx1278::setProfile(sf, bw, cr, preamble)` = the modem half of `configure()` without the RESET/version probe, ends in RX | `sx1278.*` (both copies) | 30 min | low |
| 3 | Profile table in `linkcodec.h`: FLIGHT {7, 500 k, 4/5, 8, 250 ms}, RECOVERY {12, 125 k, 4/8, 12, 10 s}; `B` beacon pack/parse + `K` keepalive; `link_test` cases (round trip, airtime helper) | `linkcodec.*`, `test/link_test.cpp` | 1–2 h | low, host-tested |
| 4 | Rocket MAC: profile state, beacon scheduler, landed/keepalive timers, `$lora rec|flight`, `$lora?` shows profile | `link.*`, `RocketNav.ino`, `control.h` (expose stillness) | 2–3 h | medium: the ACTIVE-phase lockout is the safety rule |
| 5 | Base: listen-mode machine, keepalive, beacon → NDJSON, `profile` in `hdr` and a msg on every switch | `basestation.ino` | 2 h | medium: scan timing |
| 6 | Viewer: profile badge on the LoRa bar, stale threshold from `hdr.ohz` (0.1 Hz in recovery), beacon-only records keep the map live | `viewer/index.html` | 1 h | low |
| 7 | Rocket-side RSSI of the base's uplink into the `T` frame (1 byte, unused bit budget exists) so both directions are visible in flight | `linkcodec.*`, `basestation.ino`, `SCHEMA.md` | 1 h | low, wire change → reflash both |

Total: about one working day, all host-testable except the timers.

---

## 6. Test plan and acceptance

1. **Bench, both profiles:** `$lora rec` → base swaps within 10 s, beacons
   decode, `$ping` answered on SF12; `$lora flight` → back within 10 s.
   Pull the rocket's power for 40 s → base scans and re-acquires.
2. **Walk test (do this before writing any of section 5):** rocket on a
   tripod at 1.5 m, base handheld, walk out with the viewer's **margin**
   badge visible. Log distance vs RSSI every 50 m to 500 m, then every
   200 m. Fit `RSSI = A − n·10·log10(d)`: the slope `n` (2 = free space,
   3–4 = ground clutter) and `A` (your real antenna loss) replace the two
   assumptions in this document. Expect the 15 dB antenna line to move.
3. **Ground test:** rocket lying in grass, base at 1.5 m then at 5 m (a
   pole). Find the distance where SF7 drops out and where SF12 beacons stop.
   Those two numbers are the recovery range.
4. **Acceptance:** flight profile 0 % loss over a 30 min bench soak and
   < 2 % on the walk test to the fitted distance; recovery beacons decoded
   at ≥ 2× the SF7 ground dropout distance; switch round trips ≤ 10 s.

---

## 7. Benefits and drawbacks, per lever

| Lever | Benefit | Drawback |
|---|---|---|
| Better antennas | biggest gain, no airtime, no protocol change | mechanical work on the airframe; a ¼-wave at 433 MHz is 16.5 cm |
| Recovery profile (SF12) | +20 dB after landing, ~×3 ground range, GPS fix keeps coming | 1.4 s beacons: 0.1 Hz, no attitude to speak of; extra MAC states on both ends |
| Keepalive | rocket knows it is lost; deterministic switching | 5 ms/5 s uplink airtime; one more frame type |
| SF8 flight profile | +3 dB | halves cadence or doubles duty; fades matter more than 3 dB |
| +20 dBm | +3 dB | ≤ 1 % duty spec, heat, needs a matched antenna, more battery |
| CR 4/8 | survives burst errors | +60 % airtime for ~1–2 dB |
| Continuous ADR in flight | none you need | desync when the link is weakest; rejected |

Regulatory note: 433 MHz is the US 70 cm amateur band — this needs a licence
and a callsign in the beacon (a field the plan reserved and the `B` frame
should carry). In ITU Region 1 the 433 MHz SRD band caps at 10 mW e.r.p. and
10 % duty, which the current 50 mW / 10 % flight link already exceeds.

---

## 7b. Implementation notes (2026-09-21, sections 4–5 built the same day)

- Recovery profile chosen as **SF11 / 125 kHz, CR 4/5, preamble 12** rather
  than SF12 / CR 4/8: half the airtime (a 27-byte beacon is ~0.9 s instead
  of ~2.4 s) for 3 dB less, so the 10 s cadence stays under 10 % duty. One
  line in `linkcodec.h` (`kProfileRecovery`) moves it.
- Beacon `B` is 27 bytes (22 payload): ms, status, lat, lon, alt MSL, AGL,
  euler at 2°, sats, health. Keepalive `K` is 5 bytes.
- The `T` frame grew to 54 bytes with the rocket-side uplink RSSI (step 7);
  the parser still accepts the 53-byte legacy frame so the base can be
  flashed before the rocket. `cmode` widened to 3 bits (BENCH = 4 was
  wrapping to 0 on the air).
- Rocket rules as in 4.3, with "still" = |gyro| < 3 dps and | |a| − g | <
  1 m/s² for 30 s in SAFE; the 20 s keepalive rule only arms after a
  keepalive has been heard once, so an old base never triggers it.
- Base rules as in 4.3, plus: it stays on FLIGHT until it has heard the
  rocket at least once (bench: a muted rocket's Unmute must get through),
  and a command's repeat goes out on the other profile when the rocket has
  not been heard on the current one. `$base flight|rec` pins, `$base auto`
  frees. `hdr` carries `prof` and `ohz`.
- Viewer: profile badge, Recovery/Flight buttons, up-rssi badge, BEACON
  downlink state, stale threshold from `hdr.ohz` (30 s in recovery).
- Not built: +20 dBm (rejected), CR 4/8, callsign field (still reserved).
- Tests still owed: section 6 in full. Nothing here has been on the air.

## 8. Bottom line

- The code cannot fix the biggest loss: 13 dB in antennas. Fix that first.
- In the air the current profile reaches ~14 km with a generous fade margin;
  your flights will not get close. Keep SF7 / 4 Hz for flight.
- On the ground the link dies at roughly half a kilometre today. A SF12
  recovery beacon takes that to ~2 km, a raised base antenna to ~3 km, and
  that is what finds the rocket.
- One working day of firmware for the two-profile scheme, after the walk
  test has replaced the assumptions with measurements.
