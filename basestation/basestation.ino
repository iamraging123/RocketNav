// RocketNav base station - ESP32-C3 + Ai-Thinker Ra-02 (SX1278).
//
// Transparent bridge, no UI of its own:
//   LoRa downlink ('T' state / 'B' beacon / 'M' message frames) -> NDJSON on
//   USB serial, the same records the viewer already speaks - connect the
//   viewer to THIS port and the field screen runs with zero protocol changes.
//   '$command' lines typed into the viewer console -> 'C' uplink frames
//   (sent twice; the rocket deduplicates by sequence number). A 'K'
//   keepalive goes up every 5 s so the rocket can tell it is being heard.
//
// Two air profiles (PLAN_LORA_RANGE.md): FLIGHT (SF7/500k, 4 Hz state
// frames) and RECOVERY (SF11/125k, one position beacon every 10 s). The
// base follows the rocket without negotiation: once it has heard the rocket
// at all, 10 s of silence on FLIGHT -> listen on RECOVERY; 30 s of silence
// there -> alternate 10 s on each until a frame arrives. Until the first
// frame ever, it stays on FLIGHT so a muted bench rocket's Unmute gets
// through. `$base flight|rec` pins the listen profile, `$base auto` frees it.
//
// Wiring (owner netlist): MOSI GPIO5, MISO GPIO6, SCK GPIO7, NSS GPIO0,
// RESET GPIO4, DIO0 GPIO1. DIO1-5 not connected. Antenna REQUIRED before TX.
//
// sx1278.* and linkcodec.* are copies of the rocket's files - the codec is
// the air contract; keep the copies in sync with firmware/RocketNav/.
#include <SPI.h>

#include "linkcodec.h"
#include "sx1278.h"

#define PIN_MOSI 5
#define PIN_MISO 6
#define PIN_SCK 7
#define PIN_NSS 0
#define PIN_RESET 4
#define PIN_DIO0 1
#define TX_DBM 17

static Sx1278 radio;

// Viewer-facing record counter: advanced by the AIR gap between frame
// sequence numbers, so dropped frames show up in the viewer's own
// loss/seq-gap accounting.
static uint32_t seq32_ = 0;
static bool have_seq_ = false;
static uint8_t last_seq_ = 0;
static uint32_t rx_frames_ = 0, lost_frames_ = 0, crc_errs_ = 0;
static uint32_t last_rx_ms_ = 0;
static uint32_t next_wait_note_ms_ = 3000;
// The hdr record repeats every 10 s: the ESP32-C3 reboots when its port is
// opened, so the boot-time hdr can be torn or missed by a viewer that
// attaches a moment later, and hdr.mode:"lora" / hdr.prof are what flip the
// viewer into LoRa presentation.
static uint32_t next_hdr_ms_ = 10000;

// Listen profile machine (PLAN_LORA_RANGE.md section 4.3).
#define FLIGHT_LOST_MS 10000     // silence on FLIGHT -> listen on RECOVERY
#define RECOVERY_LOST_MS 30000   // silence on RECOVERY -> scan
#define SCAN_DWELL_MS 10000      // scan: dwell per profile
static uint8_t prof_ = lc::kProfFlight;
static bool prof_pinned_ = false;      // $base flight|rec
static bool scanning_ = false;
static uint32_t prof_since_ms_ = 0;

// Uplink, listen-after-talk: each command goes out twice (the rocket drops
// the duplicate by seq), and each transmission waits for the NEXT downlink
// frame to land - right after the rocket's TxDone it sits in RX for the
// remaining ~220 ms of its 250 ms slot, so an uplink cued that way can never
// collide with a downlink frame (an immediate uplink had a ~15 % chance of
// costing the base one 'T' frame per transmission). The cue is NOT the
// instant the frame lands: the rocket only discovers its own TxDone at its
// next 250 Hz radio poll (a few ms, more when telemetry/service slots are
// due) and only then re-arms RX - an uplink started inside that gap was
// lost every time on the bench (6 pings -> 1 delivery). UPLINK_CUE_MS after
// the frame the rocket is listening, and the ~10 ms uplink is long over
// before its next slot ~220 ms later. If no downlink arrives within
// UPLINK_BLIND_MS (rocket muted, link down) the frame goes blind so
// `$lora 1` still reaches a silent rocket.
//
// When the rocket has not been heard on the current listen profile, the
// command's REPEAT goes out on the other profile (modem switched for one
// transmission, then restored): a muted bench rocket still on FLIGHT gets
// its Unmute even while the base is scanning.
#define UPLINK_CUE_MS 30
#define UPLINK_BLIND_MS 400       // FLIGHT: a C frame is ~10 ms on the air
#define UPLINK_BLIND_REC_MS 2500  // RECOVERY: ~640 ms C frame + the rocket's
                                  // immediate ~700 ms reply must both clear
                                  // before the blind repeat, or the repeat
                                  // talks over the reply (bench, 2026-09-21)
static uint8_t up_seq_ = 1;
static uint8_t pend_[lc::kMaxFrame];
static int pend_len_ = 0;
static uint8_t pend_left_ = 0;         // transmissions still owed (2, 1, 0)
static uint32_t pend_send_at_ms_ = 0;  // earliest send time (cue or blind)
static bool pend_is_cmd_ = false;      // commands get the other-profile repeat
static bool alt_restore_ = false;      // modem is on the other profile for a TX

// Keepalive: one 5-byte 'K' every 5 s through the same cued path whenever no
// command is pending. The rocket uses it only to detect a lost base.
#define KEEPALIVE_MS 5000
static uint32_t next_ka_ms_ = 5000;
static uint8_t ka_seq_ = 1;

static char line_[96];
static uint8_t line_len_ = 0;

static uint32_t blindMs() {
  return prof_ == lc::kProfRecovery ? UPLINK_BLIND_REC_MS : UPLINK_BLIND_MS;
}

static const char *profName(uint8_t p) {
  return p == lc::kProfRecovery ? "recovery" : "flight";
}
static const lc::Profile &profOf(uint8_t p) {
  return p == lc::kProfRecovery ? lc::kProfileRecovery : lc::kProfileFlight;
}
static uint8_t otherProf(uint8_t p) {
  return p == lc::kProfRecovery ? lc::kProfFlight : lc::kProfRecovery;
}
static void modemFor(uint8_t p) {
  const lc::Profile &pr = profOf(p);
  radio.setModem(pr.sf, pr.bw_hz, pr.cr_denom, pr.preamble);
}

static void emitMsg(const char *txt) {
  Serial.printf("{\"t\":\"msg\",\"us\":%lu,\"txt\":\"%s\"}\n",
                (unsigned long)millis() * 1000ul, txt);
}

static void emitHdr() {
  // ohz = the downlink cadence of the profile being listened to; the viewer
  // scales its stale threshold from it.
  Serial.printf(
      "{\"t\":\"hdr\",\"us\":0,\"fw\":\"base-0.2.0\",\"schema\":\"lora-2\","
      "\"mode\":\"lora\",\"prof\":\"%s\",\"ohz\":%s}\n",
      profName(prof_), prof_ == lc::kProfRecovery ? "0.1" : "4");
}

static bool heardSinceSwitch() {
  return rx_frames_ > 0 && (int32_t)(last_rx_ms_ - prof_since_ms_) >= 0;
}

static void listenOn(uint8_t p, const char *why) {
  modemFor(p);
  prof_ = p;
  prof_since_ms_ = millis();
  const lc::Profile &pr = profOf(p);
  char b[112];
  snprintf(b, sizeof(b), "base: listening %s (SF%u/%luk) - %s", profName(p),
           (unsigned)pr.sf, (unsigned long)(pr.bw_hz / 1000ul), why);
  emitMsg(b);
  emitHdr();
  next_hdr_ms_ = millis() + 10000;
}

static float lossPct() {
  uint32_t total = rx_frames_ + lost_frames_;
  return total ? 100.0f * (float)lost_frames_ / (float)total : 0.0f;
}

static void bumpSeq(uint8_t seq) {
  if (have_seq_) {
    uint8_t gap = (uint8_t)(seq - last_seq_);
    if (gap == 0) gap = 1;      // duplicate: count it as one
    seq32_ += gap;
    if (gap > 1) lost_frames_ += (uint32_t)(gap - 1);
  } else {
    have_seq_ = true;
    seq32_ = 1;
  }
  last_seq_ = seq;
  rx_frames_++;
  last_rx_ms_ = millis();
}

static void geoStr(char *geo, size_t cap, double lat, double lon, float alt) {
  if (lat != 0 || lon != 0) {
    snprintf(geo, cap, "\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f", lat, lon,
             (double)alt);
  } else {
    snprintf(geo, cap, "\"lat\":null,\"lon\":null,\"alt\":null");
  }
}

static void emitState(const lc::StateFields &f, uint8_t seq) {
  bumpSeq(seq);
  char b[700];
  char geo[96];
  geoStr(geo, sizeof(geo), f.lat_deg, f.lon_deg, f.alt_msl_m);
  // Validity mirrors the rocket's USB record (telemetry.cpp): v is null
  // unless the filter is in RUN (3); the vertical channel ral/rvs is null
  // outside WAIT_FIX (2) / RUN (3) / ATT_ONLY (4). The air frame packs the
  // raw filter numbers regardless (horizontal velocity is meaningless in
  // attitude-only mode), so the gate has to live here.
  char vs[56], vch[48], ur[8];
  if (f.fst == 3) {
    snprintf(vs, sizeof(vs), "\"v\":[%.2f,%.2f,%.2f]", (double)f.vel_mps[0],
             (double)f.vel_mps[1], (double)f.vel_mps[2]);
  } else {
    snprintf(vs, sizeof(vs), "\"v\":null");
  }
  if (f.fst >= 2 && f.fst <= 4) {
    snprintf(vch, sizeof(vch), "\"ral\":%.1f,\"rvs\":%.2f", (double)f.ral_m,
             (double)(-f.vel_mps[2]));
  } else {
    snprintf(vch, sizeof(vch), "\"ral\":null,\"rvs\":null");
  }
  if (f.up_rssi_dbm != 0) snprintf(ur, sizeof(ur), "%d", (int)f.up_rssi_dbm);
  else snprintf(ur, sizeof(ur), "null");
  // Health: the frame carries fresh bits only; present(1)+fresh(2) when
  // fresh, bare present otherwise - the viewer's stale logic reads bit 1.
  int hi = (f.health & 1) ? 3 : 1, hm = (f.health & 2) ? 3 : 1;
  int hb = (f.health & 4) ? 3 : 1, hg = (f.health & 8) ? 3 : 1;
  snprintf(b, sizeof(b),
           "{\"t\":\"st\",\"us\":%lu,\"seq\":%lu,\"fst\":%u,"
           "\"eul\":[%.2f,%.2f,%.2f],"
           "\"gyr\":[%.5f,%.5f,%.5f],"
           "\"acc\":[%.3f,%.3f,%.3f],"
           "%s,%s,%s,"
           "\"gfix\":%d,\"gsv\":%u,\"ghac\":%.1f,"
           "\"himu\":%d,\"hmag\":%d,\"hbar\":%d,\"hgps\":%d,"
           "\"cmode\":%u,\"cdef\":[%.2f,%.2f,%.2f,%.2f],"
           "\"rssi\":%d,\"snr\":%.1f,\"loss\":%.1f,\"urssi\":%s}",
           (unsigned long)f.ms * 1000ul, (unsigned long)seq32_, f.fst,
           (double)f.eul_deg[0], (double)f.eul_deg[1], (double)f.eul_deg[2],
           (double)(f.gyr_dps[0] * 0.017453293f),
           (double)(f.gyr_dps[1] * 0.017453293f),
           (double)(f.gyr_dps[2] * 0.017453293f),
           (double)(f.acc_g[0] * 9.80665f), (double)(f.acc_g[1] * 9.80665f),
           (double)(f.acc_g[2] * 9.80665f), vs, vch, geo, f.fix ? 3 : 0,
           f.sats, (double)f.hacc_m, hi, hm, hb, hg, f.cmode,
           (double)f.cdef_deg[0], (double)f.cdef_deg[1],
           (double)f.cdef_deg[2], (double)f.cdef_deg[3],
           (int)radio.pktRssiDbm(), (double)radio.pktSnrDb(),
           (double)lossPct(), ur);
  Serial.println(b);
}

// Recovery beacon -> the same st record, smaller: position, coarse attitude,
// fix, health, control mode. "beacon":1 marks it for the viewer.
static void emitBeacon(const lc::BeaconFields &f, uint8_t seq) {
  bumpSeq(seq);
  char b[400];
  char geo[96], vch[32];
  geoStr(geo, sizeof(geo), f.lat_deg, f.lon_deg, f.alt_msl_m);
  if (f.fst >= 2 && f.fst <= 4) snprintf(vch, sizeof(vch), "\"ral\":%.1f", (double)f.ral_m);
  else snprintf(vch, sizeof(vch), "\"ral\":null");
  int hi = (f.health & 1) ? 3 : 1, hm = (f.health & 2) ? 3 : 1;
  int hb = (f.health & 4) ? 3 : 1, hg = (f.health & 8) ? 3 : 1;
  snprintf(b, sizeof(b),
           "{\"t\":\"st\",\"us\":%lu,\"seq\":%lu,\"fst\":%u,"
           "\"eul\":[%.0f,%.0f,%.0f],%s,%s,"
           "\"gfix\":%d,\"gsv\":%u,"
           "\"himu\":%d,\"hmag\":%d,\"hbar\":%d,\"hgps\":%d,"
           "\"cmode\":%u,\"rssi\":%d,\"snr\":%.1f,\"loss\":%.1f,\"beacon\":1}",
           (unsigned long)f.ms * 1000ul, (unsigned long)seq32_, f.fst,
           (double)f.eul_deg[0], (double)f.eul_deg[1], (double)f.eul_deg[2],
           vch, geo, f.fix ? 3 : 0, f.sats, hi, hm, hb, hg, f.cmode,
           (int)radio.pktRssiDbm(), (double)radio.pktSnrDb(),
           (double)lossPct());
  Serial.println(b);
}

static void emitText(const char *txt, uint8_t seq) {
  bumpSeq(seq);
  // Rocket messages are plain ASCII without quotes; scrub defensively so a
  // stray character can never break the JSON stream.
  char clean[lc::kMaxText + 1];
  int j = 0;
  for (int i = 0; txt[i] != 0 && j < lc::kMaxText; ++i) {
    char c = txt[i];
    if (c == '"' || c == '\\') c = '\'';
    if ((uint8_t)c < 0x20) c = ' ';
    clean[j++] = c;
  }
  clean[j] = 0;
  emitMsg(clean);
}

static void sendCmd(const char *cmd) {
  if (!radio.present()) {
    emitMsg("base: no radio - command not sent");
    return;
  }
  pend_len_ = lc::packText(lc::kTypeCmd, cmd, up_seq_++, pend_);
  pend_left_ = 2;
  pend_is_cmd_ = true;
  pend_send_at_ms_ = millis() + blindMs();
}

static void queueKeepalive() {
  if (pend_left_ > 0 || !radio.present()) return;  // never displace a command
  pend_len_ = lc::packKeepalive(ka_seq_++, pend_);
  pend_left_ = 1;
  pend_is_cmd_ = false;
  pend_send_at_ms_ = millis() + blindMs();
}

// Called every loop pass; downlink_heard = a frame was just read out of the
// radio, i.e. the rocket has just finished transmitting.
static void serviceUplink(bool downlink_heard) {
  uint32_t ms = millis();
  if (alt_restore_) {  // the other-profile repeat has gone out: come back
    if (radio.txBusy()) return;
    modemFor(prof_);
    alt_restore_ = false;
  }
  if (pend_left_ == 0) return;
  if (downlink_heard) {  // pull the send time in to the cue (never push out)
    uint32_t cue = ms + UPLINK_CUE_MS;
    if ((int32_t)(cue - pend_send_at_ms_) < 0) pend_send_at_ms_ = cue;
  }
  if ((int32_t)(ms - pend_send_at_ms_) < 0 || radio.txBusy()) return;
  // The repeat of a command goes out on the other profile when the rocket
  // has not been heard on this one (rocket still on FLIGHT while we scan).
  bool alt = pend_is_cmd_ && pend_left_ == 1 && !heardSinceSwitch();
  if (alt) modemFor(otherProf(prof_));
  if (radio.txStart(pend_, (uint8_t)pend_len_)) {
    pend_left_--;
    pend_send_at_ms_ = ms + blindMs();  // the repeat waits the same way
    if (alt) alt_restore_ = true;
  } else if (alt) {
    modemFor(prof_);
  }
}

static void handleLine(char *s) {
  if (s[0] != '$') return;
  if (strcmp(s, "$base regs") == 0) {
    // Raw radio state, no transmission: dio0 is the GPIO1 pin level, irq
    // the SX1278 flag register (0x40 = RxDone pending). irq showing RxDone
    // while dio0 stays 0 = the DIO0 wire, not the radio.
    char b[120];
    snprintf(b, sizeof(b),
             "base: dio0 %d opmode 0x%02X irq 0x%02X modem 0x%02X "
             "rssi %d dBm frf %02X%02X%02X",
             (int)radio.dio0High(), radio.reg(0x01), radio.reg(0x12),
             radio.reg(0x18), (int)radio.reg(0x1B) - 164, radio.reg(0x06),
             radio.reg(0x07), radio.reg(0x08));
    emitMsg(b);
    return;
  }
  if (strcmp(s, "$base flight") == 0 || strcmp(s, "$base rec") == 0) {
    prof_pinned_ = true;
    scanning_ = false;
    listenOn(s[6] == 'r' ? lc::kProfRecovery : lc::kProfFlight, "pinned");
    return;
  }
  if (strcmp(s, "$base auto") == 0) {
    prof_pinned_ = false;
    emitMsg("base: listen profile follows the rocket again");
    return;
  }
  if (strcmp(s, "$base?") == 0) {
    char b[160];
    snprintf(b, sizeof(b),
             "base: radio %s %s%s%s rx %lu lost %lu crcerr %lu rssi %d "
             "snr x10 %d dio0 %lu timer %lu",
             radio.present() ? "ok" : "ABSENT", profName(prof_),
             prof_pinned_ ? " (pinned)" : "", scanning_ ? " (scanning)" : "",
             (unsigned long)rx_frames_, (unsigned long)lost_frames_,
             (unsigned long)crc_errs_, (int)radio.pktRssiDbm(),
             (int)(radio.pktSnrDb() * 10.0f),
             (unsigned long)radio.rxViaDio0(),
             (unsigned long)radio.rxViaTimer());
    emitMsg(b);
    return;
  }
  sendCmd(s);
  char b[96];
  snprintf(b, sizeof(b), "base: uplinked %s", s);
  emitMsg(b);
}

// Follow the rocket's profile without negotiation (section 4.3 of the plan).
static void serviceProfile(uint32_t ms) {
  if (prof_pinned_ || rx_frames_ == 0) return;  // never heard it: stay FLIGHT
  uint32_t ref = ((int32_t)(last_rx_ms_ - prof_since_ms_) >= 0) ? last_rx_ms_
                                                                  : prof_since_ms_;
  uint32_t quiet = ms - ref;
  if (!scanning_) {
    if (prof_ == lc::kProfFlight && quiet > FLIGHT_LOST_MS) {
      listenOn(lc::kProfRecovery, "flight quiet 10 s");
    } else if (prof_ == lc::kProfRecovery && quiet > RECOVERY_LOST_MS) {
      scanning_ = true;
      listenOn(lc::kProfFlight, "recovery quiet 30 s, scanning");
    }
    return;
  }
  if (heardSinceSwitch()) {
    scanning_ = false;
    char b[64];
    snprintf(b, sizeof(b), "base: scan found the rocket on %s", profName(prof_));
    emitMsg(b);
    return;
  }
  if (ms - prof_since_ms_ > SCAN_DWELL_MS) listenOn(otherProf(prof_), "scan");
}

void setup() {
  Serial.begin(921600);
  delay(300);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  bool ok = radio.begin(&SPI, PIN_NSS, PIN_RESET, PIN_DIO0);
  if (ok) {
    radio.configure(LC_LORA_FREQ_HZ, LC_LORA_SF, LC_LORA_BW_HZ,
                    LC_LORA_CR_DENOM, LC_LORA_SYNC, LC_LORA_PREAMBLE, TX_DBM);
  }
  prof_ = lc::kProfFlight;
  prof_since_ms_ = millis();
  emitHdr();
  emitMsg(ok ? "base: SX1278 OK - listening flight 433.5 MHz SF7/500k"
             : "base: SX1278 NOT FOUND - check SPI wiring");
}

void loop() {
  uint8_t ev = radio.poll();
  if (ev & Sx1278::EV_CRCERR) crc_errs_++;
  bool heard = false;
  if (ev & Sx1278::EV_RXDONE) {
    uint8_t buf[lc::kMaxFrame];
    int n = radio.rxRead(buf, sizeof(buf));
    heard = n > 0;
    if (n > 0) {
      lc::StateFields f;
      lc::BeaconFields bf;
      uint8_t seq = 0, type = 0;
      char txt[lc::kMaxText + 1];
      if (lc::parseState(buf, n, &f, &seq)) {
        emitState(f, seq);
      } else if (lc::parseBeacon(buf, n, &bf, &seq)) {
        emitBeacon(bf, seq);
      } else if (lc::parseText(buf, n, txt, sizeof(txt), &seq, &type) &&
                 type == lc::kTypeMsg) {
        emitText(txt, seq);
      } else {
        crc_errs_++;  // valid radio CRC but not a frame we know
      }
    }
  }

  serviceUplink(heard);  // listen-after-talk: cue off the frame just heard

  // console input -> commands
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line_[line_len_] = 0;
      if (line_len_ > 0) handleLine(line_);
      line_len_ = 0;
    } else if (line_len_ < sizeof(line_) - 1) {
      line_[line_len_++] = c;
    } else {
      line_len_ = 0;
    }
  }

  uint32_t ms = millis();
  if ((int32_t)(ms - next_ka_ms_) >= 0) {
    next_ka_ms_ = ms + KEEPALIVE_MS;
    queueKeepalive();
  }

  // quiet-link heartbeat so a dead rocket link is obvious in the console.
  // The threshold follows the listen profile: 5 s on FLIGHT (4 Hz), 25 s on
  // RECOVERY (a beacon every 10 s is not "quiet").
  const uint32_t quiet_ms = prof_ == lc::kProfRecovery ? 25000ul : 5000ul;
  if (rx_frames_ == 0 || ms - last_rx_ms_ > quiet_ms) {
    if (ms >= next_wait_note_ms_) {
      next_wait_note_ms_ = ms + quiet_ms;
      if (rx_frames_ == 0) {
        emitMsg("base: waiting for first downlink frame");
      } else {
        char q[48];
        snprintf(q, sizeof(q), "base: downlink quiet > %lu s",
                 (unsigned long)(quiet_ms / 1000ul));
        emitMsg(q);
      }
    }
  }
  serviceProfile(ms);
  if ((int32_t)(ms - next_hdr_ms_) >= 0) {
    next_hdr_ms_ = ms + 10000;
    emitHdr();
  }
}
