// RocketNav base station - ESP32-C3 + Ai-Thinker Ra-02 (SX1278).
//
// Transparent bridge, no UI of its own:
//   LoRa downlink ('T' state / 'M' message frames)  ->  NDJSON on USB serial,
//   the same records the viewer already speaks - connect the viewer to THIS
//   port and the field screen runs with zero protocol changes.
//   '$command' lines typed into the viewer console  ->  'C' uplink frames
//   (sent twice; the rocket deduplicates by sequence number).
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
#define UPLINK_CUE_MS 30
#define UPLINK_BLIND_MS 400
static uint8_t up_seq_ = 1;
static uint8_t pend_[lc::kMaxFrame];
static int pend_len_ = 0;
static uint8_t pend_left_ = 0;         // transmissions still owed (2, 1, 0)
static uint32_t pend_send_at_ms_ = 0;  // earliest send time (cue or blind)

static char line_[96];
static uint8_t line_len_ = 0;

static void emitMsg(const char *txt) {
  Serial.printf("{\"t\":\"msg\",\"us\":%lu,\"txt\":\"%s\"}\n",
                (unsigned long)millis() * 1000ul, txt);
}

static void emitHdr() {
  Serial.print(
      "{\"t\":\"hdr\",\"us\":0,\"fw\":\"base-0.1.0\",\"schema\":\"lora-1\","
      "\"mode\":\"lora\",\"ohz\":4}\n");
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

static void emitState(const lc::StateFields &f, uint8_t seq) {
  bumpSeq(seq);
  char b[640];
  char geo[96];
  if (f.lat_deg != 0 || f.lon_deg != 0) {
    snprintf(geo, sizeof(geo), "\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f",
             f.lat_deg, f.lon_deg, (double)f.alt_msl_m);
  } else {
    snprintf(geo, sizeof(geo), "\"lat\":null,\"lon\":null,\"alt\":null");
  }
  // Validity mirrors the rocket's USB record (telemetry.cpp): v is null
  // unless the filter is in RUN (3); the vertical channel ral/rvs is null
  // outside WAIT_FIX (2) / RUN (3) / ATT_ONLY (4). The air frame packs the
  // raw filter numbers regardless (horizontal velocity is meaningless in
  // attitude-only mode), so the gate has to live here.
  char vs[56], vch[48];
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
           "\"rssi\":%d,\"snr\":%.1f,\"loss\":%.1f}",
           (unsigned long)f.ms * 1000ul, (unsigned long)seq32_, f.fst,
           (double)f.eul_deg[0], (double)f.eul_deg[1], (double)f.eul_deg[2],
           (double)(f.gyr_dps[0] * 0.017453293f),
           (double)(f.gyr_dps[1] * 0.017453293f),
           (double)(f.gyr_dps[2] * 0.017453293f),
           (double)(f.acc_g[0] * 9.80665f), (double)(f.acc_g[1] * 9.80665f),
           (double)(f.acc_g[2] * 9.80665f), vs, vch, geo, f.fix ? 3 : 0,
           f.sats,
           (double)f.hacc_m, hi, hm, hb, hg, f.cmode, (double)f.cdef_deg[0],
           (double)f.cdef_deg[1], (double)f.cdef_deg[2],
           (double)f.cdef_deg[3], (int)radio.pktRssiDbm(),
           (double)radio.pktSnrDb(), (double)lossPct());
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
  pend_send_at_ms_ = millis() + UPLINK_BLIND_MS;
}

// Called every loop pass; downlink_heard = a frame was just read out of the
// radio, i.e. the rocket has just finished transmitting.
static void serviceUplink(bool downlink_heard) {
  if (pend_left_ == 0) return;
  uint32_t ms = millis();
  if (downlink_heard) {  // pull the send time in to the cue (never push out)
    uint32_t cue = ms + UPLINK_CUE_MS;
    if ((int32_t)(cue - pend_send_at_ms_) < 0) pend_send_at_ms_ = cue;
  }
  if ((int32_t)(ms - pend_send_at_ms_) < 0 || radio.txBusy()) return;
  if (radio.txStart(pend_, (uint8_t)pend_len_)) {
    pend_left_--;
    pend_send_at_ms_ = ms + UPLINK_BLIND_MS;  // the repeat waits the same way
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
  if (strcmp(s, "$base?") == 0) {
    char b[112];
    snprintf(b, sizeof(b),
             "base: radio %s rx %lu lost %lu crcerr %lu rssi %d snr x10 %d",
             radio.present() ? "ok" : "ABSENT", (unsigned long)rx_frames_,
             (unsigned long)lost_frames_, (unsigned long)crc_errs_,
             (int)radio.pktRssiDbm(), (int)(radio.pktSnrDb() * 10.0f));
    emitMsg(b);
    return;
  }
  sendCmd(s);
  char b[96];
  snprintf(b, sizeof(b), "base: uplinked %s", s);
  emitMsg(b);
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
  emitHdr();
  emitMsg(ok ? "base: SX1278 OK - listening 433.5 MHz SF7/500k"
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
      uint8_t seq = 0, type = 0;
      char txt[lc::kMaxText + 1];
      if (lc::parseState(buf, n, &f, &seq)) {
        emitState(f, seq);
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

  // quiet-link heartbeat so a dead rocket link is obvious in the console
  uint32_t ms = millis();
  if (rx_frames_ == 0 || ms - last_rx_ms_ > 5000) {
    if (ms >= next_wait_note_ms_) {
      next_wait_note_ms_ = ms + 5000;
      emitMsg(rx_frames_ == 0 ? "base: waiting for first downlink frame"
                              : "base: downlink quiet > 5 s");
    }
  }
}
