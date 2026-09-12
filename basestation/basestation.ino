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

// Uplink: forward once immediately, repeat once 150 ms later (the rocket
// listens whenever it is not transmitting; the duplicate is dropped by seq).
static uint8_t up_seq_ = 1;
static uint8_t pend_[lc::kMaxFrame];
static int pend_len_ = 0;
static uint32_t pend_at_ms_ = 0;

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
  // Health: the frame carries fresh bits only; present(1)+fresh(2) when
  // fresh, bare present otherwise - the viewer's stale logic reads bit 1.
  int hi = (f.health & 1) ? 3 : 1, hm = (f.health & 2) ? 3 : 1;
  int hb = (f.health & 4) ? 3 : 1, hg = (f.health & 8) ? 3 : 1;
  snprintf(b, sizeof(b),
           "{\"t\":\"st\",\"us\":%lu,\"seq\":%lu,\"fst\":%u,"
           "\"eul\":[%.2f,%.2f,%.2f],"
           "\"gyr\":[%.5f,%.5f,%.5f],"
           "\"acc\":[%.3f,%.3f,%.3f],"
           "\"v\":[%.2f,%.2f,%.2f],"
           "\"ral\":%.1f,\"rvs\":%.2f,%s,"
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
           (double)(f.acc_g[2] * 9.80665f), (double)f.vel_mps[0],
           (double)f.vel_mps[1], (double)f.vel_mps[2], (double)f.ral_m,
           (double)(-f.vel_mps[2]), geo, f.fix ? 3 : 0, f.sats,
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
  if (radio.txStart(pend_, (uint8_t)pend_len_)) {
    pend_at_ms_ = millis() + 150;  // one repeat, then forget
  } else {
    pend_at_ms_ = millis() + 30;   // radio busy: retry shortly
  }
}

static void handleLine(char *s) {
  if (s[0] != '$') return;
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
  if (ev & Sx1278::EV_RXDONE) {
    uint8_t buf[lc::kMaxFrame];
    int n = radio.rxRead(buf, sizeof(buf));
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

  // one deferred uplink repeat
  if (pend_len_ > 0 && millis() >= pend_at_ms_ && !radio.txBusy()) {
    radio.txStart(pend_, (uint8_t)pend_len_);
    pend_len_ = 0;
  }

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
