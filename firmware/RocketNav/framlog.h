// framlog - the flight black box kept in the 2 kB FRAM. Pure: no hardware,
// no Arduino; the store is a pair of read/write callbacks, so the host test
// runs the identical code over a 2048-byte array.
//
// Recording is OPT-IN. Nothing is written until start() (the viewer's
// Record button, `$fram start` on USB or LoRa); stop() ends it. The
// recording flag lives in the header, so a reset mid-flight resumes and
// logs a BOOT event with the reset cause and the last phase seen. Errors
// that happen while not recording are not written anywhere.
//
// Layout (2048 B):
//   0x000  32  header (counters, cursors, recording flag, 1 Hz heartbeat)
//   0x040 256  reserved: config mirror (RNV2 record), not used yet
//   0x140 512  event ring, 32 x 16 B
//   0x340 128  flight summaries, 2 x 64 B (slot 0 = latest)
//   0x3C0 1080 trajectory: launch segment 21 x 24 B (written once per
//              flight), then a rolling ring 24 x 24 B (always the latest)
//   0x7F8   8  scratch for $fram test
// Every record carries CRC-8, so a write torn by a power loss is skipped on
// read. The header is written after the record it points at.
//
// Writes are queued and drained by service(), ONE bounded store write per
// call (<= 28 B), so the scheduler slot that calls it costs under a
// millisecond and no FRAM traffic ever lands in the IMU branch.
#pragma once

#include <stdint.h>

namespace fl {

const uint16_t kSize = 2048;
const uint16_t kHdrAddr = 0x000;
const uint16_t kHdrLen = 32;
const uint16_t kCfgAddr = 0x040;
const uint16_t kCfgLen = 256;
const uint16_t kEvtAddr = 0x140;
const uint8_t kEvtN = 32;
const uint16_t kEvtLen = 16;
const uint16_t kSumAddr = 0x340;
const uint8_t kSumN = 2;
const uint16_t kSumLen = 64;
const uint16_t kTrajAddr = 0x3C0;
const uint8_t kTrajLaunchN = 21;
const uint8_t kTrajRollN = 24;
const uint16_t kTrajLen = 24;
const uint16_t kScratchAddr = 0x7F8;
const uint16_t kScratchLen = 8;
const uint8_t kChunk = 28;  // one store write per service() call
static_assert(kHdrAddr + kHdrLen <= kCfgAddr, "layout");
static_assert(kCfgAddr + kCfgLen == kEvtAddr, "layout");
static_assert(kEvtAddr + kEvtN * kEvtLen == kSumAddr, "layout");
static_assert(kSumAddr + kSumN * kSumLen == kTrajAddr, "layout");
static_assert(kTrajAddr + (kTrajLaunchN + kTrajRollN) * kTrajLen == kScratchAddr, "layout");
static_assert(kScratchAddr + kScratchLen == kSize, "layout");

// Event codes are on the dump: append, never renumber.
enum : uint8_t {
  EV_BOOT = 0x01,           // arg = reset cause | last phase << 8, aux = last heartbeat ms
  EV_REC_START = 0x02,
  EV_ALIGN_DONE = 0x03,     // aux = heading deg x100
  EV_ALIGN_DEGRADED = 0x04,
  EV_GPS_FIX = 0x05,        // arg = sats
  EV_GPS_LOST = 0x06,
  EV_ORIGIN = 0x07,         // aux = alt MSL m
  EV_ARM = 0x10,
  EV_DISARM = 0x11,
  EV_LAUNCH = 0x12,         // aux = |a| g x100
  EV_APOGEE = 0x13,         // aux = AGL m x10 (written at landing from the summary)
  EV_SAFE = 0x14,           // arg = reason (control: 1 imu 2 tilt 3 descent 4 timeout)
  EV_LANDED = 0x15,         // aux = AGL m x10
  EV_BENCH = 0x16,          // arg = 1 start, 0 stop
  EV_SENSOR_FAULT = 0x20,   // arg = sensor (0 imu 1 mag 2 baro 3 gnss), aux = health bits
  EV_SENSOR_OK = 0x21,
  EV_BUS_RESET = 0x22,      // aux = bus reset count
  EV_PCA_ABSENT = 0x23,
  EV_PCA_OK = 0x24,
  EV_LORA_PROFILE = 0x30,   // arg = profile
  EV_LORA_BASE_LOST = 0x31,
  EV_CFG_SAVED = 0x40,      // arg = 1 mag, 2 linkage; aux = ok
  EV_CFG_FAIL = 0x41,
  EV_REC_STOP = 0x7E,
  EV_MARK = 0x7F,           // arg = the user's number
};

struct Store {
  bool (*read)(void *ctx, uint16_t addr, uint8_t *buf, uint16_t n);
  bool (*write)(void *ctx, uint16_t addr, const uint8_t *buf, uint16_t n);
  void *ctx;
};

struct Header {
  bool recording;
  uint16_t boot_count;
  uint8_t flight;         // launches since start()
  uint8_t reset_cause;    // of the most recent boot while recording
  uint8_t evt_head, evt_n;
  uint8_t tl_n;           // launch-segment records written this flight
  uint8_t tr_head, tr_n;  // rolling ring
  uint8_t seq;            // next event seq
  uint8_t last_phase;
  uint32_t hb_ms;         // heartbeat: last time the firmware was alive
};

struct Event {
  uint32_t t_ms;
  uint8_t flight, phase, code;
  int16_t arg;
  int32_t aux;
  uint8_t seq;
};

// Quantization on the wire: ral 0.5 m | vd/spd 0.1 m/s | eul 2 deg |
// amag 0.1 g | rollrate 1 dps | dN/dE 1 m.
struct Traj {
  uint32_t t_ms;
  uint8_t flight, phase;
  float ral_m, vd_mps, spd_mps;
  float eul_deg[3];
  float amag_g;
  float rollrate_dps;
  float dn_m, de_m;
  uint8_t health;
};

struct Summary {
  bool valid;
  uint8_t flight;
  uint32_t launch_ms;
  float apogee_m;
  uint32_t apogee_ms;
  float max_a_g, max_spd_mps, max_rollrate_dps, max_tilt_deg, max_cdef_deg;
  uint8_t safe_reason;
  uint32_t safe_ms, land_ms;
  double lat_deg, lon_deg;
  float alt_msl_m;
  uint32_t dur_ms;
  uint8_t loss_pct;
  int8_t min_urssi;
  uint16_t boot_count;
};

uint8_t crc8(const uint8_t *p, int n);
void packHeader(const Header &h, uint8_t out[kHdrLen]);
bool unpackHeader(const uint8_t in[kHdrLen], Header *h);
void packEvent(const Event &e, uint8_t out[kEvtLen]);
bool unpackEvent(const uint8_t in[kEvtLen], Event *e);
void packTraj(const Traj &t, uint8_t out[kTrajLen]);
bool unpackTraj(const uint8_t in[kTrajLen], Traj *t);
void packSummary(const Summary &s, uint8_t out[kSumLen]);
bool unpackSummary(const uint8_t in[kSumLen], Summary *s);

class FramLog {
 public:
  // Reads the header; an invalid one means "not recording, empty".
  void begin(const Store &s);
  bool recording() const { return hdr_.recording; }
  const Header &header() const { return hdr_; }

  // Control. Everything reaches the store through service().
  void start(uint32_t t_ms, uint8_t phase);   // wipe counters, record on
  void stop(uint32_t t_ms, uint8_t phase);    // record off
  void onBoot(uint32_t t_ms, uint8_t reset_cause);  // resumes a recording
  void event(uint32_t t_ms, uint8_t phase, uint8_t code, int16_t arg,
             int32_t aux);
  void newFlight();                           // at launch: fresh segment
  void traj(const Traj &t);
  void heartbeat(uint32_t t_ms, uint8_t phase);
  void summary(const Summary &s);             // slot 0 <- s, slot 1 <- old 0

  // One bounded store write per call. Returns true if it wrote.
  bool service();
  bool idle() const { return q_n_ == 0 && !hdr_dirty_; }
  uint32_t writeErrors() const { return write_errs_; }
  uint32_t dropped() const { return dropped_; }

  // Reading (synchronous, one or two store reads each; for the dump).
  uint8_t eventCount() const { return hdr_.evt_n; }
  bool readEvent(uint8_t i, Event *e);        // oldest first
  uint8_t trajCount() const { return (uint8_t)(hdr_.tl_n + hdr_.tr_n); }
  bool readTraj(uint8_t i, Traj *t);          // launch segment, then ring
  bool readSummary(uint8_t slot, Summary *s) const;
  bool readRaw(uint16_t addr, uint8_t *buf, uint16_t n);
  bool scratchTest();                         // write/read on the spare bytes

 private:
  struct Item { uint16_t addr; uint8_t len, off; uint8_t buf[kSumLen]; };
  static const uint8_t kQ = 16;  // a 10 Hz burst of transitions never drops
  bool push(uint16_t addr, const uint8_t *data, uint8_t len);
  void queueHeader();

  Store st_;
  Header hdr_;
  Summary sum_[kSumN];
  Item q_[kQ];
  uint8_t q_head_ = 0, q_n_ = 0;
  bool hdr_dirty_ = false;
  uint32_t write_errs_ = 0, dropped_ = 0;
};

}  // namespace fl
