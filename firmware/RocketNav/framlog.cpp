#include "framlog.h"

#include <string.h>

namespace fl {

static const uint8_t kMagic[4] = { 'R', 'N', 'F', 'R' };
static const uint8_t kVersion = 1;

uint8_t crc8(const uint8_t *p, int n) {
  uint8_t c = 0;
  for (int i = 0; i < n; ++i) {
    c ^= p[i];
    for (int k = 0; k < 8; ++k) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

static void put16(uint8_t *p, int16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)((uint16_t)v >> 8); }
static void put32(uint8_t *p, int32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)((uint32_t)v >> 8);
  p[2] = (uint8_t)((uint32_t)v >> 16); p[3] = (uint8_t)((uint32_t)v >> 24);
}
static int16_t get16(const uint8_t *p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static int32_t get32(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}
static int16_t sat16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)(v + (v >= 0 ? 0.5f : -0.5f));
}
static int8_t sat8(float v) {
  if (v > 127.0f) return 127;
  if (v < -128.0f) return -128;
  return (int8_t)(v + (v >= 0 ? 0.5f : -0.5f));
}
static uint8_t satu8(float v) {
  if (v < 0) return 0;
  if (v > 255.0f) return 255;
  return (uint8_t)(v + 0.5f);
}
static uint16_t satu16(float v) {
  if (v < 0) return 0;
  if (v > 65535.0f) return 65535;
  return (uint16_t)(v + 0.5f);
}

// ---------- codecs ----------

void packHeader(const Header &h, uint8_t out[kHdrLen]) {
  memset(out, 0, kHdrLen);
  memcpy(out, kMagic, 4);
  out[4] = kVersion;
  out[5] = h.recording ? 1 : 0;
  put16(out + 6, (int16_t)h.boot_count);
  out[8] = h.flight;
  out[9] = h.reset_cause;
  out[10] = h.evt_head;
  out[11] = h.evt_n;
  out[12] = h.tl_n;
  out[13] = h.tr_head;
  out[14] = h.tr_n;
  out[15] = h.seq;
  out[16] = h.last_phase;
  put32(out + 20, (int32_t)h.hb_ms);
  out[kHdrLen - 1] = crc8(out, kHdrLen - 1);
}

bool unpackHeader(const uint8_t in[kHdrLen], Header *h) {
  memset(h, 0, sizeof(*h));
  if (memcmp(in, kMagic, 4) != 0 || in[4] != kVersion) return false;
  if (crc8(in, kHdrLen - 1) != in[kHdrLen - 1]) return false;
  h->recording = (in[5] & 1) != 0;
  h->boot_count = (uint16_t)get16(in + 6);
  h->flight = in[8];
  h->reset_cause = in[9];
  h->evt_head = in[10];
  h->evt_n = in[11];
  h->tl_n = in[12];
  h->tr_head = in[13];
  h->tr_n = in[14];
  h->seq = in[15];
  h->last_phase = in[16];
  h->hb_ms = (uint32_t)get32(in + 20);
  if (h->evt_head >= kEvtN || h->evt_n > kEvtN || h->tl_n > kTrajLaunchN ||
      h->tr_head >= kTrajRollN || h->tr_n > kTrajRollN) {
    return false;
  }
  return true;
}

void packEvent(const Event &e, uint8_t out[kEvtLen]) {
  memset(out, 0, kEvtLen);
  put32(out, (int32_t)e.t_ms);
  out[4] = e.flight;
  out[5] = e.phase;
  out[6] = e.code;
  put16(out + 7, e.arg);
  put32(out + 9, e.aux);
  out[13] = e.seq;
  out[kEvtLen - 1] = crc8(out, kEvtLen - 1);
}

bool unpackEvent(const uint8_t in[kEvtLen], Event *e) {
  if (crc8(in, kEvtLen - 1) != in[kEvtLen - 1]) return false;
  e->t_ms = (uint32_t)get32(in);
  e->flight = in[4];
  e->phase = in[5];
  e->code = in[6];
  e->arg = get16(in + 7);
  e->aux = get32(in + 9);
  e->seq = in[13];
  return e->code != 0;  // an all-zero record has a valid CRC but is empty
}

void packTraj(const Traj &t, uint8_t out[kTrajLen]) {
  memset(out, 0, kTrajLen);
  put32(out, (int32_t)t.t_ms);
  out[4] = t.flight;
  out[5] = t.phase;
  put16(out + 6, sat16(t.ral_m * 2.0f));
  put16(out + 8, sat16(t.vd_mps * 10.0f));
  put16(out + 10, (int16_t)satu16(t.spd_mps * 10.0f));
  for (int i = 0; i < 3; ++i) out[12 + i] = (uint8_t)sat8(t.eul_deg[i] * 0.5f);
  out[15] = satu8(t.amag_g * 10.0f);
  put16(out + 16, sat16(t.rollrate_dps));
  put16(out + 18, sat16(t.dn_m));
  put16(out + 20, sat16(t.de_m));
  out[22] = t.health;
  out[kTrajLen - 1] = crc8(out, kTrajLen - 1);
}

bool unpackTraj(const uint8_t in[kTrajLen], Traj *t) {
  if (crc8(in, kTrajLen - 1) != in[kTrajLen - 1]) return false;
  t->t_ms = (uint32_t)get32(in);
  t->flight = in[4];
  t->phase = in[5];
  t->ral_m = get16(in + 6) * 0.5f;
  t->vd_mps = get16(in + 8) * 0.1f;
  t->spd_mps = (uint16_t)get16(in + 10) * 0.1f;
  for (int i = 0; i < 3; ++i) t->eul_deg[i] = (int8_t)in[12 + i] * 2.0f;
  t->amag_g = in[15] * 0.1f;
  t->rollrate_dps = (float)get16(in + 16);
  t->dn_m = (float)get16(in + 18);
  t->de_m = (float)get16(in + 20);
  t->health = in[22];
  return t->t_ms != 0;
}

void packSummary(const Summary &s, uint8_t out[kSumLen]) {
  memset(out, 0, kSumLen);
  out[0] = s.valid ? 1 : 0;
  out[1] = s.flight;
  put32(out + 2, (int32_t)s.launch_ms);
  put16(out + 6, sat16(s.apogee_m * 2.0f));
  put32(out + 8, (int32_t)s.apogee_ms);
  out[12] = satu8(s.max_a_g * 10.0f);
  put16(out + 13, (int16_t)satu16(s.max_spd_mps * 10.0f));
  put16(out + 15, sat16(s.max_rollrate_dps));
  out[17] = satu8(s.max_tilt_deg);
  out[18] = (uint8_t)sat8(s.max_cdef_deg * 4.0f);
  out[19] = s.safe_reason;
  put32(out + 20, (int32_t)s.safe_ms);
  put32(out + 24, (int32_t)s.land_ms);
  put32(out + 28, (int32_t)(s.lat_deg * 1e7));
  put32(out + 32, (int32_t)(s.lon_deg * 1e7));
  put16(out + 36, sat16(s.alt_msl_m));
  put32(out + 38, (int32_t)s.dur_ms);
  out[42] = s.loss_pct;
  out[43] = (uint8_t)s.min_urssi;
  put16(out + 44, (int16_t)s.boot_count);
  out[kSumLen - 1] = crc8(out, kSumLen - 1);
}

bool unpackSummary(const uint8_t in[kSumLen], Summary *s) {
  memset(s, 0, sizeof(*s));
  if (crc8(in, kSumLen - 1) != in[kSumLen - 1]) return false;
  s->valid = (in[0] & 1) != 0;
  s->flight = in[1];
  s->launch_ms = (uint32_t)get32(in + 2);
  s->apogee_m = get16(in + 6) * 0.5f;
  s->apogee_ms = (uint32_t)get32(in + 8);
  s->max_a_g = in[12] * 0.1f;
  s->max_spd_mps = (uint16_t)get16(in + 13) * 0.1f;
  s->max_rollrate_dps = (float)get16(in + 15);
  s->max_tilt_deg = (float)in[17];
  s->max_cdef_deg = (int8_t)in[18] * 0.25f;
  s->safe_reason = in[19];
  s->safe_ms = (uint32_t)get32(in + 20);
  s->land_ms = (uint32_t)get32(in + 24);
  s->lat_deg = get32(in + 28) * 1e-7;
  s->lon_deg = get32(in + 32) * 1e-7;
  s->alt_msl_m = (float)get16(in + 36);
  s->dur_ms = (uint32_t)get32(in + 38);
  s->loss_pct = in[42];
  s->min_urssi = (int8_t)in[43];
  s->boot_count = (uint16_t)get16(in + 44);
  return s->valid;
}

// ---------- logger ----------

void FramLog::begin(const Store &s) {
  st_ = s;
  memset(&hdr_, 0, sizeof(hdr_));
  memset(sum_, 0, sizeof(sum_));
  q_head_ = q_n_ = 0;
  hdr_dirty_ = false;
  write_errs_ = dropped_ = 0;
  uint8_t buf[kSumLen];
  if (st_.read != nullptr && st_.read(st_.ctx, kHdrAddr, buf, kHdrLen)) {
    Header h;
    if (unpackHeader(buf, &h)) hdr_ = h;
  }
  for (uint8_t i = 0; i < kSumN; ++i) {
    Summary sm;
    if (st_.read != nullptr &&
        st_.read(st_.ctx, (uint16_t)(kSumAddr + i * kSumLen), buf, kSumLen) &&
        unpackSummary(buf, &sm)) {
      sum_[i] = sm;
    }
  }
}

bool FramLog::push(uint16_t addr, const uint8_t *data, uint8_t len) {
  if (q_n_ >= kQ) { dropped_++; return false; }
  Item &it = q_[(q_head_ + q_n_) % kQ];
  it.addr = addr;
  it.len = len;
  it.off = 0;
  memcpy(it.buf, data, len);
  q_n_++;
  return true;
}

void FramLog::queueHeader() { hdr_dirty_ = true; }

void FramLog::start(uint32_t t_ms, uint8_t phase) {
  hdr_.recording = true;
  hdr_.flight = 0;
  hdr_.evt_head = hdr_.evt_n = 0;
  hdr_.tl_n = hdr_.tr_head = hdr_.tr_n = 0;
  hdr_.seq = 0;
  hdr_.last_phase = phase;
  hdr_.hb_ms = t_ms;
  memset(sum_, 0, sizeof(sum_));
  uint8_t z[kSumLen];
  memset(z, 0, sizeof(z));
  for (uint8_t i = 0; i < kSumN; ++i) push((uint16_t)(kSumAddr + i * kSumLen), z, kSumLen);
  queueHeader();
  event(t_ms, phase, EV_REC_START, 0, 0);
}

void FramLog::stop(uint32_t t_ms, uint8_t phase) {
  if (!hdr_.recording) return;
  event(t_ms, phase, EV_REC_STOP, 0, 0);
  hdr_.recording = false;
  hdr_.last_phase = phase;
  hdr_.hb_ms = t_ms;
  queueHeader();
}

void FramLog::onBoot(uint32_t t_ms, uint8_t reset_cause) {
  if (!hdr_.recording) return;  // nothing was asked for: write nothing
  uint32_t last_hb = hdr_.hb_ms;
  uint8_t last_phase = hdr_.last_phase;
  hdr_.boot_count++;
  hdr_.reset_cause = reset_cause;
  event(t_ms, 0, EV_BOOT, (int16_t)(reset_cause | ((uint16_t)last_phase << 8)),
        (int32_t)last_hb);
}

void FramLog::event(uint32_t t_ms, uint8_t phase, uint8_t code, int16_t arg,
                    int32_t aux) {
  if (!hdr_.recording) return;
  Event e;
  e.t_ms = t_ms;
  e.flight = hdr_.flight;
  e.phase = phase;
  e.code = code;
  e.arg = arg;
  e.aux = aux;
  e.seq = hdr_.seq;
  uint8_t buf[kEvtLen];
  packEvent(e, buf);
  if (!push((uint16_t)(kEvtAddr + hdr_.evt_head * kEvtLen), buf, kEvtLen)) return;
  hdr_.evt_head = (uint8_t)((hdr_.evt_head + 1) % kEvtN);
  if (hdr_.evt_n < kEvtN) hdr_.evt_n++;
  hdr_.seq++;
  hdr_.last_phase = phase;
  queueHeader();
}

void FramLog::newFlight() {
  if (!hdr_.recording) return;
  hdr_.flight++;
  hdr_.tl_n = 0;  // the launch segment belongs to this flight
  queueHeader();
}

void FramLog::traj(const Traj &t) {
  if (!hdr_.recording) return;
  uint8_t buf[kTrajLen];
  Traj c = t;
  c.flight = hdr_.flight;
  packTraj(c, buf);
  if (hdr_.tl_n < kTrajLaunchN) {
    if (!push((uint16_t)(kTrajAddr + hdr_.tl_n * kTrajLen), buf, kTrajLen)) return;
    hdr_.tl_n++;
  } else {
    uint16_t base = (uint16_t)(kTrajAddr + kTrajLaunchN * kTrajLen);
    if (!push((uint16_t)(base + hdr_.tr_head * kTrajLen), buf, kTrajLen)) return;
    hdr_.tr_head = (uint8_t)((hdr_.tr_head + 1) % kTrajRollN);
    if (hdr_.tr_n < kTrajRollN) hdr_.tr_n++;
  }
  hdr_.last_phase = t.phase;
  queueHeader();
}

void FramLog::heartbeat(uint32_t t_ms, uint8_t phase) {
  if (!hdr_.recording) return;
  hdr_.hb_ms = t_ms;
  hdr_.last_phase = phase;
  queueHeader();
}

void FramLog::summary(const Summary &s) {
  if (!hdr_.recording) return;
  sum_[1] = sum_[0];
  sum_[0] = s;
  sum_[0].valid = true;
  uint8_t buf[kSumLen];
  packSummary(sum_[1], buf);
  push((uint16_t)(kSumAddr + kSumLen), buf, kSumLen);
  packSummary(sum_[0], buf);
  push(kSumAddr, buf, kSumLen);
}

bool FramLog::service() {
  if (st_.write == nullptr) return false;
  if (q_n_ == 0) {
    if (!hdr_dirty_) return false;
    // The header goes last, after every record it points at.
    uint8_t buf[kHdrLen];
    packHeader(hdr_, buf);
    hdr_dirty_ = false;
    push(kHdrAddr, buf, kHdrLen);
  }
  Item &it = q_[q_head_];
  uint8_t k = (uint8_t)(it.len - it.off);
  if (k > kChunk) k = kChunk;
  bool ok = st_.write(st_.ctx, (uint16_t)(it.addr + it.off), it.buf + it.off, k);
  if (!ok) write_errs_++;
  it.off = (uint8_t)(it.off + k);
  if (it.off >= it.len || !ok) {  // done, or give up on this item
    q_head_ = (uint8_t)((q_head_ + 1) % kQ);
    q_n_--;
  }
  return true;
}

bool FramLog::readEvent(uint8_t i, Event *e) {
  if (i >= hdr_.evt_n || st_.read == nullptr) return false;
  uint8_t oldest = (uint8_t)((hdr_.evt_head + kEvtN - hdr_.evt_n) % kEvtN);
  uint8_t idx = (uint8_t)((oldest + i) % kEvtN);
  uint8_t buf[kEvtLen];
  if (!st_.read(st_.ctx, (uint16_t)(kEvtAddr + idx * kEvtLen), buf, kEvtLen)) return false;
  return unpackEvent(buf, e);
}

bool FramLog::readTraj(uint8_t i, Traj *t) {
  if (st_.read == nullptr) return false;
  uint16_t addr;
  if (i < hdr_.tl_n) {
    addr = (uint16_t)(kTrajAddr + i * kTrajLen);
  } else {
    uint8_t j = (uint8_t)(i - hdr_.tl_n);
    if (j >= hdr_.tr_n) return false;
    uint8_t oldest = (uint8_t)((hdr_.tr_head + kTrajRollN - hdr_.tr_n) % kTrajRollN);
    uint8_t idx = (uint8_t)((oldest + j) % kTrajRollN);
    addr = (uint16_t)(kTrajAddr + kTrajLaunchN * kTrajLen + idx * kTrajLen);
  }
  uint8_t buf[kTrajLen];
  if (!st_.read(st_.ctx, addr, buf, kTrajLen)) return false;
  return unpackTraj(buf, t);
}

bool FramLog::readSummary(uint8_t slot, Summary *s) const {
  if (slot >= kSumN) return false;
  *s = sum_[slot];
  return s->valid;
}

bool FramLog::readRaw(uint16_t addr, uint8_t *buf, uint16_t n) {
  if (st_.read == nullptr) return false;
  return st_.read(st_.ctx, addr, buf, n);
}

bool FramLog::scratchTest() {
  if (st_.read == nullptr || st_.write == nullptr) return false;
  uint8_t pat[kScratchLen], back[kScratchLen];
  for (int i = 0; i < kScratchLen; ++i) pat[i] = (uint8_t)(0xA5 ^ (i * 0x31));
  if (!st_.write(st_.ctx, kScratchAddr, pat, kScratchLen)) return false;
  if (!st_.read(st_.ctx, kScratchAddr, back, kScratchLen)) return false;
  if (memcmp(pat, back, kScratchLen) != 0) return false;
  for (int i = 0; i < kScratchLen; ++i) pat[i] = (uint8_t)~pat[i];
  if (!st_.write(st_.ctx, kScratchAddr, pat, kScratchLen)) return false;
  if (!st_.read(st_.ctx, kScratchAddr, back, kScratchLen)) return false;
  return memcmp(pat, back, kScratchLen) == 0;
}

}  // namespace fl
