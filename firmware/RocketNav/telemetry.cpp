// telemetry.cpp — JSON building and the never-blocking output ring.
// See telemetry.h for sizing and SCHEMA.md for the field contract.

#include "telemetry.h"
#include <math.h>
#include <string.h>

namespace telem {

static HardwareSerial *port_ = nullptr;
static uint8_t ring_[TELEM_RING_BYTES];
static volatile uint32_t head_ = 0;  // write index
static volatile uint32_t tail_ = 0;  // read index
static uint32_t dropped_ = 0;
static char line_[TELEM_LINE_BYTES];

void init(HardwareSerial *port) {
  port_ = port;
  head_ = tail_ = 0;
  dropped_ = 0;
}

uint32_t droppedRecords() { return dropped_; }

// ---------- ring ----------

static uint32_t ringFree() {
  uint32_t h = head_, t = tail_;
  uint32_t used = (h >= t) ? (h - t) : (TELEM_RING_BYTES - t + h);
  return TELEM_RING_BYTES - 1 - used;
}

static void ringPush(const char *data, uint32_t len) {
  if (len == 0) return;
  if (ringFree() < len) {  // whole-record drop, counted, never block
    dropped_++;
    return;
  }
  uint32_t h = head_;
  uint32_t first = TELEM_RING_BYTES - h;
  if (first > len) first = len;
  memcpy(&ring_[h], data, first);
  if (len > first) memcpy(&ring_[0], data + first, len - first);
  head_ = (h + len) % TELEM_RING_BYTES;
}

void drain() {
  if (port_ == nullptr) return;
  uint32_t t = tail_, h = head_;
  while (t != h) {
    int afw = port_->availableForWrite();
    if (afw <= 0) break;
    uint32_t contig = (h > t) ? (h - t) : (TELEM_RING_BYTES - t);
    uint32_t n = contig < (uint32_t)afw ? contig : (uint32_t)afw;
    size_t wrote = port_->write(&ring_[t], n);
    t = (t + (uint32_t)wrote) % TELEM_RING_BYTES;
    if (wrote < n) break;
  }
  tail_ = t;
}

// ---------- JSON appenders (bounds-checked, no printf floats) ----------

static char *ap_raw(char *p, char *end, const char *s) {
  while (*s && p < end) *p++ = *s++;
  return p;
}

static char *ap_key(char *p, char *end, const char *k) {
  p = ap_raw(p, end, ",\"");
  p = ap_raw(p, end, k);
  p = ap_raw(p, end, "\":");
  return p;
}

static char *ap_u64(char *p, char *end, uint64_t v) {
  char buf[21];
  int i = 0;
  do {
    buf[i++] = (char)('0' + (v % 10ull));
    v /= 10ull;
  } while (v && i < 20);
  while (i > 0 && p < end) *p++ = buf[--i];
  return p;
}

static char *ap_i32(char *p, char *end, long v) {
  if (v < 0) {
    if (p < end) *p++ = '-';
    return ap_u64(p, end, (uint64_t)(-(int64_t)v));
  }
  return ap_u64(p, end, (uint64_t)v);
}

static const double kPow10[8] = { 1.0, 10.0, 100.0, 1000.0, 10000.0,
                                  100000.0, 1000000.0, 10000000.0 };

static char *ap_f(char *p, char *end, double v, int dec) {
  if (!isfinite(v) || fabs(v) >= 4.0e9) return ap_raw(p, end, "null");
  if (dec < 0) dec = 0;
  if (dec > 7) dec = 7;
  bool neg = v < 0;
  if (neg) v = -v;
  double scale = kPow10[dec];
  uint64_t total = (uint64_t)(v * scale + 0.5);
  uint64_t ip = total / (uint64_t)scale;
  uint64_t fp = total - ip * (uint64_t)scale;
  if (neg && total > 0 && p < end) *p++ = '-';
  p = ap_u64(p, end, ip);
  if (dec > 0) {
    if (p < end) *p++ = '.';
    char buf[8];
    for (int i = dec - 1; i >= 0; --i) {
      buf[i] = (char)('0' + (fp % 10ull));
      fp /= 10ull;
    }
    for (int i = 0; i < dec && p < end; ++i) *p++ = buf[i];
  }
  return p;
}

static char *ap_null(char *p, char *end) { return ap_raw(p, end, "null"); }

static char *ap_bool01(char *p, char *end, bool b) {
  return ap_raw(p, end, b ? "1" : "0");
}

static char *ap_arr3(char *p, char *end, const float *v, int dec) {
  if (p < end) *p++ = '[';
  p = ap_f(p, end, v[0], dec);
  if (p < end) *p++ = ',';
  p = ap_f(p, end, v[1], dec);
  if (p < end) *p++ = ',';
  p = ap_f(p, end, v[2], dec);
  if (p < end) *p++ = ']';
  return p;
}

static void finishLine(char *p, char *end) {
  if (p < end) *p++ = '}';
  if (p < end) *p++ = '\n';
  ringPush(line_, (uint32_t)(p - line_));
}

// ---------- records ----------

void emitHeader(const TelemetryHeaderInfo &h) {
  char *p = line_, *end = line_ + TELEM_LINE_BYTES - 2;
  p = ap_raw(p, end, "{\"t\":\"hdr\"");
  p = ap_key(p, end, "us"); p = ap_u64(p, end, h.us);
  p = ap_key(p, end, "fw"); p = ap_raw(p, end, "\"");
  p = ap_raw(p, end, h.fw); p = ap_raw(p, end, "\"");
  p = ap_key(p, end, "sch"); p = ap_i32(p, end, h.schema);
  p = ap_key(p, end, "imu"); p = ap_bool01(p, end, h.imu);
  p = ap_key(p, end, "mag"); p = ap_bool01(p, end, h.mag);
  p = ap_key(p, end, "bar"); p = ap_bool01(p, end, h.bar);
  p = ap_key(p, end, "gps"); p = ap_bool01(p, end, h.gps);
  p = ap_key(p, end, "iodr"); p = ap_i32(p, end, h.iodr);
  p = ap_key(p, end, "afs"); p = ap_i32(p, end, h.afs);
  p = ap_key(p, end, "gfs"); p = ap_i32(p, end, h.gfs);
  p = ap_key(p, end, "modr"); p = ap_i32(p, end, h.modr);
  p = ap_key(p, end, "mmode"); p = ap_raw(p, end, "\"");
  p = ap_raw(p, end, h.mmode); p = ap_raw(p, end, "\"");
  p = ap_key(p, end, "gmode"); p = ap_raw(p, end, "\"");
  p = ap_raw(p, end, h.gmode); p = ap_raw(p, end, "\"");
  p = ap_key(p, end, "bhz"); p = ap_i32(p, end, h.bhz);
  p = ap_key(p, end, "ghz"); p = ap_i32(p, end, h.ghz);
  p = ap_key(p, end, "ohz"); p = ap_i32(p, end, h.ohz);
  finishLine(p, end);
}

static void (*msg_tap_)(const char *txt) = nullptr;
void setMsgTap(void (*tap)(const char *txt)) { msg_tap_ = tap; }

void emitMsg(uint64_t us, const char *txt) {
  if (msg_tap_ != nullptr) msg_tap_(txt);
  char *p = line_, *end = line_ + TELEM_LINE_BYTES - 2;
  p = ap_raw(p, end, "{\"t\":\"msg\"");
  p = ap_key(p, end, "us"); p = ap_u64(p, end, us);
  p = ap_key(p, end, "txt"); p = ap_raw(p, end, "\"");
  p = ap_raw(p, end, txt); p = ap_raw(p, end, "\"");
  finishLine(p, end);
}

void emitLcal(uint64_t us, int8_t fin, uint8_t src, uint8_t n,
              const float deg[5], const float pus[5]) {
  char *p = line_, *end = line_ + TELEM_LINE_BYTES - 2;
  p = ap_raw(p, end, "{\"t\":\"lcal\"");
  p = ap_key(p, end, "us"); p = ap_u64(p, end, us);
  p = ap_key(p, end, "fin"); p = ap_f(p, end, (float)fin, 0);
  p = ap_key(p, end, "src"); p = ap_u64(p, end, src);
  p = ap_key(p, end, "n"); p = ap_u64(p, end, n);
  p = ap_key(p, end, "deg");
  p = ap_raw(p, end, "[");
  for (int i = 0; i < (int)n; ++i) {
    if (i) p = ap_raw(p, end, ",");
    p = ap_f(p, end, deg[i], 1);
  }
  p = ap_raw(p, end, "]");
  p = ap_key(p, end, "pus");
  p = ap_raw(p, end, "[");
  for (int i = 0; i < (int)n; ++i) {
    if (i) p = ap_raw(p, end, ",");
    p = ap_f(p, end, pus[i], 0);
  }
  p = ap_raw(p, end, "]");
  finishLine(p, end);
}

void emitCal(const TelemetryCalInfo &c) {
  char *p = line_, *end = line_ + TELEM_LINE_BYTES - 2;
  p = ap_raw(p, end, "{\"t\":\"cal\"");
  p = ap_key(p, end, "us"); p = ap_u64(p, end, c.us);
  p = ap_key(p, end, "bg"); p = ap_arr3(p, end, c.bg_dps, 4);
  p = ap_key(p, end, "eul"); p = ap_arr3(p, end, c.eul_deg, 2);
  p = ap_key(p, end, "alq"); p = ap_bool01(p, end, c.alq);
  p = ap_key(p, end, "dur"); p = ap_f(p, end, c.dur_s, 2);
  finishLine(p, end);
}

void emitOrigin(const TelemetryOriginInfo &o) {
  char *p = line_, *end = line_ + TELEM_LINE_BYTES - 2;
  p = ap_raw(p, end, "{\"t\":\"org\"");
  p = ap_key(p, end, "us"); p = ap_u64(p, end, o.us);
  p = ap_key(p, end, "lat"); p = ap_f(p, end, o.lat, 7);
  p = ap_key(p, end, "lon"); p = ap_f(p, end, o.lon, 7);
  p = ap_key(p, end, "alt"); p = ap_f(p, end, o.alt, 2);
  p = ap_key(p, end, "hae"); p = ap_f(p, end, o.hae, 2);
  p = ap_key(p, end, "g0"); p = ap_f(p, end, o.g0, 4);
  p = ap_key(p, end, "p0"); p = ap_f(p, end, o.p0, 1);
  p = ap_key(p, end, "t0"); p = ap_f(p, end, o.t0, 2);
  finishLine(p, end);
}

void emitState(const TelemetryStateInfo &s) {
  char *p = line_, *end = line_ + TELEM_LINE_BYTES - 2;
  p = ap_raw(p, end, "{\"t\":\"st\"");
  p = ap_key(p, end, "us"); p = ap_u64(p, end, s.us);
  p = ap_key(p, end, "seq"); p = ap_u64(p, end, s.seq);
  p = ap_key(p, end, "fst"); p = ap_i32(p, end, s.fst);
  p = ap_key(p, end, "alq"); p = ap_bool01(p, end, s.alq);

  p = ap_key(p, end, "lat");
  if (s.geo_valid) p = ap_f(p, end, s.lat, 7); else p = ap_null(p, end);
  p = ap_key(p, end, "lon");
  if (s.geo_valid) p = ap_f(p, end, s.lon, 7); else p = ap_null(p, end);
  p = ap_key(p, end, "alt");
  if (s.geo_valid) p = ap_f(p, end, s.alt, 2); else p = ap_null(p, end);

  p = ap_key(p, end, "p");
  if (s.nav_valid) p = ap_arr3(p, end, s.p, 3); else p = ap_null(p, end);
  p = ap_key(p, end, "v");
  if (s.nav_valid) p = ap_arr3(p, end, s.v, 3); else p = ap_null(p, end);
  p = ap_key(p, end, "q");
  if (p < end) *p++ = '[';
  p = ap_f(p, end, s.q[0], 5);
  if (p < end) *p++ = ',';
  p = ap_f(p, end, s.q[1], 5);
  if (p < end) *p++ = ',';
  p = ap_f(p, end, s.q[2], 5);
  if (p < end) *p++ = ',';
  p = ap_f(p, end, s.q[3], 5);
  if (p < end) *p++ = ']';
  p = ap_key(p, end, "eul"); p = ap_arr3(p, end, s.eul_deg, 2);

  p = ap_key(p, end, "sp");
  if (s.nav_valid) p = ap_arr3(p, end, s.sp, 3); else p = ap_null(p, end);
  p = ap_key(p, end, "sv");
  if (s.nav_valid) p = ap_arr3(p, end, s.sv, 3); else p = ap_null(p, end);
  p = ap_key(p, end, "sa"); p = ap_arr3(p, end, s.sa_deg, 3);

  p = ap_key(p, end, "bg"); p = ap_arr3(p, end, s.bg, 5);
  p = ap_key(p, end, "ba"); p = ap_arr3(p, end, s.ba, 4);
  p = ap_key(p, end, "bb");
  if (s.bb_valid) p = ap_f(p, end, s.bb, 2); else p = ap_null(p, end);
  p = ap_key(p, end, "ral");
  if (s.vch_valid) p = ap_f(p, end, s.ral, 2); else p = ap_null(p, end);
  p = ap_key(p, end, "rvs");
  if (s.vch_valid) p = ap_f(p, end, s.rvs, 2); else p = ap_null(p, end);
  p = ap_key(p, end, "sra");
  if (s.vch_valid) p = ap_f(p, end, s.sra, 2); else p = ap_null(p, end);
  p = ap_key(p, end, "srv");
  if (s.vch_valid) p = ap_f(p, end, s.srv, 2); else p = ap_null(p, end);

  p = ap_key(p, end, "acc");
  if (s.imu_valid) p = ap_arr3(p, end, s.acc, 3); else p = ap_null(p, end);
  p = ap_key(p, end, "gyr");
  if (s.imu_valid) p = ap_arr3(p, end, s.gyr, 4); else p = ap_null(p, end);
  p = ap_key(p, end, "mag");
  if (s.mag_valid) p = ap_arr3(p, end, s.mag, 2); else p = ap_null(p, end);
  p = ap_key(p, end, "mgr");
  if (s.mgr_valid) p = ap_arr3(p, end, s.mgr, 2); else p = ap_null(p, end);
  p = ap_key(p, end, "pa");
  if (s.baro_valid) p = ap_f(p, end, s.pa, 1); else p = ap_null(p, end);
  p = ap_key(p, end, "tc");
  if (s.baro_valid) p = ap_f(p, end, s.tc, 2); else p = ap_null(p, end);

  p = ap_key(p, end, "gfix");
  if (s.gps_seen) p = ap_i32(p, end, s.gfix); else p = ap_null(p, end);
  p = ap_key(p, end, "gsv");
  if (s.gps_seen) p = ap_i32(p, end, s.gsv); else p = ap_null(p, end);
  p = ap_key(p, end, "ghac");
  if (s.gps_seen) p = ap_f(p, end, s.ghac, 2); else p = ap_null(p, end);
  p = ap_key(p, end, "gvac");
  if (s.gps_seen) p = ap_f(p, end, s.gvac, 2); else p = ap_null(p, end);
  p = ap_key(p, end, "gsac");
  if (s.gps_seen) p = ap_f(p, end, s.gsac, 2); else p = ap_null(p, end);
  p = ap_key(p, end, "glat");
  if (s.gps_seen) p = ap_f(p, end, s.glat, 7); else p = ap_null(p, end);
  p = ap_key(p, end, "glon");
  if (s.gps_seen) p = ap_f(p, end, s.glon, 7); else p = ap_null(p, end);
  p = ap_key(p, end, "galt");
  if (s.gps_seen) p = ap_f(p, end, s.galt, 2); else p = ap_null(p, end);
  p = ap_key(p, end, "gage");
  if (s.gps_seen) p = ap_f(p, end, s.gage, 2); else p = ap_null(p, end);

  p = ap_key(p, end, "igp");
  if (s.igp_v) p = ap_arr3(p, end, s.igp, 2); else p = ap_null(p, end);
  p = ap_key(p, end, "igv");
  if (s.igv_v) p = ap_arr3(p, end, s.igv, 2); else p = ap_null(p, end);
  p = ap_key(p, end, "img");
  if (s.img_v) p = ap_f(p, end, s.img_deg, 3); else p = ap_null(p, end);
  p = ap_key(p, end, "ibr");
  if (s.ibr_v) p = ap_f(p, end, s.ibr, 2); else p = ap_null(p, end);

  p = ap_key(p, end, "ngp");
  if (s.ngp_v) p = ap_f(p, end, s.ngp, 2); else p = ap_null(p, end);
  p = ap_key(p, end, "ngv");
  if (s.ngv_v) p = ap_f(p, end, s.ngv, 2); else p = ap_null(p, end);
  p = ap_key(p, end, "nmg");
  if (s.nmg_v) p = ap_f(p, end, s.nmg, 2); else p = ap_null(p, end);
  p = ap_key(p, end, "nbr");
  if (s.nbr_v) p = ap_f(p, end, s.nbr, 2); else p = ap_null(p, end);

  p = ap_key(p, end, "kgp");
  if (s.kgp_v) p = ap_bool01(p, end, s.kgp); else p = ap_null(p, end);
  p = ap_key(p, end, "kgv");
  if (s.kgv_v) p = ap_bool01(p, end, s.kgv); else p = ap_null(p, end);
  p = ap_key(p, end, "kmg");
  if (s.kmg_v) p = ap_bool01(p, end, s.kmg); else p = ap_null(p, end);
  p = ap_key(p, end, "kbr");
  if (s.kbr_v) p = ap_bool01(p, end, s.kbr); else p = ap_null(p, end);

  p = ap_key(p, end, "himu"); p = ap_i32(p, end, s.himu);
  p = ap_key(p, end, "hmag"); p = ap_i32(p, end, s.hmag);
  p = ap_key(p, end, "hbar"); p = ap_i32(p, end, s.hbar);
  p = ap_key(p, end, "hgps"); p = ap_i32(p, end, s.hgps);

  p = ap_key(p, end, "dimu"); p = ap_u64(p, end, s.dimu);
  p = ap_key(p, end, "dmag"); p = ap_u64(p, end, s.dmag);
  p = ap_key(p, end, "dbar"); p = ap_u64(p, end, s.dbar);
  p = ap_key(p, end, "dgps"); p = ap_u64(p, end, s.dgps);
  p = ap_key(p, end, "dtx"); p = ap_u64(p, end, s.dtx);

  p = ap_key(p, end, "ei2c"); p = ap_u64(p, end, s.ei2c);
  p = ap_key(p, end, "ri2c"); p = ap_u64(p, end, s.ri2c);

  p = ap_key(p, end, "grz");
  if (s.grz_v) p = ap_f(p, end, s.grz, 1); else p = ap_null(p, end);
  p = ap_key(p, end, "fhz"); p = ap_f(p, end, s.fhz, 1);
  p = ap_key(p, end, "lmx"); p = ap_u64(p, end, s.lmx);

  p = ap_key(p, end, "lre"); p = ap_bool01(p, end, s.lre);
  p = ap_key(p, end, "rssi");
  if (s.rssi_v) p = ap_f(p, end, s.rssi_dbm, 0); else p = ap_raw(p, end, "null");
  p = ap_key(p, end, "snr");
  if (s.rssi_v) p = ap_f(p, end, s.snr_db, 1); else p = ap_raw(p, end, "null");
  p = ap_key(p, end, "ltx"); p = ap_u64(p, end, s.ltx);
  p = ap_key(p, end, "lrx"); p = ap_u64(p, end, s.lrx);
  p = ap_key(p, end, "lcrc"); p = ap_u64(p, end, s.lcrc);
  p = ap_key(p, end, "cmode"); p = ap_u64(p, end, s.cmode);
  p = ap_key(p, end, "cdef");
  p = ap_raw(p, end, "[");
  for (int i = 0; i < 4; ++i) {
    if (i) p = ap_raw(p, end, ",");
    p = ap_f(p, end, s.cdef[i], 2);
  }
  p = ap_raw(p, end, "]");

  finishLine(p, end);
}

}  // namespace telem
