// Host tests for the FRAM black-box logger over a 2048-byte RAM store. Build:
//   zig c++ -std=c++17 -O2 -I../firmware/RocketNav -o framlog_test.exe \
//     framlog_test.cpp ../firmware/RocketNav/framlog.cpp
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "framlog.h"

static int checks = 0, fails = 0;
#define CHECK(cond, ...)                    \
  do {                                      \
    checks++;                               \
    if (cond) {                             \
      printf("  ok  : ");                   \
    } else {                                \
      fails++;                              \
      printf("  FAIL: ");                   \
    }                                       \
    printf(__VA_ARGS__);                    \
    printf("\n");                           \
  } while (0)

// RAM store that records the largest single write and can fail on demand.
struct Ram {
  uint8_t mem[fl::kSize];
  int writes = 0, reads = 0;
  uint16_t max_write = 0;
  bool fail_writes = false;
};
static bool ramRead(void *ctx, uint16_t a, uint8_t *b, uint16_t n) {
  Ram *r = (Ram *)ctx;
  if (a + n > fl::kSize) return false;
  memcpy(b, r->mem + a, n);
  r->reads++;
  return true;
}
static bool ramWrite(void *ctx, uint16_t a, const uint8_t *b, uint16_t n) {
  Ram *r = (Ram *)ctx;
  if (r->fail_writes) return false;
  if (a + n > fl::kSize) return false;
  memcpy(r->mem + a, b, n);
  r->writes++;
  if (n > r->max_write) r->max_write = n;
  return true;
}
static void drain(fl::FramLog &L) { int guard = 0; while (L.service() && guard++ < 10000) {} }

int main() {
  printf("== layout + codecs ==\n");
  {
    CHECK(fl::kScratchAddr + fl::kScratchLen == 2048, "layout ends exactly at 2048");
    fl::Event e = { 123456u, 2, 3, fl::EV_SAFE, -7, 987654321, 200 }, g;
    uint8_t buf[fl::kEvtLen];
    fl::packEvent(e, buf);
    CHECK(fl::unpackEvent(buf, &g) && g.t_ms == e.t_ms && g.flight == 2 && g.phase == 3 &&
              g.code == fl::EV_SAFE && g.arg == -7 && g.aux == 987654321 && g.seq == 200,
          "event round trip");
    buf[9] ^= 0x01;
    CHECK(!fl::unpackEvent(buf, &g), "torn event rejected by crc");
    uint8_t z[fl::kEvtLen];
    memset(z, 0, sizeof(z));
    z[fl::kEvtLen - 1] = fl::crc8(z, fl::kEvtLen - 1);
    CHECK(!fl::unpackEvent(z, &g), "blank event (code 0) is not an event");

    fl::Traj t, u;
    memset(&t, 0, sizeof(t));
    t.t_ms = 5000; t.flight = 1; t.phase = 2; t.ral_m = 412.4f; t.vd_mps = -61.26f;
    t.spd_mps = 63.15f; t.eul_deg[0] = -179.0f; t.eul_deg[1] = 88.0f; t.eul_deg[2] = 123.0f;
    t.amag_g = 8.34f; t.rollrate_dps = -720.4f; t.dn_m = -1234.6f; t.de_m = 321.4f; t.health = 0x0F;
    fl::packTraj(t, buf);
    uint8_t tb[fl::kTrajLen];
    fl::packTraj(t, tb);
    CHECK(fl::unpackTraj(tb, &u) && u.t_ms == 5000 && u.phase == 2 && fabsf(u.ral_m - 412.5f) < 1e-3f &&
              fabsf(u.vd_mps + 61.3f) < 0.06f && fabsf(u.spd_mps - 63.2f) < 0.06f &&
              fabsf(u.eul_deg[0] + 180.0f) < 1.01f && fabsf(u.eul_deg[1] - 88.0f) < 1.01f &&
              fabsf(u.amag_g - 8.3f) < 0.06f && fabsf(u.rollrate_dps + 720.0f) < 0.6f &&
              fabsf(u.dn_m + 1235.0f) < 0.6f && u.health == 0x0F,
          "trajectory round trip within LSBs (ral %.1f vd %.2f eul0 %.0f)", u.ral_m, u.vd_mps, u.eul_deg[0]);

    fl::Summary s, r;
    memset(&s, 0, sizeof(s));
    s.valid = true; s.flight = 3; s.launch_ms = 100000; s.apogee_m = 812.4f; s.apogee_ms = 112300;
    s.max_a_g = 12.7f; s.max_spd_mps = 210.3f; s.max_rollrate_dps = -900; s.max_tilt_deg = 71;
    s.max_cdef_deg = -9.75f; s.safe_reason = 3; s.safe_ms = 113000; s.land_ms = 170500;
    s.lat_deg = 40.1164017; s.lon_deg = -88.2434000; s.alt_msl_m = 222.6f; s.dur_ms = 70500;
    s.loss_pct = 4; s.min_urssi = -101; s.boot_count = 17;
    uint8_t sb[fl::kSumLen];
    fl::packSummary(s, sb);
    CHECK(fl::unpackSummary(sb, &r) && r.valid && r.flight == 3 && r.launch_ms == 100000 &&
              fabsf(r.apogee_m - 812.5f) < 1e-3f && r.apogee_ms == 112300 && fabsf(r.max_a_g - 12.7f) < 0.06f &&
              fabsf(r.max_spd_mps - 210.3f) < 0.06f && r.max_rollrate_dps == -900 && r.max_tilt_deg == 71 &&
              fabsf(r.max_cdef_deg + 9.75f) < 1e-3f && r.safe_reason == 3 && r.land_ms == 170500 &&
              fabs(r.lat_deg - s.lat_deg) < 1e-6 && fabs(r.lon_deg - s.lon_deg) < 1e-6 &&
              r.alt_msl_m == 223.0f && r.dur_ms == 70500 && r.loss_pct == 4 && r.min_urssi == -101 && r.boot_count == 17,
          "summary round trip");
  }

  printf("== opt-in recording ==\n");
  Ram ram;
  memset(ram.mem, 0xFF, sizeof(ram.mem));  // fresh chip: garbage
  fl::Store st = { ramRead, ramWrite, &ram };
  fl::FramLog L;
  L.begin(st);
  CHECK(!L.recording(), "garbage header -> not recording");
  L.onBoot(1000, 0x02);
  L.event(1200, 0, fl::EV_ALIGN_DONE, 0, 27500);
  L.heartbeat(2000, 0);
  drain(L);
  CHECK(ram.writes == 0, "nothing written while not recording (%d writes)", ram.writes);

  L.start(3000, 0);
  drain(L);
  CHECK(L.recording() && L.eventCount() == 1, "start writes the REC_START event (%u)", L.eventCount());
  CHECK(ram.max_write <= fl::kChunk, "no single write above %u bytes (max %u)", fl::kChunk, ram.max_write);
  {
    fl::FramLog M;  // a fresh instance over the same bytes: the header persisted
    M.begin(st);
    CHECK(M.recording() && M.eventCount() == 1 && M.header().seq == 1, "header survives a re-read");
  }

  printf("== boot across a reset while recording ==\n");
  L.heartbeat(45000, 2);   // ACTIVE, alive at t=45 s
  drain(L);
  {
    fl::FramLog M;
    M.begin(st);
    M.onBoot(500, 0x04 | 0x01);  // BOR + PIN
    drain(M);
    fl::Event e;
    CHECK(M.eventCount() == 2 && M.readEvent(1, &e) && e.code == fl::EV_BOOT &&
              (e.arg & 0xFF) == 0x05 && ((e.arg >> 8) & 0xFF) == 2 && e.aux == 45000,
          "BOOT event carries cause 0x05, last phase ACTIVE, heartbeat 45 s");
    CHECK(M.header().boot_count == 1, "boot count incremented");
    L.begin(st);  // resync the main instance
  }

  printf("== event ring wraps oldest-first ==\n");
  for (int i = 0; i < 40; ++i) {
    L.event(10000 + i, 1, fl::EV_MARK, (int16_t)i, 0);
    if (i % 5 == 4) drain(L);  // bursts of five: well inside the queue
  }
  drain(L);
  CHECK(L.dropped() == 0, "no queue drops in bursts of five");
  CHECK(L.eventCount() == fl::kEvtN, "ring holds %u", fl::kEvtN);
  {
    fl::Event e0, eL;
    bool ok = L.readEvent(0, &e0) && L.readEvent((uint8_t)(fl::kEvtN - 1), &eL);
    CHECK(ok && e0.code == fl::EV_MARK && e0.arg == 8 && eL.arg == 39, "oldest = mark 8, newest = mark 39 (%d %d)", e0.arg, eL.arg);
  }

  printf("== trajectory: launch segment then rolling ring ==\n");
  L.newFlight();
  for (int i = 0; i < 60; ++i) {
    fl::Traj t;
    memset(&t, 0, sizeof(t));
    t.t_ms = 20000 + i * 500;
    t.phase = 2;
    t.ral_m = (float)i;
    L.traj(t);
    drain(L);
  }
  CHECK(L.trajCount() == fl::kTrajLaunchN + fl::kTrajRollN, "count %u", L.trajCount());
  {
    fl::Traj a, b, c;
    bool ok = L.readTraj(0, &a) && L.readTraj(fl::kTrajLaunchN - 1, &b) && L.readTraj((uint8_t)(L.trajCount() - 1), &c);
    CHECK(ok && a.ral_m == 0 && b.ral_m == 20 && c.ral_m == 59 && a.flight == 1,
          "launch segment 0..20 kept, ring ends at the newest (%.0f %.0f %.0f)", a.ral_m, b.ral_m, c.ral_m);
    fl::Traj d;
    CHECK(L.readTraj(fl::kTrajLaunchN, &d) && d.ral_m == 36, "ring oldest is record 36 (%.0f)", d.ral_m);
  }
  L.newFlight();
  {
    fl::Traj t;
    memset(&t, 0, sizeof(t));
    t.t_ms = 90000; t.ral_m = 100;
    L.traj(t);
    drain(L);
    fl::Traj a;
    CHECK(L.header().tl_n == 1 && L.readTraj(0, &a) && a.ral_m == 100 && a.flight == 2,
          "a new flight restarts the launch segment, flight 2");
  }

  printf("== summary slots ==\n");
  {
    fl::Summary s;
    memset(&s, 0, sizeof(s));
    s.flight = 1; s.apogee_m = 300;
    L.summary(s);
    s.flight = 2; s.apogee_m = 500;
    L.summary(s);
    drain(L);
    fl::FramLog M;
    M.begin(st);
    fl::Summary a, b;
    CHECK(M.readSummary(0, &a) && a.flight == 2 && a.apogee_m == 500 &&
              M.readSummary(1, &b) && b.flight == 1 && b.apogee_m == 300,
          "slot 0 latest, slot 1 previous, both persisted");
  }

  printf("== torn record + stop + wipe ==\n");
  {
    // corrupt the newest event on the medium: the reader must skip it
    fl::Event e;
    CHECK(L.readEvent((uint8_t)(L.eventCount() - 1), &e), "newest event readable");
    uint8_t oldest = (uint8_t)((L.header().evt_head + fl::kEvtN - L.header().evt_n) % fl::kEvtN);
    uint8_t idx = (uint8_t)((oldest + L.eventCount() - 1) % fl::kEvtN);
    ram.mem[fl::kEvtAddr + idx * fl::kEvtLen + 5] ^= 0xFF;
    CHECK(!L.readEvent((uint8_t)(L.eventCount() - 1), &e), "torn event skipped");
    L.stop(99000, 0);
    drain(L);
    CHECK(!L.recording(), "stopped");
    L.event(99500, 0, fl::EV_MARK, 1, 0);
    int w = ram.writes;
    drain(L);
    CHECK(ram.writes == w, "no writes after stop");
    L.start(100000, 0);
    drain(L);
    fl::Summary a;
    CHECK(L.eventCount() == 1 && L.trajCount() == 0 && !L.readSummary(0, &a) && L.header().flight == 0,
          "start wipes counters and summaries");
  }

  printf("== write failure is counted, never blocks ==\n");
  {
    ram.fail_writes = true;
    L.event(101000, 0, fl::EV_MARK, 2, 0);
    drain(L);
    CHECK(L.writeErrors() > 0 && L.idle(), "errors counted (%u), queue drained", L.writeErrors());
    ram.fail_writes = false;
    CHECK(L.scratchTest(), "scratch test passes on a working store");
  }

  printf("\n%d checks, %d failed -> %s\n", checks, fails, fails ? "FAIL" : "PASS");
  return fails ? 1 : 0;
}
