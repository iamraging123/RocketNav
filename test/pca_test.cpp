// Host tests for the PCA9685 driver + servo mapping layer: every channel's
// register bytes verified against the datasheet layout through a recording
// I2C mock. Build:
//   zig c++ -std=c++17 -O2 -Imock -I../firmware/RocketNav -o pca_test.exe \
//     pca_test.cpp ../firmware/RocketNav/pca9685.cpp \
//     ../firmware/RocketNav/servos.cpp
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "linkage.h"
#include "servos.h"  // pulls pca9685.h, mock Arduino.h/Wire.h

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

// Expected OFF counts for a pulse width, from the datasheet math at the
// ACHIEVED frame: the integer prescale (121 for a 50 Hz ask, 30 for 200 Hz)
// makes one count last (PRE+1)/25 us, so counts = us * 25 / (PRE+1). The
// driver stores and uses the achieved frame; expectations computed with the
// requested frame drift by 1 count at 50 Hz and ~2% at 200 Hz.
static int countsAt(float us, int pre) {
  return (int)(us * 25.0f / (float)(pre + 1) + 0.5f);
}

// Find the last transaction that starts at the given register.
static const TwoWire::Txn *lastAt(const TwoWire &w, uint8_t reg) {
  for (int i = (int)w.txns.size() - 1; i >= 0; --i) {
    if (!w.txns[i].bytes.empty() && w.txns[i].bytes[0] == reg)
      return &w.txns[i];
  }
  return nullptr;
}
static float clampf_us(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
static int countAt(const TwoWire &w, uint8_t reg) {
  int n = 0;
  for (const auto &t : w.txns)
    if (!t.bytes.empty() && t.bytes[0] == reg) n++;
  return n;
}

int main() {
  printf("== driver: init sequence ==\n");
  {
    TwoWire w;
    Pca9685 pca;
    CHECK(pca.begin(&w, 0x40, 50.0f) && pca.present(), "begin at 0x40");
    const TwoWire::Txn *pre = lastAt(w, 0xFE);
    CHECK(pre != nullptr && pre->bytes.size() == 2 && pre->bytes[1] == 121,
          "prescale 121 for a 50 Hz frame (datasheet value)");
    // sleep -> prescale -> wake -> restart order
    int i_sleep = -1, i_pre = -1, i_wake = -1, i_rst = -1;
    for (int i = 0; i < (int)w.txns.size(); ++i) {
      const auto &b = w.txns[i].bytes;
      if (b.size() == 2 && b[0] == 0x00 && b[1] == 0x30) i_sleep = i;
      if (b.size() == 2 && b[0] == 0xFE) i_pre = i;
      if (b.size() == 2 && b[0] == 0x00 && b[1] == 0x20) i_wake = i;
      if (b.size() == 2 && b[0] == 0x00 && b[1] == 0xA0) i_rst = i;
    }
    CHECK(i_sleep >= 0 && i_sleep < i_pre && i_pre < i_wake && i_wake < i_rst,
          "sleep -> prescale -> wake -> restart order");
  }

  printf("== driver: every channel's register block ==\n");
  {
    TwoWire w;
    Pca9685 pca;
    pca.begin(&w, 0x40, 50.0f);
    w.txns.clear();
    bool reg_ok = true, val_ok = true, addr_ok = true;
    for (int ch = 0; ch < 16; ++ch) {
      float us = 1000.0f + 50.0f * (float)ch;  // distinct width per channel
      pca.setPwmUs((uint8_t)ch, us);
      const auto &t = w.txns.back();
      if (t.addr != 0x40) addr_ok = false;
      if (t.bytes.size() != 5 || t.bytes[0] != (uint8_t)(0x06 + 4 * ch))
        reg_ok = false;
      uint16_t off = (uint16_t)(t.bytes[3] | (t.bytes[4] << 8));
      uint16_t want = (uint16_t)countsAt(us, 121);
      if (t.bytes[1] != 0 || t.bytes[2] != 0 || off != want) val_ok = false;
    }
    CHECK((int)w.txns.size() == 16, "one transaction per channel");
    CHECK(addr_ok, "all 16 on the 0x40 address");
    CHECK(reg_ok, "register = 0x06 + 4*ch for ch 0..15 (datasheet layout)");
    CHECK(val_ok, "ON=0 and OFF counts exact for every channel");

    w.txns.clear();
    pca.setPwmUs(7, 0);
    const auto &t = w.txns.back();
    CHECK(t.bytes[0] == 0x22 && t.bytes[4] == 0x10 && t.bytes[3] == 0 &&
              t.bytes[1] == 0 && t.bytes[2] == 0,
          "us=0 sets the channel full-off flag");

    w.txns.clear();
    pca.setPwmUs(0, 1e6f);
    CHECK((w.txns.back().bytes[3] | (w.txns.back().bytes[4] << 8)) == 4095,
          "overlong pulse clamps at 4095 counts");

    w.txns.clear();
    CHECK(!pca.setPwmUs(16, 1500.0f) && w.txns.empty(),
          "channel 16 rejected, nothing on the bus");

    pca.allOff();
    CHECK(w.txns.back().bytes[0] == 0xFD && w.txns.back().bytes[1] == 0x10,
          "allOff hits ALL_LED_OFF_H");
  }

  printf("== driver: frame rate + errors ==\n");
  {
    TwoWire w;
    Pca9685 pca;
    pca.begin(&w, 0x40, 50.0f);
    pca.setFrameHz(200.0f);
    const TwoWire::Txn *pre = lastAt(w, 0xFE);
    CHECK(pre != nullptr && pre->bytes[1] == 30, "prescale 30 for 200 Hz");
    w.txns.clear();
    pca.setPwmUs(3, 1500.0f);
    uint16_t off =
        (uint16_t)(w.txns.back().bytes[3] | (w.txns.back().bytes[4] << 8));
    CHECK(off == countsAt(1500.0f, 30),
          "1500 us at the achieved 196.9 Hz frame = 1210 counts");

    w.ack = false;
    CHECK(!pca.setPwmUs(3, 1200.0f) && pca.errors() == 1,
          "NACK returns false and counts an error");
  }

  printf("== servos: mapping, signs, write-on-change ==\n");
  TwoWire w;
  Servos sv;
  {
    ServoConfig c;
    c.addr = 0x40;
    c.frame_hz = 50.0f;
    for (int i = 0; i < 4; ++i) {
      c.ch[i] = (uint8_t)i;
      c.center_us[i] = 1500.0f;
    }
    c.us_per_deg[0] = 10.0f;
    c.us_per_deg[1] = -10.0f;
    c.us_per_deg[2] = 10.0f;
    c.us_per_deg[3] = -10.0f;
    c.min_us = 900.0f;
    c.max_us = 2100.0f;
    c.write_period_us = 20000;
    CHECK(sv.begin(&w, c), "servos begin");
    uint64_t t = 1000000;
    sv.service(t);
    CHECK(countAt(w, 0x06) == 1 && countAt(w, 0x0A) == 1 &&
              countAt(w, 0x0E) == 1 && countAt(w, 0x12) == 1,
          "first service centers exactly ch 0..3");

    w.txns.clear();
    float d[4] = { 5.0f, 5.0f, 5.0f, 5.0f };
    sv.setDeflDeg(d);
    t += 20000;
    sv.service(t);
    auto cntOf = [&](uint8_t reg) {
      const TwoWire::Txn *x = lastAt(w, reg);
      return x ? (int)(x->bytes[3] | (x->bytes[4] << 8)) : -1;
    };
    // 1550 us -> 318 counts, 1450 -> 297, at the achieved 50.03 Hz frame
    CHECK(cntOf(0x06) == countsAt(1550.0f, 121) &&
              cntOf(0x0A) == countsAt(1450.0f, 121) &&
              cntOf(0x0E) == countsAt(1550.0f, 121) &&
              cntOf(0x12) == countsAt(1450.0f, 121),
          "+5 deg maps through the per-fin linkage signs (318/297 counts)");

    w.txns.clear();
    t += 20000;
    sv.service(t);
    CHECK(w.txns.empty(), "unchanged targets = zero bus traffic");

    float d2[4] = { 5.1f, 5.1f, 5.1f, 5.1f };  // 1 us shift, below 1 LSB
    sv.setDeflDeg(d2);
    t += 20000;
    sv.service(t);
    CHECK(w.txns.empty(), "sub-LSB change suppressed");

    float d3[4] = { 100.0f, 100.0f, 100.0f, 100.0f };
    sv.setDeflDeg(d3);
    t += 20000;
    sv.service(t);
    CHECK(cntOf(0x06) == 430, "throw guard clamps at max_us (430 counts)");

    printf("== servos: recovery after direct chip writes ==\n");
    // A bench $ang moved a canard channel BEHIND the bookkeeping. center()
    // must rewrite anyway (this also guards the ARMED entry).
    sv.center();
    t += 20000;
    w.txns.clear();
    sv.service(t);
    CHECK(countAt(w, 0x06) == 1 && countAt(w, 0x0A) == 1 &&
              countAt(w, 0x0E) == 1 && countAt(w, 0x12) == 1,
          "center() force-writes all 4 even when bookkeeping matched");

    sv.testUs(2, 1500.0f);  // equals what bookkeeping believes
    t += 20000;
    w.txns.clear();
    sv.service(t);
    CHECK(countAt(w, 0x0E) == 1, "testUs force-writes its channel too");

    w.txns.clear();
    sv.off();
    CHECK(countAt(w, 0x06) == 1 && countAt(w, 0x0A) == 1 &&
              countAt(w, 0x0E) == 1 && countAt(w, 0x12) == 1 &&
              (int)w.txns.size() == 4 &&
              lastAt(w, 0x06)->bytes[4] == 0x10,
          "off() releases exactly the 4 canard channels, nothing else");
  }

  printf("== servos: channel scan covers all 16 ==\n");
  {
    sv.center();
    uint64_t t = 100000000;
    sv.service(t);
    w.txns.clear();
    sv.scanStart(t);
    bool wiggled[16] = { false };
    bool released[16] = { false };
    int reported_max = -1;
    for (int k = 0; k < (int)(21.0f / 0.005f); ++k) {
      w.txns.clear();
      t += 5000;
      sv.service(t);
      if (sv.scanChannel() > reported_max) reported_max = sv.scanChannel();
      for (const auto &x : w.txns) {
        int ch = (x.bytes[0] - 0x06) / 4;
        if (x.bytes[0] == 0xFD || ch < 0 || ch > 15) continue;
        uint16_t off = (uint16_t)(x.bytes[3] | (x.bytes[4] << 8));
        bool is_off = (x.bytes[4] & 0x10) != 0;
        if (is_off) released[ch] = true;
        else if (off != 0) wiggled[ch] = true;
      }
      // The service call that ends the scan also recenters the canards -
      // stop here so those transactions survive for the check below.
      if (reported_max == 15 && sv.scanChannel() == -1) break;
    }
    bool all_wiggled = true, all_released = true;
    for (int ch = 0; ch < 16; ++ch) {
      if (!wiggled[ch]) all_wiggled = false;
      if (!released[ch]) all_released = false;
    }
    CHECK(all_wiggled, "every channel 0..15 got wiggle pulses");
    CHECK(all_released, "every channel released after its turn");
    CHECK(reported_max == 15 && sv.scanChannel() == -1,
          "scanChannel() walked 0..15 then went idle");
    // scan end recenters the canards with forced writes (same service
    // call that closed the scan - kept in w.txns by the break above)
    bool centered = true;
    for (uint8_t reg : { 0x06, 0x0A, 0x0E, 0x12 }) {
      const TwoWire::Txn *x = lastAt(w, reg);
      if (x == nullptr) { centered = false; continue; }
      uint16_t off = (uint16_t)(x->bytes[3] | (x->bytes[4] << 8));
      if (off != 307) centered = false;  // 1500 us at 50 Hz
    }
    CHECK(centered, "canards recentered (307 counts) after the scan");
  }

  printf("== servos: release latch, mode ownership, frame change ==\n");
  {
    TwoWire wf;
    Servos sf;
    ServoConfig c;
    c.addr = 0x40; c.frame_hz = 50.0f;
    for (int i = 0; i < 4; ++i) {
      c.ch[i] = (uint8_t)i; c.center_us[i] = 1500.0f; c.us_per_deg[i] = 10.0f;
    }
    c.min_us = 900.0f; c.max_us = 2100.0f; c.write_period_us = 20000;
    sf.begin(&wf, c);
    uint64_t t = 1000000;
    sf.service(t);  // first write of all four centers

    wf.txns.clear();
    sf.releaseCanard(2);
    CHECK(lastAt(wf, 0x0E) != nullptr && lastAt(wf, 0x0E)->bytes[4] == 0x10,
          "releaseCanard writes the full-off flag");
    t += 20000;
    wf.txns.clear();
    sf.service(t);
    CHECK(lastAt(wf, 0x0E) == nullptr,
          "released canard is NOT re-energized by the next service tick");
    sf.testUs(2, 1400.0f);
    t += 20000;
    wf.txns.clear();
    sf.service(t);
    CHECK(lastAt(wf, 0x0E) != nullptr &&
              (lastAt(wf, 0x0E)->bytes[4] & 0x10) == 0,
          "testUs on a released fin resumes its pulses");

    sf.scanStart(t);
    t += 7300000;           // wall-clock puts the scan on ch 6
    sf.service(t);
    CHECK(sf.scanChannel() == 6, "scan reached ch 6 (wall-clock indexed)");
    wf.txns.clear();
    sf.center();
    CHECK(sf.scanChannel() == -1 && lastAt(wf, 0x1E) != nullptr &&
              lastAt(wf, 0x1E)->bytes[4] == 0x10,
          "center() cancels the scan and releases the wiggled channel");

    t += 20000;
    sf.service(t);          // push centers
    sf.scanStart(t);
    t += 11000000;          // ch 9
    sf.service(t);
    CHECK(sf.scanChannel() == 9, "second scan reached ch 9");
    wf.txns.clear();
    sf.off();
    CHECK(sf.scanChannel() == -1 && lastAt(wf, 0x2A) != nullptr &&
              lastAt(wf, 0x2A)->bytes[4] == 0x10,
          "off() cancels the scan and releases the wiggled channel");

    // Stalled scan: the wall-clock index jumps from a mid channel straight
    // past 15 (flash-save freeze). The LAST WIGGLED channel must be the one
    // released - a hardcoded 15 left it parked at its wiggle pulse forever.
    sf.center();
    t += 20000;
    sf.service(t);
    sf.scanStart(t);
    t += 7300000;           // ch 6 wiggling
    sf.service(t);
    t += 13000000;          // stall: next tick computes ch 16
    wf.txns.clear();
    sf.service(t);
    CHECK(sf.scanChannel() == -1 && lastAt(wf, 0x1E) != nullptr &&
              lastAt(wf, 0x1E)->bytes[4] == 0x10,
          "stalled scan end releases the last wiggled channel (ch 6)");

    t += 20000;
    sf.service(t);
    wf.txns.clear();
    CHECK(sf.setFrameHz(200.0f, 5000), "setFrameHz 200 succeeds");
    CHECK(lastAt(wf, 0xFD) != nullptr,
          "frame change turns the whole bank off (parked ch 4-15 stale)");
    t += 20000;
    wf.txns.clear();
    sf.service(t);
    CHECK(lastAt(wf, 0x06) != nullptr &&
              (int)(lastAt(wf, 0x06)->bytes[3] |
                    (lastAt(wf, 0x06)->bytes[4] << 8)) ==
                  countsAt(1500.0f, 30),
          "canards rewritten with counts at the ACHIEVED 196.9 Hz frame");

    wf.ack = false;
    CHECK(!sf.setFrameHz(50.0f, 20000), "NACKed frame change returns false");
    wf.ack = true;
    wf.txns.clear();
    sf.pca().setPwmUs(0, 1500.0f);
    CHECK((int)(wf.txns.back().bytes[3] | (wf.txns.back().bytes[4] << 8)) ==
              countsAt(1500.0f, 30),
          "aborted frame change leaves the old conversion basis intact");
  }

  printf("== linkage: default synthesis == \n");
  {
    LinkageCal lc;
    linkage::synthDefault(lc, 1500.0f, 10.0f, 900.0f, 2100.0f);
    bool same = true;
    for (float d = -80.0f; d <= 80.0f; d += 2.5f) {
      float old_us = 1500.0f + d * 10.0f;
      if (old_us < 900.0f) old_us = 900.0f;
      if (old_us > 2100.0f) old_us = 2100.0f;
      if (fabsf(linkage::deflToUs(lc, d) - old_us) > 0.01f) same = false;
    }
    CHECK(same, "2-pt default reproduces the old linear map exactly");
    linkage::synthDefault(lc, 1500.0f, -10.0f, 900.0f, 2100.0f);
    CHECK(fabsf(linkage::deflToUs(lc, 5.0f) - 1450.0f) < 0.01f &&
              lc.deg[0] < lc.deg[1] && lc.us[0] > lc.us[1],
          "negative linkage sign: table sorted by deg, us descending");
    CHECK(fabsf(linkage::endAuthorityDeg(lc) - 60.0f) < 0.01f,
          "default authority = 60 deg (600 us / 10 us-per-deg)");
  }

  printf("== linkage: measured 5-point table ==\n");
  {
    // Asymmetric nonlinear linkage: up-gain != down-gain, offset center.
    LinkageCal lc;
    lc.n = 5;
    float dg[5] = { 10.5f, -12.0f, 0.0f, -6.0f, 5.0f };   // entry order:
    float uu[5] = { 1815.0f, 1120.0f, 1502.0f, 1300.0f, 1680.0f };  // scrambled
    for (int i = 0; i < 5; ++i) { lc.deg[i] = dg[i]; lc.us[i] = uu[i]; }
    CHECK(linkage::finalize(lc), "finalize sorts and accepts");
    CHECK(lc.deg[0] == -12.0f && lc.deg[4] == 10.5f && lc.us[0] == 1120.0f,
          "points sorted by canard angle");
    CHECK(fabsf(linkage::deflToUs(lc, 0.0f) - 1502.0f) < 0.01f,
          "node exact: 0 deg -> 1502 us (the offset center)");
    // midpoint between (-6,1300) and (0,1502): -3 deg -> 1401
    CHECK(fabsf(linkage::deflToUs(lc, -3.0f) - 1401.0f) < 0.01f,
          "interpolates between nodes");
    CHECK(fabsf(linkage::deflToUs(lc, -40.0f) - 1120.0f) < 0.01f &&
              fabsf(linkage::deflToUs(lc, 40.0f) - 1815.0f) < 0.01f,
          "clamped at the mechanical table ends");
    CHECK(fabsf(linkage::endAuthorityDeg(lc) - 10.5f) < 0.01f,
          "authority = smaller end (10.5 deg)");

    LinkageCal bad = lc;
    bad.us[2] = 1119.0f;  // reversal
    CHECK(!linkage::finalize(bad), "non-monotonic pulses rejected");
    LinkageCal dup = lc;
    dup.deg[1] = dup.deg[0] + 0.1f;  // near-duplicate angle
    CHECK(!linkage::finalize(dup), "duplicate angles rejected");
    LinkageCal one;
    one.n = 1;
    CHECK(!linkage::finalize(one), "single point rejected");
    // Range bounds mirror the flash loader's finSane(): a table that
    // finalizes must never be zeroed at reboot.
    LinkageCal rng = lc;
    rng.deg[0] = -95.0f;
    CHECK(!linkage::finalize(rng), "angle past +/-90 rejected at finalize");
    rng = lc;
    rng.us[0] = 480.0f;
    CHECK(!linkage::finalize(rng), "pulse outside (500,2500) us rejected");

    // One-point offset save with an off-center neutral: 1200 us at 10 us/deg
    // spans the 2100 us limit at exactly 90 deg, which the loader bound
    // rejects - every such $lcal save failed as "table invalid". The end
    // must be pulled in along the same line, keeping (d0,u0) and the gain.
    for (float g : { 10.0f, -10.0f }) {
      LinkageCal off;
      linkage::synthOffset(off, 0.0f, 1200.0f, g, 900.0f, 2100.0f);
      bool fin_ok = linkage::finalize(off);
      CHECK(fin_ok, "offset table at 1200 us (gain %+.0f) finalizes", g);
      CHECK(fabsf(linkage::deflToUs(off, 0.0f) - 1200.0f) < 0.01f &&
                fabsf(linkage::deflToUs(off, 5.0f) - (1200.0f + 5.0f * g)) <
                    0.01f,
            "offset table keeps the measured neutral and the default gain");
      CHECK(off.deg[0] > -90.0f && off.deg[1] < 90.0f,
            "offset table ends inside the loader's +/-90 deg bound");
    }
  }

  printf("== linkage: applied through the servo layer ==\n");
  {
    // Same mock-driven servos instance from above: install a measured
    // table on fin 0 and verify the actual I2C pulse follows it.
    LinkageCal in[4];
    for (int i = 0; i < 4; ++i) in[i].n = 0;  // others synthesize
    in[0].n = 3;
    in[0].deg[0] = -10.0f; in[0].us[0] = 1200.0f;
    in[0].deg[1] = 1.0f;   in[0].us[1] = 1520.0f;   // offset neutral
    in[0].deg[2] = 12.0f;  in[0].us[2] = 1900.0f;
    sv.setLinkage(in);
    CHECK(fabsf(sv.deflToUs(0, 1.0f) - 1520.0f) < 0.01f &&
              fabsf(sv.deflToUs(1, 5.0f) - 1450.0f) < 0.01f,
          "fin 0 uses the table, fin 1 keeps the synthesized default");
    CHECK(fabsf(sv.authorityDeg() - 10.0f) < 0.01f,
          "authority = min over fins (10 deg from fin 0)");

    uint64_t t = 200000000;
    sv.center();
    sv.service(t);
    w.txns.clear();
    float d[4] = { 6.5f, 0.0f, 0.0f, 0.0f };
    sv.setDeflDeg(d);
    t += 20000;
    sv.service(t);
    // 6.5 deg on fin 0: between (1,1520) and (12,1900) -> 1710 us
    // -> 1710 * 50 * 4096 / 1e6 = 350.2 -> 350 counts
    const TwoWire::Txn *x = lastAt(w, 0x06);
    CHECK(x != nullptr &&
              (int)(x->bytes[3] | (x->bytes[4] << 8)) == 350,
          "commanded 6.5 deg canard becomes 1710 us on the wire");
    // center now means canard-0 through the table: fin 0 -> ~1491 us
    w.txns.clear();
    sv.center();
    t += 20000;
    sv.service(t);
    x = lastAt(w, 0x06);
    int cnt = x ? (int)(x->bytes[3] | (x->bytes[4] << 8)) : -1;
    // 0 deg between (-10,1200) and (1,1520): 1200 + (10/11)*320 = 1490.9 us
    CHECK(cnt == countsAt(1490.909f, 121),
          "center() drives the calibrated canard-zero (306 counts)");
  }

  printf("== startup: every fin driven to its calibrated 0 deg ==\n");
  {
    // Mirror the boot sequence: begin -> setLinkage(stored) -> center(),
    // then the first service tick. Each fin has a DIFFERENT calibrated
    // neutral; the first pulse written must be that neutral, not 1500 us.
    TwoWire wb;
    Servos sb;
    ServoConfig c;
    c.addr = 0x40; c.frame_hz = 50.0f;
    for (int i = 0; i < 4; ++i) {
      c.ch[i] = (uint8_t)i; c.center_us[i] = 1500.0f; c.us_per_deg[i] = 10.0f;
    }
    c.min_us = 900.0f; c.max_us = 2100.0f; c.write_period_us = 20000;
    sb.begin(&wb, c);
    LinkageCal lc[4];  // distinct neutrals, all != 1500
    const float neu[4] = { 1435.0f, 1560.0f, 1478.0f, 1522.0f };
    for (int i = 0; i < 4; ++i) {
      lc[i].n = 2;
      lc[i].deg[0] = -12.0f; lc[i].us[0] = neu[i] - 120.0f;
      lc[i].deg[1] = 12.0f;  lc[i].us[1] = neu[i] + 120.0f;
    }
    sb.setLinkage(lc);
    sb.center();               // the boot call
    wb.txns.clear();
    sb.service(1000000);       // first tick after boot
    bool ok = true;
    for (int i = 0; i < 4; ++i) {
      const TwoWire::Txn *x = lastAt(wb, (uint8_t)(0x06 + 4 * i));
      if (!x) { ok = false; continue; }
      int cnt = (int)(x->bytes[3] | (x->bytes[4] << 8));
      int want = countsAt(neu[i], 121);
      if (cnt != want) ok = false;
    }
    CHECK(ok, "boot writes each fin's calibrated neutral (not 1500 us)");

    // Test-all-fins: +10 -> -10 -> 0, each step through the linkage table,
    // ending at neutral. Sample one time point inside each phase.
    auto cntAll = [&](uint8_t base) {
      const TwoWire::Txn *x = lastAt(wb, base);
      return x ? (int)(x->bytes[3] | (x->bytes[4] << 8)) : -1;
    };
    sb.canardTestStart(2000000);
    // Sample one point INSIDE each phase (phase clock is relative to start:
    // 0-1.3 s = +10, 1.3-2.6 = -10, 2.6-3.9 = 0).
    struct { float t; float deg; } steps[3] = {
      { 0.5f, 10.0f }, { 1.8f, -10.0f }, { 3.0f, 0.0f } };
    bool seq_ok = true;
    for (int s = 0; s < 3; ++s) {
      wb.txns.clear();
      sb.service(2000000 + (uint64_t)(steps[s].t * 1e6f));
      for (int i = 0; i < 4; ++i) {
        int want = countsAt(clampf_us(linkage::deflToUs(lc[i], steps[s].deg),
                                      900.0f, 2100.0f), 121);
        if (cntAll((uint8_t)(0x06 + 4 * i)) != want) seq_ok = false;
      }
    }
    CHECK(seq_ok, "test sweep drives all 4 fins +10/-10/0 via the tables");
    // After the sequence window it ends at neutral.
    wb.txns.clear();
    sb.service(2000000 + (uint64_t)(4.2f * 1e6f));
    CHECK(sb.canardTestPhase() == -1, "test ends idle (settled at neutral)");
  }

  printf("\n%d checks, %d failed -> %s\n", checks, fails,
         fails ? "FAIL" : "PASS");
  return fails ? 1 : 0;
}
