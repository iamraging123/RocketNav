#include "servos.h"

float Servos::clampUs(float us) const {
  if (us < cfg_.min_us) return cfg_.min_us;
  if (us > cfg_.max_us) return cfg_.max_us;
  return us;
}

bool Servos::begin(TwoWire *wire, const ServoConfig &cfg) {
  wire_ = wire;
  cfg_ = cfg;
  bool ok = pca_.begin(wire, cfg_.addr, cfg_.frame_hz);
  setLinkage(nullptr);  // defaults until the stored tables are applied
  for (int i = 0; i < 4; ++i) {
    tgt_us_[i] = cfg_.center_us[i];
    sent_us_[i] = -1;  // force the first write
  }
  off_ = false;
  sweep_ = false;
  scan_ = false;
  ctest_ = false;
  released_ = 0;
  next_write_us_ = 0;
  return ok;
}

bool Servos::recover() {
  if (!pca_.begin(wire_, cfg_.addr, cfg_.frame_hz)) return false;
  // The chip may have been alive-but-unreachable and still hold pre-fault
  // channel state: clear the bank, then force-rewrite the canard targets.
  pca_.allOff();
  for (int i = 0; i < 4; ++i) sent_us_[i] = -1;
  next_write_us_ = 0;
  return true;
}

void Servos::takeOwnership() {
  sweep_ = false;
  ctest_ = false;
  ctest_phase_ = -1;
  if (scan_) {
    if (scan_ch_ >= 0) pca_.setPwmUs((uint8_t)scan_ch_, 0);
    scan_ = false;
    scan_ch_ = -1;
  }
  off_ = false;
}

void Servos::setDeflDeg(const float d[4]) {
  // Defensive: a bench mode can no longer coexist with a controlling flight
  // mode (arming centers, which cancels them), but if one is ever active
  // its targets must not be interleaved with control output.
  if (benchBusy()) return;
  off_ = false;
  released_ = 0;  // control owns every canard
  for (int i = 0; i < 4; ++i) {
    tgt_us_[i] = clampUs(linkage::deflToUs(lcal_[i], d[i]));
  }
}

void Servos::setLinkage(const LinkageCal in[4]) {
  for (int i = 0; i < 4; ++i) {
    if (in != nullptr && in[i].n >= 2) {
      lcal_[i] = in[i];
    } else {
      linkage::synthDefault(lcal_[i], cfg_.center_us[i], cfg_.us_per_deg[i],
                            cfg_.min_us, cfg_.max_us);
    }
  }
}

float Servos::deflToUs(int fin, float d_deg) const {
  if (fin < 0 || fin > 3) return 0;
  return linkage::deflToUs(lcal_[fin], d_deg);
}

float Servos::authorityDeg() const {
  float a = 1e9f;
  for (int i = 0; i < 4; ++i) {
    float e = linkage::endAuthorityDeg(lcal_[i]);
    if (e < a) a = e;
  }
  return a;
}

void Servos::buildOffsetTable(int fin, float d0_deg, float u0_us,
                              LinkageCal &out) const {
  if (fin < 0 || fin > 3) { out.n = 0; return; }
  linkage::synthOffset(out, d0_deg, u0_us, cfg_.us_per_deg[fin], cfg_.min_us,
                       cfg_.max_us);
}

bool Servos::testUs(int canard, float us) {
  if (canard < 0 || canard > 3) return false;
  takeOwnership();
  released_ &= (uint8_t)~(1u << canard);  // driving this fin un-releases it
  tgt_us_[canard] = clampUs(us);
  sent_us_[canard] = -1;  // force the write: a direct $ang/pca write may
                          // have moved the chip behind the bookkeeping
  return true;
}

void Servos::releaseCanard(int fin) {
  if (fin < 0 || fin > 3) return;
  pca_.setPwmUs(cfg_.ch[fin], 0);  // full-off: pulses stop, servo limp
  released_ |= (uint8_t)(1u << fin);  // latch: service() must not rewrite
  sent_us_[fin] = -1;              // whoever un-releases starts fresh
}

void Servos::center() {
  takeOwnership();
  released_ = 0;  // center intentionally re-energizes every canard
  for (int i = 0; i < 4; ++i) {
    // Canard-zero through the linkage table, not the raw servo center.
    tgt_us_[i] = clampUs(linkage::deflToUs(lcal_[i], 0.0f));
    sent_us_[i] = -1;  // force the write (see testUs) - this also makes
                       // ARMED start from true chip centers, whatever a
                       // bench $ang left on a canard channel
  }
}

void Servos::off() {
  // Cancel any bench mode FIRST: off_ short-circuits service(), so a scan
  // left latched here would freeze with its wiggled channel still driving.
  takeOwnership();
  off_ = true;
  // Release ONLY the canard channels - $ang users may be driving 4-15.
  for (int i = 0; i < 4; ++i) {
    pca_.setPwmUs(cfg_.ch[i], 0);
    sent_us_[i] = -1;  // resume forces rewrites
  }
}

void Servos::sweepStart(uint64_t now_us) {
  takeOwnership();
  released_ = 0;  // the sweep drives all four
  sweep_ = true;
  sweep_t0_ = now_us;
}

void Servos::canardTestStart(uint64_t now_us) {
  takeOwnership();
  released_ = 0;  // the test drives all four
  ctest_ = true;
  ctest_t0_ = now_us;
  ctest_phase_ = -1;
}

void Servos::scanStart(uint64_t now_us) {
  takeOwnership();
  released_ = 0;  // the scan wiggles (and finally recenters) all four
  scan_ = true;
  scan_t0_ = now_us;
  scan_ch_ = -1;
}

bool Servos::setFrameHz(float hz, uint32_t write_period_us) {
  if (!pca_.present()) return false;
  if (!pca_.setFrameHz(hz)) return false;
  cfg_.frame_hz = pca_.frameHz();
  cfg_.write_period_us = write_period_us;
  // A prescale change rescales the COUNTS of all 16 channels: a channel
  // parked by $ang would keep its old counts and output a wildly wrong
  // pulse at the new frame (307 counts is 1500 us at 50 Hz but ~380 us at
  // 200 Hz — hard against the end stop). No one owns those channels, so
  // turn the whole bank off; the canards are force-rewritten at the new
  // frame within one write period.
  pca_.allOff();
  for (int i = 0; i < 4; ++i) sent_us_[i] = -1;
  next_write_us_ = 0;
  return true;
}

void Servos::service(uint64_t now_us) {
  if (!pca_.present() || off_) return;
  if (now_us < next_write_us_) return;
  next_write_us_ = now_us + cfg_.write_period_us;

  if (scan_) {
    // Channel finder owns ALL outputs while it runs.
    float t = (float)(now_us - scan_t0_) * 1e-6f;
    int ch = (int)(t / 1.2f);
    if (ch > 15) {
      // Release the channel that was ACTUALLY last wiggled: a service stall
      // (flash save, bus recovery) can jump the wall-clock index past 15
      // from any channel, and releasing a hardcoded 15 would leave that
      // one parked at its wiggle pulse forever.
      if (scan_ch_ >= 0) pca_.setPwmUs((uint8_t)scan_ch_, 0);
      scan_ = false;
      scan_ch_ = -1;
      // The canard channels were wiggled and released mid-scan: force a
      // rewrite back to their centers.
      for (int i = 0; i < 4; ++i) {
        tgt_us_[i] = clampUs(linkage::deflToUs(lcal_[i], 0.0f));
        sent_us_[i] = -1;
      }
    } else {
      if (ch != scan_ch_) {
        if (scan_ch_ >= 0) pca_.setPwmUs((uint8_t)scan_ch_, 0);
        scan_ch_ = ch;
      }
      float ph = t - 1.2f * (float)ch;
      float us = (ph < 0.4f) ? 1333.0f : (ph < 0.8f ? 1667.0f : 1500.0f);
      pca_.setPwmUs((uint8_t)scan_ch_, us);
      return;
    }
  }

  if (ctest_) {
    // Functional test in calibrated canard degrees: +10 -> -10 -> 0.
    float t = (float)(now_us - ctest_t0_) * 1e-6f;
    int phase = (t < 1.3f) ? 0 : (t < 2.6f) ? 1 : (t < 3.9f) ? 2 : 3;
    if (phase >= 3) {
      ctest_ = false;
      ctest_phase_ = -1;
      center();  // settle at calibrated neutral, force-written
    } else {
      ctest_phase_ = phase;
      const float deg = (phase == 0) ? 10.0f : (phase == 1) ? -10.0f : 0.0f;
      for (int i = 0; i < 4; ++i) {
        tgt_us_[i] = clampUs(linkage::deflToUs(lcal_[i], deg));
      }
    }
  }

  if (sweep_) {
    // ~6 s: two slow triangles across a modest +/-200 us throw, then home.
    float t = (float)(now_us - sweep_t0_) * 1e-6f;
    if (t >= 6.0f) {
      sweep_ = false;
      center();
    } else {
      float phase = t / 3.0f;                 // one triangle per 3 s
      phase -= (float)(int)phase;             // 0..1
      float tri = (phase < 0.5f) ? (4.0f * phase - 1.0f)
                                 : (3.0f - 4.0f * phase);  // -1..1..-1
      for (int i = 0; i < 4; ++i) {
        tgt_us_[i] =
            clampUs(linkage::deflToUs(lcal_[i], 0.0f) + 200.0f * tri);
      }
    }
  }

  // Skip writes below the chip's own resolution (1 LSB of the 12-bit frame:
  // ~4.9 us at 50 Hz, ~1 us at 250 Hz) so steady flight costs zero bus
  // traffic - derived, so raising the frame rate tightens the deadband too.
  float thr = 0.9f * 1000000.0f / (4096.0f * pca_.frameHz());
  if (thr < 0.5f) thr = 0.5f;
  for (int i = 0; i < 4; ++i) {
    if (released_ & (1u << i)) continue;  // latched limp: never rewrite
    if (sent_us_[i] >= 0 && fabsf(tgt_us_[i] - sent_us_[i]) < thr) continue;
    if (pca_.setPwmUs(cfg_.ch[i], tgt_us_[i])) sent_us_[i] = tgt_us_[i];
  }
}
