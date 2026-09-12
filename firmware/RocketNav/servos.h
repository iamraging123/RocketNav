// servos - canard servo mapping layer over the PCA9685: deflection degrees
// in, pulse widths out. Owns the bench-test overrides ($servo commands) and
// the write-on-change scheduler that keeps servo traffic to one short
// transaction per changed channel at the PWM frame cadence.
#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "linkage.h"
#include "pca9685.h"

struct ServoConfig {
  uint8_t addr;            // PCA9685 I2C address
  float frame_hz;          // PWM frame rate (analog servos: keep 50)
  uint8_t ch[4];           // PCA channel per canard
  float center_us[4];      // linkage-neutral pulse per canard
  float us_per_deg[4];     // pulse per degree of deflection, SIGN = linkage
                           // direction (set after the first bench check)
  float min_us, max_us;    // absolute pulse guard
  uint32_t write_period_us;
};

class Servos {
 public:
  bool begin(TwoWire *wire, const ServoConfig &cfg);

  // Deflections from the controller (TRUE canard degrees, already mixed).
  // Converted to pulses through the per-fin linkage tables.
  void setDeflDeg(const float d[4]);

  // Per-fin linkage tables. Fins with n<2 in `in` (or in==nullptr) get the
  // synthesized default that reproduces the compile-time linear map.
  void setLinkage(const LinkageCal in[4]);
  const LinkageCal *linkage() const { return lcal_; }
  float deflToUs(int fin, float d_deg) const;
  // min symmetric canard authority across the four fins (table ends).
  float authorityDeg() const;
  // Offset-only table for a fin from ONE measured point, using that fin's
  // configured default gain and pulse limits. Backs a single-point $lcal.
  void buildOffsetTable(int fin, float d0_deg, float u0_us,
                        LinkageCal &out) const;

  // Bench overrides - active until cleared by center()/setDeflDeg source.
  // testUs takes ownership: a running sweep/scan/fin-test is cancelled (the
  // scan's wiggled channel released) so the commanded pulse actually lands.
  bool testUs(int canard, float us);   // raw pulse to one canard
  // Stop pulses on one canard (limp) and LATCH it released: service() skips
  // it until testUs on that fin, or center()/setDeflDeg/a bench mode, takes
  // the outputs back. Without the latch the next write tick silently
  // re-energized the fin from its stale target.
  void releaseCanard(int fin);
  void center();                        // all canards to center_us
  void off();                           // pulses stop (servos limp)
  void sweepStart(uint64_t now_us);     // ~6 s triangle sweep, all canards
  // All-fin functional test in CALIBRATED canard degrees: +10 -> -10 -> 0,
  // ~1.3 s per step, ends at neutral. Angles beyond a fin's authority clamp.
  void canardTestStart(uint64_t now_us);
  // -1 idle, else the step now commanded: 0 = +10 deg, 1 = -10 deg, 2 = 0.
  int canardTestPhase() const { return ctest_ ? ctest_phase_ : -1; }

  // Channel finder: wiggles EVERY PCA channel in turn (60->120->90 deg,
  // ~1.2 s each, ~19 s total, previous channel released) so the true
  // header->channel map can be read off the bench by watching the servos.
  void scanStart(uint64_t now_us);
  // -1 when idle, else the channel being wiggled right now - the .ino
  // announces changes as msg records.
  int scanChannel() const { return scan_ ? scan_ch_ : -1; }

  // Write-on-change, one bounded transaction per changed canard.
  void service(uint64_t now_us);

  // Reprogram the PWM frame rate + write cadence at runtime ($sframe): the
  // frame period is the dominant command->pulse latency, and how far it can
  // be raised depends on the servo model - a bench knob, not a reflash.
  // On success ALL 16 channels are turned off (a prescale change rescales
  // every channel's counts, and non-canard channels parked by $ang have no
  // owner to rewrite them - off beats a wildly wrong pulse), then the four
  // canards are force-rewritten at the new frame within one write period.
  // Returns false on bus failure or when the PCA is absent.
  bool setFrameHz(float hz, uint32_t write_period_us);
  float frameHz() const { return pca_.frameHz(); }

  // Re-probe and re-init an absent PCA9685 (boot probe failed or a frame
  // change died mid-sequence). Keeps linkage tables and current targets;
  // on success every canard channel is force-rewritten. Called from the
  // 10 Hz service slot with backoff - the chip gets the same second chance
  // every sensor on this bus already had.
  bool recover();

  // True while a bench mode (sweep / channel scan / fin test) owns the
  // outputs - staged calibration must not read targets as "applied".
  bool benchBusy() const { return sweep_ || scan_ || ctest_; }

  // Debug pass-through for the $ang command: drive ANY of the 16 PCA
  // channels, not just the canard four.
  Pca9685 &pca() { return pca_; }

  bool present() const { return pca_.present(); }
  uint32_t errors() const { return pca_.errors(); }
  const float *targetUs() const { return tgt_us_; }

 private:
  float clampUs(float us) const;
  // Cancel any running bench mode (sweep / scan / fin test), releasing the
  // scan's currently wiggled channel, and clear the whole-bank off latch.
  // Every path that takes control of the outputs goes through here so no
  // two modes ever fight over tgt_us_ / the chip.
  void takeOwnership();

  Pca9685 pca_;
  TwoWire *wire_ = nullptr;
  ServoConfig cfg_;
  LinkageCal lcal_[4];
  float tgt_us_[4] = { 0, 0, 0, 0 };
  float sent_us_[4] = { -1, -1, -1, -1 };
  uint8_t released_ = 0;   // bit i: canard i latched limp by releaseCanard
  bool off_ = false;
  bool sweep_ = false;
  uint64_t sweep_t0_ = 0;
  bool scan_ = false;
  uint64_t scan_t0_ = 0;
  int scan_ch_ = -1;
  bool ctest_ = false;
  uint64_t ctest_t0_ = 0;
  int ctest_phase_ = -1;
  uint64_t next_write_us_ = 0;
};
