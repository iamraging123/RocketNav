// magfit - least-squares sphere fit for magnetometer hard-iron estimation.
// Pure math, no hardware dependencies: host tests exercise it directly.
//
// Model: every clean sample lies on a sphere |m - c|^2 = r^2 whose center c
// IS the hard-iron offset. Linearized (2 c . m + (r^2 - |c|^2) = |m|^2) this
// is a 4-parameter linear least-squares problem accumulated sample by
// sample - no sample storage, O(1) memory.
//
// Why not min/max midpoints: one glitched sample corrupts a midpoint
// permanently, and any axis the sweep never tumbles through the field keeps
// min ~= max there, so the EARTH field itself (47.8 uT vertical here) leaks
// into that axis's "offset". The fit averages noise across every sample,
// and thin coverage is detected per axis (spread) instead of silently
// poisoning the result.
#pragma once

#include <stdint.h>

namespace magfit {

struct SphereFit {
  void reset();

  // Feed one body-frame sample (uT). Samples with |m| outside [5, 200] uT
  // (bus glitches, saturation) are ignored.
  void add(const float m_ut[3]);

  // Least-squares center. Returns false when there is too little data or
  // the normal equations are numerically degenerate; c_ut is untouched
  // then. spread_ut always reports the per-axis swing (max - min) seen -
  // the caller's coverage yardstick: an axis that never swung through the
  // field cannot separate iron from Earth field and must not be trusted.
  bool solve(float c_ut[3], float spread_ut[3]) const;

  uint32_t count() const { return n_; }

  // Per-axis swing seen so far - live coverage feedback mid-sweep.
  void spreads(float spread_ut[3]) const;

 private:
  // Normal equations in double: |m|^2-scale products overflow float32
  // resolution over a 30 s sweep. Samples are shifted by the first sample
  // (m0_) for conditioning; the center is unshifted in solve().
  double ata_[4][4];
  double aty_[4];
  float m0_[3];
  float mn_[3], mx_[3];
  uint32_t n_;
  bool have_m0_;
};

}  // namespace magfit
