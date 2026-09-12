// linkage - the servo->canard mechanical map. Pure math, no hardware, no
// Arduino: servos.cpp applies it, cfgstore persists it, host tests drive it.
//
// The linkage (horn ratio, pushrod geometry) means servo degrees are NOT
// canard degrees: each fin has its own gain, neutral offset, mild
// nonlinearity/asymmetry, and mechanical limits. Everything is captured by
// one monotonic table of up to 5 measured (canard_deg, servo_us) points per
// fin, applied by piecewise-linear interpolation in the deg -> us direction
// and clamped at the table ends (which ARE the bench-found mechanical
// limits).
#pragma once

#include <stdint.h>

struct LinkageCal {
  uint8_t n;      // 0/1 = uncalibrated (defaults synthesized), else 2..5
  float deg[5];   // canard degrees, strictly increasing after finalize()
  float us[5];    // matching servo pulse, strictly monotonic
};

namespace linkage {

// Canard deflection -> servo pulse through the table, clamped at the ends.
float deflToUs(const LinkageCal &lc, float d_deg);

// Uncalibrated fallback: the 2-point table that reproduces the old linear
// map us = clamp(center + d * us_per_deg, min, max) exactly.
void synthDefault(LinkageCal &lc, float center_us, float us_per_deg,
                  float min_us, float max_us);

// Offset-only calibration: a 2-point table with the DEFAULT gain
// (us_per_deg) whose line passes exactly through one measured point
// (d0_deg, u0_us) and spans [min_us, max_us]. This is "keep the gain, fix
// the zero" - the single-point calibration flow. finalize() still applies.
void synthOffset(LinkageCal &lc, float d0_deg, float u0_us, float us_per_deg,
                 float min_us, float max_us);

// Sort staged points by canard angle and validate: 2..5 points, angles
// strictly increasing (>= 0.5 deg apart) and inside +/-90 deg, pulses in
// (500, 2500) us and strictly monotonic in one direction — the range
// bounds mirror the flash loader's finSane() so every table that
// finalizes also survives a reboot. false leaves the table unusable.
bool finalize(LinkageCal &lc);

// How much symmetric deflection this fin can actually deliver:
// min(|first deg|, |last deg|). 0 for an unusable table.
float endAuthorityDeg(const LinkageCal &lc);

}  // namespace linkage
