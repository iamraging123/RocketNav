// Host tests for the roll controller + phase machine. Build:
//   zig c++ -std=c++17 -O2 -I../firmware/RocketNav -o control_test.exe \
//     control_test.cpp ../firmware/RocketNav/control.cpp
//
// The plant is a 1-DOF roll model: I_x p_dot = L_delta * u - c_damp * p,
// numbers of canardRocket6DOF magnitude (small 4-canard vehicle at boost
// airspeed). The point is response ENVELOPES and machine logic, not exact
// aero: the 6DOF pass refines gains later against the same test shapes.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "control.h"

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

static const float kDt = 1.0f / 400.0f;

static ctl::Config baseCfg() {
  ctl::Config c;
  c.kp_rate = 0.04f;
  c.ki_rate = 0.5f;  // matched to the test plant below (flight value is
                     // 6DOF-tuned separately; structure is what's tested)
  c.kp_ang = 3.0f;
  c.rate_cmd_max_dps = 180.0f;
  c.defl_max_deg = 10.0f;
  c.slew_dps = 400.0f;
  for (int i = 0; i < 4; ++i) c.mix_sign[i] = 1.0f;
  c.angle_mode = false;
  c.launch_acc_g = 3.0f;
  c.launch_hold_s = 0.1f;
  c.safe_tilt_deg = 60.0f;
  c.safe_time_s = 20.0f;
  c.safe_vd_mps = 3.0f;
  return c;
}

// Boost-phase roll plant. u = mean canard deflection [deg].
struct Plant {
  float p = 0;  // roll rate dps
  float roll = 0;
  void step(float u_deg) {
    const float Ldelta = 400.0f;  // dps^2 per deg - 4-canard authority at
                                  // boost airspeed, canardRocket6DOF scale
    const float cdamp = 2.0f;     // 1/s aero roll damping
    p += (Ldelta * u_deg - cdamp * p) * kDt;
    roll += p * kDt;
  }
};

static void toActive(ctl::Control &c, Plant &pl) {
  c.armRequest();
  for (int k = 0; k < 100; ++k) {  // 0.25 s of boost accel
    c.tick(kDt, pl.p, pl.roll, 5.0f, 8.0f, -20.0f, true);
  }
}

int main() {
  printf("== phase machine ==\n");
  {
    ctl::Control c;
    c.init(baseCfg());
    CHECK(c.mode() == ctl::CM_IDLE, "boots IDLE");
    c.tick(kDt, 0, 0, 2, 0.1f, 0, true);
    CHECK(c.mode() == ctl::CM_IDLE && !c.armRequest() == false, "arms from IDLE");
    CHECK(c.mode() == ctl::CM_ARMED, "ARMED");
    CHECK(!c.armRequest(), "second arm refused");
    // 60 ms of 8 g is shorter than the 100 ms hold: stays ARMED
    for (int k = 0; k < 24; ++k) c.tick(kDt, 0, 0, 2, 8.0f, 0, true);
    for (int k = 0; k < 24; ++k) c.tick(kDt, 0, 0, 2, 0.1f, 0, true);
    CHECK(c.mode() == ctl::CM_ARMED, "60 ms spike does not trigger launch");
    // sustained accel does
    for (int k = 0; k < 60; ++k) c.tick(kDt, 0, 0, 2, 8.0f, 0, true);
    CHECK(c.mode() == ctl::CM_ACTIVE, "sustained 8 g -> ACTIVE");
    // tilt past the cone -> SAFE, outputs slew home
    for (int k = 0; k < 400; ++k) c.tick(kDt, 0, 0, 75.0f, 1.0f, 0, true);
    CHECK(c.mode() == ctl::CM_SAFE, "tilt > 60 deg -> SAFE");
    CHECK(fabsf(c.deflDeg()[0]) < 0.01f, "SAFE deflections centered");
    c.disarm();
    CHECK(c.mode() == ctl::CM_IDLE, "disarm -> IDLE");

    // IMU failure while ARMED -> SAFE
    c.armRequest();
    c.tick(kDt, 0, 0, 2, 0.1f, 0, false);
    CHECK(c.mode() == ctl::CM_SAFE, "IMU fault while armed -> SAFE");

    // descent trips SAFE too
    ctl::Control d;
    d.init(baseCfg());
    Plant pl;
    toActive(d, pl);
    CHECK(d.mode() == ctl::CM_ACTIVE, "helper reaches ACTIVE");
    for (int k = 0; k < 200; ++k) d.tick(kDt, 0, 0, 5, 1.0f, 8.0f, true);
    CHECK(d.mode() == ctl::CM_SAFE, "descending (vd 8 m/s) -> SAFE");

    // timeout trips SAFE
    ctl::Control e;
    e.init(baseCfg());
    Plant pe;
    toActive(e, pe);
    int n21 = (int)(21.0f / kDt);
    for (int k = 0; k < n21; ++k) e.tick(kDt, 0, 0, 5, 1.0f, -5.0f, true);
    CHECK(e.mode() == ctl::CM_SAFE, "20 s timeout -> SAFE");
  }

  printf("== rate damping ==\n");
  {
    ctl::Control c;
    c.init(baseCfg());
    Plant pl;
    pl.p = 200.0f;  // launch with a hard 200 dps spin
    toActive(c, pl);
    float maxdefl = 0;
    for (int k = 0; k < (int)(2.0f / kDt); ++k) {
      c.tick(kDt, pl.p, pl.roll, 5.0f, 4.0f, -30.0f, true);
      pl.step(c.deflDeg()[0]);
      maxdefl = fmaxf(maxdefl, fabsf(c.deflDeg()[0]));
    }
    CHECK(fabsf(pl.p) < 10.0f, "200 dps spin damped to %.1f dps in 2 s",
          pl.p);
    CHECK(maxdefl <= 10.0f + 1e-3f, "deflection never exceeded the clamp");
    // steady state: no residual limit cycle
    float pmax = 0;
    for (int k = 0; k < (int)(1.0f / kDt); ++k) {
      c.tick(kDt, pl.p, pl.roll, 5.0f, 4.0f, -30.0f, true);
      pl.step(c.deflDeg()[0]);
      pmax = fmaxf(pmax, fabsf(pl.p));
    }
    CHECK(pmax < 5.0f, "no limit cycle (max %.2f dps)", pmax);
  }

  printf("== anti-windup under saturation ==\n");
  {
    ctl::Control c;
    c.init(baseCfg());
    Plant pl;
    toActive(c, pl);
    // 3 s of a disturbance the clamp cannot cancel (needs 20 deg, cap 10):
    // the integrator must not wind past the clamp.
    for (int k = 0; k < (int)(3.0f / kDt); ++k) {
      c.tick(kDt, 800.0f, 0, 5.0f, 4.0f, -30.0f, true);
    }
    CHECK(c.saturated(), "reported saturated");
    CHECK(fabsf(c.deflDeg()[0]) <= 10.0f + 1e-3f, "clamped at the limit");
    // disturbance gone: recovery to near-zero output in a bounded time -
    // a wound-up integrator would hold the surface pinned for many seconds
    float t_rec = -1;
    for (int k = 0; k < (int)(2.0f / kDt); ++k) {
      c.tick(kDt, 0.0f, 0, 5.0f, 4.0f, -30.0f, true);
      if (t_rec < 0 && fabsf(c.deflDeg()[0]) < 1.0f) t_rec = k * kDt;
    }
    CHECK(t_rec >= 0 && t_rec < 1.0f, "recovered in %.2f s (windup-free)",
          t_rec);
  }

  printf("== slew limit ==\n");
  {
    ctl::Control c;
    ctl::Config cf = baseCfg();
    cf.slew_dps = 100.0f;
    c.init(cf);
    Plant pl;
    toActive(c, pl);
    float last = c.deflDeg()[0], maxstep = 0;
    for (int k = 0; k < 200; ++k) {
      c.tick(kDt, -400.0f, 0, 5.0f, 4.0f, -30.0f, true);
      maxstep = fmaxf(maxstep, fabsf(c.deflDeg()[0] - last));
      last = c.deflDeg()[0];
    }
    CHECK(maxstep <= 100.0f * kDt + 1e-4f,
          "per-tick step bounded by slew (%.4f deg)", maxstep);
  }

  printf("== angle hold ==\n");
  {
    ctl::Control c;
    ctl::Config cf = baseCfg();
    cf.angle_mode = true;
    c.init(cf);
    Plant pl;
    pl.roll = 30.0f;  // launch roll captured as the target
    toActive(c, pl);
    CHECK(fabsf(c.rollTargetDeg() - 30.0f) < 1e-3f,
          "target captured at ACTIVE entry");
    pl.roll = 80.0f;  // knocked 50 deg off
    for (int k = 0; k < (int)(3.0f / kDt); ++k) {
      c.tick(kDt, pl.p, pl.roll, 5.0f, 4.0f, -30.0f, true);
      pl.step(c.deflDeg()[0]);
    }
    CHECK(fabsf(pl.roll - 30.0f) < 2.0f,
          "roll pulled back to target (%.1f deg)", pl.roll);
    CHECK(fabsf(pl.p) < 5.0f, "settles without spin (%.2f dps)", pl.p);
  }

  printf("== mixer ==\n");
  {
    ctl::Control c;
    ctl::Config cf = baseCfg();
    cf.mix_sign[0] = 1;
    cf.mix_sign[1] = -1;
    cf.mix_sign[2] = 1;
    cf.mix_sign[3] = -1;
    c.init(cf);
    Plant pl;
    toActive(c, pl);
    for (int k = 0; k < 100; ++k) {
      c.tick(kDt, -100.0f, 0, 5.0f, 4.0f, -30.0f, true);
    }
    const float *d = c.deflDeg();
    CHECK(d[0] > 0.5f && d[2] > 0.5f && fabsf(d[0] + d[1]) < 1e-4f &&
              fabsf(d[2] + d[3]) < 1e-4f,
          "mix signs applied per canard");
  }

  printf("== bench roll-hold test ==\n");
  {
    ctl::Control c;
    c.init(baseCfg());  // rate mode
    CHECK(c.rollTestRequest(), "rollTest from IDLE -> BENCH");
    CHECK(c.mode() == ctl::CM_BENCH, "BENCH");
    CHECK(!c.rollTestRequest(), "second rollTest refused");
    CHECK(!c.armRequest(), "arm refused while BENCH");

    // Still airframe: canards sit at neutral (entered with no launch).
    for (int k = 0; k < 50; ++k) c.tick(kDt, 0, 0, 5.0f, 0.5f, 0.0f, true);
    CHECK(fabsf(c.deflDeg()[0]) < 0.1f, "BENCH neutral when still (u=%.3f)",
          c.deflDeg()[0]);

    // Rate mode: an imposed spin deflects the canards to oppose it - the
    // same law as ACTIVE, reached with no launch detect.
    c.tick(kDt, -120.0f, 0, 5.0f, 0.5f, 0.0f, true);
    CHECK(c.deflDeg()[0] > 0.5f, "BENCH fights a -120 dps spin (u=%.2f)",
          c.deflDeg()[0]);

    // The whole point: the flight SAFE triggers are inert on the bench, so
    // it can be tilted/handled past the flight timeout without dropping out.
    bool stayed = true;
    for (int k = 0; k < 9000; ++k) {  // 22.5 s, tilt 80 deg, vd 8 m/s
      c.tick(kDt, 0, 0, 80.0f, 0.5f, 8.0f, true);
      if (c.mode() != ctl::CM_BENCH) {
        stayed = false;
        break;
      }
    }
    CHECK(stayed, "BENCH ignores tilt/descent/timeout");

    // IMU fault still drops it to SAFE (canards slew home).
    c.tick(kDt, 0, 0, 5.0f, 0.5f, 0.0f, false);
    CHECK(c.mode() == ctl::CM_SAFE, "IMU fault in BENCH -> SAFE");
    c.disarm();
    CHECK(c.mode() == ctl::CM_IDLE, "disarm -> IDLE");
  }
  {
    // Angle mode: the roll target is captured on the first BENCH tick.
    ctl::Control c;
    ctl::Config cf = baseCfg();
    cf.angle_mode = true;
    c.init(cf);
    c.rollTestRequest();
    c.tick(kDt, 0, 42.0f, 5.0f, 0.5f, 0.0f, true);
    CHECK(fabsf(c.rollTargetDeg() - 42.0f) < 1e-3f,
          "BENCH captures entry roll (%.2f)", c.rollTargetDeg());
    // Rolled 20 deg off the captured heading -> a correcting command appears.
    c.tick(kDt, 0, 22.0f, 5.0f, 0.5f, 0.0f, true);
    CHECK(fabsf(c.deflDeg()[0]) > 0.1f, "BENCH angle-hold reacts to roll error");
  }

  printf("\n%d checks, %d failed -> %s\n", checks, fails,
         fails ? "FAIL" : "PASS");
  return fails ? 1 : 0;
}
