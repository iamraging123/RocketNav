// control - roll-axis canard controller with its own minimal arming/phase
// machine. Pure math, no hardware, no Arduino: host tests drive it directly.
//
// The navigation filter stays phase-free; ALL flight-phase logic lives here,
// consumes filter outputs, and feeds nothing back into estimation.
//
// Structure: rate loop (PI on roll-rate error, gyro feedback) optionally
// cascaded under an angle loop (P on roll error -> rate command). Rate-only
// mode needs no magnetometer and is the first-flight configuration.
//
// Phases:  IDLE -(armRequest)-> ARMED -(launch detect)-> ACTIVE -(tilt /
// descent / timeout)-> SAFE.  disarm() from anywhere -> IDLE.  An unhealthy
// IMU while ARMED/ACTIVE/BENCH -> SAFE.  Outputs are centered (slew-limited
// to 0) in every mode except ACTIVE and BENCH.
//
// BENCH is a ground roll-hold test: IDLE -(rollTestRequest)-> BENCH runs the
// exact same control law as ACTIVE but is entered directly (no arm, no launch
// detect) and skips the flight-only SAFE triggers (tilt / descent / timeout)
// so the rocket can be turned by hand as long as you like. An unhealthy IMU
// still drops it to SAFE, and disarm() returns it to IDLE.
#pragma once

#include <stdint.h>

namespace ctl {

enum Mode : uint8_t {
  CM_IDLE = 0,
  CM_ARMED = 1,
  CM_ACTIVE = 2,
  CM_SAFE = 3,
  CM_BENCH = 4  // ground roll-hold test; wire value appended, never reordered
};

struct Config {
  // Rate loop: deflection [deg] = kp_rate * rate_err [dps] + integral
  float kp_rate;          // deg per dps
  float ki_rate;          // deg per dps per s (conditional integration)
  // Angle loop (angle_mode only): rate cmd [dps] = kp_ang * roll_err [deg]
  float kp_ang;
  float rate_cmd_max_dps;
  float defl_max_deg;     // hard clamp, per canard, pre-mix magnitude
  float slew_dps;         // deflection slew limit, deg/s
  float mix_sign[4];      // per-canard sign of a positive roll command
  bool angle_mode;        // false = rate damping only (no mag dependence)
  // Launch detect (ARMED -> ACTIVE): |axial accel| above threshold, held
  float launch_acc_g;
  float launch_hold_s;
  // ACTIVE -> SAFE
  float safe_tilt_deg;    // nose angle from vertical
  float safe_time_s;      // since ACTIVE entry
  float safe_vd_mps;      // NED down velocity (positive = descending)
};

class Control {
 public:
  void init(const Config &c);

  // IDLE -> ARMED only; returns false from any other mode.
  bool armRequest();
  // IDLE -> BENCH only (ground roll-hold test, no launch needed); returns
  // false from any other mode. Roll target is captured on the first tick.
  bool rollTestRequest();
  // Any mode -> IDLE, outputs slew to center.
  void disarm();

  void setGains(float kp_rate, float ki_rate, float kp_ang);
  void setAngleMode(bool on);
  // Authority from the calibrated linkage tables: min over fins, applied
  // at boot and after every $lcal save/clear.
  void setDeflMax(float deg) { cfg_.defl_max_deg = deg; }

  // Call at the IMU cadence.
  //   dt        step [s]
  //   p_dps     bias-corrected body roll rate [deg/s]
  //   roll_deg  estimated roll [deg]
  //   tilt_deg  nose angle from vertical [deg]
  //   ax_g      axial specific force [g]
  //   vd_mps    NED down velocity [m/s]
  //   imu_ok    IMU producing fresh valid samples
  void tick(float dt, float p_dps, float roll_deg, float tilt_deg, float ax_g,
            float vd_mps, bool imu_ok);

  Mode mode() const { return mode_; }
  const float *deflDeg() const { return defl_; }  // per canard, post-mix
  float rollTargetDeg() const { return roll_tgt_deg_; }
  bool saturated() const { return sat_; }
  float activeT() const { return active_t_; }
  const Config &config() const { return cfg_; }

 private:
  void enterSafe();
  // Shared rate/angle control law (integrator + anti-windup + saturation).
  // Returns the saturated pre-mix deflection; used by ACTIVE and BENCH.
  float controlLaw(float dt, float p_dps, float roll_deg);

  Config cfg_;
  Mode mode_ = CM_IDLE;
  float defl_[4] = { 0, 0, 0, 0 };
  float u_ = 0;             // slew-limited pre-mix deflection [deg]
  float integ_ = 0;         // rate-loop integral [deg]
  float roll_tgt_deg_ = 0;  // captured at ACTIVE/BENCH entry (angle mode)
  float launch_t_ = 0;
  float active_t_ = 0;
  bool sat_ = false;
  bool bench_hold_pending_ = false;  // capture roll target on first BENCH tick
};

}  // namespace ctl
