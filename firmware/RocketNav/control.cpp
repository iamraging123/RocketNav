#include "control.h"

#include <math.h>

namespace ctl {

static float wrap180(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}
static float clampf(float v, float lim) {
  if (v > lim) return lim;
  if (v < -lim) return -lim;
  return v;
}

void Control::init(const Config &c) {
  cfg_ = c;
  mode_ = CM_IDLE;
  u_ = 0;
  integ_ = 0;
  roll_tgt_deg_ = 0;
  launch_t_ = 0;
  active_t_ = 0;
  sat_ = false;
  bench_hold_pending_ = false;
  for (int i = 0; i < 4; ++i) defl_[i] = 0;
}

bool Control::armRequest() {
  if (mode_ != CM_IDLE) return false;
  mode_ = CM_ARMED;
  launch_t_ = 0;
  return true;
}

bool Control::rollTestRequest() {
  if (mode_ != CM_IDLE) return false;
  mode_ = CM_BENCH;
  active_t_ = 0;
  integ_ = 0;
  sat_ = false;
  bench_hold_pending_ = true;  // roll_tgt_deg_ set on the first tick
  return true;
}

void Control::disarm() {
  mode_ = CM_IDLE;
  launch_t_ = 0;
  integ_ = 0;
  sat_ = false;
  bench_hold_pending_ = false;
}

void Control::setGains(float kp_rate, float ki_rate, float kp_ang) {
  cfg_.kp_rate = kp_rate;
  cfg_.ki_rate = ki_rate;
  cfg_.kp_ang = kp_ang;
  integ_ = 0;  // old integral is meaningless under new gains
}

void Control::setAngleMode(bool on) { cfg_.angle_mode = on; }

void Control::enterSafe() {
  mode_ = CM_SAFE;
  integ_ = 0;
  sat_ = false;
}

float Control::controlLaw(float dt, float p_dps, float roll_deg) {
  float rate_tgt = 0.0f;
  if (cfg_.angle_mode) {
    rate_tgt = clampf(cfg_.kp_ang * wrap180(roll_tgt_deg_ - roll_deg),
                      cfg_.rate_cmd_max_dps);
  }
  float err = rate_tgt - p_dps;
  float u_unsat = cfg_.kp_rate * err + integ_;
  float u_sat = clampf(u_unsat, cfg_.defl_max_deg);
  sat_ = (u_sat != u_unsat);
  // Conditional integration: while clamped, only integrate error that
  // pulls back INTO the range - the classic anti-windup for a rocket
  // that spends boost saturated.
  if (!sat_ || (u_unsat > 0 && err < 0) || (u_unsat < 0 && err > 0)) {
    integ_ = clampf(integ_ + cfg_.ki_rate * err * dt, cfg_.defl_max_deg);
  }
  return u_sat;
}

void Control::tick(float dt, float p_dps, float roll_deg, float tilt_deg,
                   float ax_g, float vd_mps, bool imu_ok) {
  if (!(dt > 0)) return;
  if (!imu_ok &&
      (mode_ == CM_ARMED || mode_ == CM_ACTIVE || mode_ == CM_BENCH)) {
    enterSafe();
  }

  float u_target = 0.0f;

  switch (mode_) {
    case CM_ARMED: {
      if (fabsf(ax_g) >= cfg_.launch_acc_g) {
        launch_t_ += dt;
        if (launch_t_ >= cfg_.launch_hold_s) {
          mode_ = CM_ACTIVE;
          active_t_ = 0;
          integ_ = 0;
          roll_tgt_deg_ = roll_deg;  // hold what we launched with
        }
      } else {
        launch_t_ = 0;
      }
      break;
    }
    case CM_ACTIVE: {
      active_t_ += dt;
      if (tilt_deg > cfg_.safe_tilt_deg || vd_mps > cfg_.safe_vd_mps ||
          active_t_ > cfg_.safe_time_s) {
        enterSafe();
        break;
      }
      u_target = controlLaw(dt, p_dps, roll_deg);
      break;
    }
    case CM_BENCH: {
      // Ground roll-hold test: same control law as ACTIVE, but no launch
      // gate on entry and none of the flight SAFE triggers, so the vehicle
      // can be tilted and spun by hand for as long as you like.
      active_t_ += dt;
      if (bench_hold_pending_) {
        roll_tgt_deg_ = roll_deg;  // hold the orientation it started in
        bench_hold_pending_ = false;
      }
      u_target = controlLaw(dt, p_dps, roll_deg);
      break;
    }
    default:
      break;  // IDLE / SAFE: u_target stays 0, surfaces slew home
  }

  float step = cfg_.slew_dps * dt;
  float d = u_target - u_;
  if (d > step) d = step;
  if (d < -step) d = -step;
  u_ += d;

  for (int i = 0; i < 4; ++i) defl_[i] = cfg_.mix_sign[i] * u_;
}

}  // namespace ctl
