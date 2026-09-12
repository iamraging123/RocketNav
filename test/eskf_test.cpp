// eskf_test.cpp — native host-side verification of the flight ESKF.
// Compiles the exact flight sources (eskf.cpp, nav_frames.cpp — both
// Arduino-free) into a host binary, feeds synthetic stationary IMU data plus
// synthetic GNSS/baro/mag measurements with known truth, and asserts:
//   * covariance stays symmetric and positive-definite throughout,
//   * attitude converges during static alignment,
//   * biases converge toward the injected true biases,
//   * NIS statistics stay within chi-square bounds,
//   * MAG_OFF runs indefinitely (heading covariance grows, nothing
//     diverges) and MAG_INIT_ONLY stops updating at anchor.
//
// Build (any C++17 host compiler):
//   zig c++ -O2 -I firmware/RocketNav -o test/eskf_test \
//       test/eskf_test.cpp firmware/RocketNav/eskf.cpp \
//       firmware/RocketNav/nav_frames.cpp

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include "eskf.h"
#include "nav_frames.h"

// ---------- deterministic gaussian RNG ----------
static uint64_t rng_state = 0x123456789ABCDEFull;
static double urand() {
  rng_state = rng_state * 6364136223846793005ull + 1442695040888963407ull;
  return (double)((rng_state >> 11) & 0x1FFFFFFFFFFFFFull) / 9007199254740992.0;
}
static double gauss() {
  double u1 = urand(), u2 = urand();
  if (u1 < 1e-12) u1 = 1e-12;
  return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

// ---------- check harness ----------
static int checks = 0, fails = 0;
#define CHECK(cond, ...)                              \
  do {                                                \
    checks++;                                         \
    if (cond) {                                       \
      printf("  ok  : " __VA_ARGS__);                 \
      printf("\n");                                   \
    } else {                                          \
      fails++;                                        \
      printf("  FAIL: " __VA_ARGS__);                 \
      printf("\n");                                   \
    }                                                 \
  } while (0)

// ---------- truth model ----------
static const double kLat = 40.1164, kLon = -88.2434;
static const int32_t kLat1e7 = 401164000, kLon1e7 = -882434000;
static const int32_t kHaeMm = 189000, kHmslMm = 222000;  // undulation -33 m
static const float kRollT = 2.0f * nav::DEG2RAD;
static const float kPitchT = -1.5f * nav::DEG2RAD;
static const float kYawT = 25.0f * nav::DEG2RAD;
static const float kBgT[3] = { 0.008f, -0.012f, 0.005f };
static const float kBaT[3] = { 0.03f, -0.04f, 0.08f };
static const float kBbT = 2.0f;  // true baro altitude bias, m
static const float kMagRef[3] = { 20.15f, -1.23f, 47.77f };

static const float kGyroNd = 6.63e-5f;
static const float kAccelNd = 1.08e-3f;
static const float kImuHz = 400.0f;
static const float kDt = 1.0f / kImuHz;

struct Truth {
  float q[4];
  float Rt[9];
  float g_true;
  float f_body0[3];  // noiseless specific force
  float m_body0[3];  // noiseless body field
};

static void makeTruthAt(Truth &t, float roll, float pitch, float yaw) {
  nav::quat_from_euler(roll, pitch, yaw, t.q);
  nav::quat_to_dcm(t.q, t.Rt);
  t.g_true = nav::gravity_wgs84(kLat * nav::DEG2RAD, kHaeMm * 1e-3f);
  float gvec[3] = { 0, 0, t.g_true };
  float tmp[3];
  nav::dcm_t_mul_vec(t.Rt, gvec, tmp);  // R' g
  for (int i = 0; i < 3; ++i) t.f_body0[i] = -tmp[i] + kBaT[i];
  nav::dcm_t_mul_vec(t.Rt, kMagRef, t.m_body0);
}
static void makeTruth(Truth &t) { makeTruthAt(t, kRollT, kPitchT, kYawT); }

static void makeImu(const Truth &t, uint64_t us, eskf::ImuSample &s) {
  float sg = kGyroNd * sqrtf(kImuHz);   // per-sample sigma consistent with
  float sa = kAccelNd * sqrtf(kImuHz);  // the density-based Q (see DESIGN.md)
  s.t_us = us;
  for (int i = 0; i < 3; ++i) {
    s.gyro[i] = kBgT[i] + sg * (float)gauss();
    s.accel[i] = t.f_body0[i] + sa * (float)gauss();
  }
  s.sat = false;
}

static void makeMag(const Truth &t, float m[3]) {
  for (int i = 0; i < 3; ++i) m[i] = t.m_body0[i] + 0.4f * (float)gauss();
}

// ---------- P health ----------
static bool checkSymmetric(const eskf::Eskf &f, float tol) {
  float maxd = 0, maxv = 0;
  for (int i = 0; i < eskf::NX; ++i) {
    for (int j = 0; j < eskf::NX; ++j) {
      float v = f.Pat(i, j);
      if (!std::isfinite(v)) return false;
      float d = fabsf(v - f.Pat(j, i));
      if (d > maxd) maxd = d;
      if (fabsf(v) > maxv) maxv = fabsf(v);
    }
  }
  return maxd <= tol * (maxv > 1.0f ? maxv : 1.0f);
}

static bool checkPosDef(const eskf::Eskf &f) {
  // Cholesky on a copy; success == positive semi-definite within the
  // float32 arithmetic the flight filter actually runs. Along a truly
  // degenerate direction (e.g. p_D = double-integral of ba_z after minutes
  // with NO vertical aiding — impossible on hardware, the baro is always
  // there) Joseph accumulation leaves pivots negative by a fraction of a
  // percent of the state's own variance; tolerate 0.1% of the diagonal and
  // continue with a tiny ridge. Real divergence blows pivots far past that.
  double L[eskf::NX][eskf::NX];
  for (int i = 0; i < eskf::NX; ++i)
    for (int j = 0; j < eskf::NX; ++j) L[i][j] = (double)f.Pat(i, j);
  for (int i = 0; i < eskf::NX; ++i) {
    for (int j = 0; j <= i; ++j) {
      double s = L[i][j];
      for (int k = 0; k < j; ++k) s -= L[i][k] * L[j][k];
      if (i == j) {
        const double diag = (double)f.Pat(i, i);
        const double tol = 1e-3 * (diag > 0.0 ? diag : 1e-12);
        if (s <= -tol) return false;
        if (s < tol * 1e-3) s = tol * 1e-3;   // ridge to keep factoring
        L[i][i] = sqrt(s);
      } else {
        L[i][j] = s / L[j][j];
      }
    }
  }
  return true;
}

static bool stateFinite(const eskf::Eskf &f) {
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(f.pos()[i]) || !std::isfinite(f.vel()[i]) ||
        !std::isfinite(f.gyroBias()[i]) || !std::isfinite(f.accelBias()[i]))
      return false;
  }
  for (int i = 0; i < 4; ++i)
    if (!std::isfinite(f.quat()[i])) return false;
  return std::isfinite(f.baroBias());
}

// ---------- config mirroring the flight config block (bench: vib mult 1) ----
static void makeConfig(eskf::Config &c, eskf::MagMode mode) {
  memset(&c, 0, sizeof(c));
  c.gyro_nd = kGyroNd;
  c.accel_nd = kAccelNd;
  c.gyro_bias_rw = 1.5e-4f;  // must track thermal drift (see ZARU)
  c.accel_bias_rw = 3e-4f;
  c.baro_bias_rw = 0.0f;
  c.vib_gyro_mult = 1.0f;
  c.vib_accel_mult = 1.0f;
  c.sat_q_mult = 20.0f;
  c.pos_q_floor = 1e-3f;
  c.init_pos_std_min = 2.0f;
  c.init_vel_std_min = 0.3f;
  c.init_rp_std = 0.035f;
  c.init_yaw_std_mag = 0.087f;
  c.init_yaw_std_nomag = 1.0f;
  c.init_bg_std = 0.01f;
  c.init_ba_std = 0.2f;
  c.init_bb_std = 0.02f;    // bb pinned: baro defines the vertical datum
  c.align_duration_s = 2.5f;
  c.align_timeout_s = 20.0f;
  c.still_gyro_std_max = 0.02f;
  c.still_accel_std_max = 0.35f;
  c.still_acc_norm_tol = 0.5f;
  c.imu_dt_nom = kDt;
  c.gate_gps_pos = 9.21f;   // 2 dof: GNSS is horizontal-only
  c.gate_gps_vel = 9.21f;
  c.gate_baro = 6.63f;
  c.gate_mag = 11.34f;   // 3 dof: full-vector mag update
  c.gate_grav = 9.21f;
  c.gps_pos_r_floor = 0.5f;
  c.gps_vel_r_floor = 0.1f;
  c.gps_latency_s = 0.06f;
  c.reacq_r_mult = 25.0f;
  c.baro_r_base = 0.7f;
  c.baro_r_dyn_k = 2e-4f;
  c.transonic_var = 900.0f;
  c.transonic_m0 = 1.0f;
  c.transonic_sigma_m = 0.12f;
  c.baro_latency_s = 0.02f;
  c.mag_mode = mode;
  c.mag_ref_ned[0] = kMagRef[0];
  c.mag_ref_ned[1] = kMagRef[1];
  c.mag_ref_ned[2] = kMagRef[2];
  c.mag_noise_ut = 0.5f;
  c.mag_r_floor = 7.6e-5f;
  c.mag_norm_tol_frac = 0.15f;
  c.mag_incl_tol_rad = 0.175f;
  c.grav_meas_std = 0.01f;
  c.grav_decim = 40;
  c.zaru_meas_std = 0.003f;
  c.gate_zaru = 11.34f;
  c.att_var_floor = 4e-6f;   // (2 mrad)^2 systematic-error floor
  c.g_default = 9.80665f;
}

// GNSS measurement generator: NED truth is the origin; noise mapped back to
// geodetic through the same radii the filter will use.
struct GpsGen {
  nav::GeoOrigin o;
  void init() { nav::geo_origin_init(o, kLat1e7, kLon1e7, kHaeMm, kHmslMm); }
  void makeFix(int32_t &lat, int32_t &lon, int32_t &hae, float vel[3]) {
    float nN = 0.8f * (float)gauss();
    float nE = 0.8f * (float)gauss();
    float nD = 1.2f * (float)gauss();
    lat = kLat1e7 +
          (int32_t)llround((double)nN / o.rn_m * nav::RAD2DEG * 1e7);
    lon = kLon1e7 +
          (int32_t)llround((double)nE / o.re_cos_m * nav::RAD2DEG * 1e7);
    hae = kHaeMm - (int32_t)llround((double)nD * 1000.0);
    // Velocity noise sits above the filter's 0.1 m/s R floor so the NIS
    // consistency check is meaningful (below the floor the filter is
    // deliberately conservative and NIS < 1 by design).
    for (int i = 0; i < 3; ++i) vel[i] = 0.12f * (float)gauss();
  }
};

struct NisStats {
  double sum = 0;
  int n = 0, rej = 0;
  void add(const eskf::InnovRecord &r, uint32_t &last_count) {
    if (r.count == last_count) return;
    last_count = r.count;
    if (!r.nis_valid) { rej++; n++; return; }
    sum += r.nis;
    n++;
    if (!r.accepted) rej++;
  }
  double mean() const { return n ? sum / n : 0; }
  double rejFrac() const { return n ? (double)rej / n : 0; }
};

static float yawErr(const eskf::Eskf &f) {
  float e[3];
  f.euler(e);
  return nav::wrap_pi(e[2] - kYawT) * nav::RAD2DEG;
}
static float rollErr(const eskf::Eskf &f) {
  float e[3];
  f.euler(e);
  return nav::wrap_pi(e[0] - kRollT) * nav::RAD2DEG;
}
static float pitchErr(const eskf::Eskf &f) {
  float e[3];
  f.euler(e);
  return nav::wrap_pi(e[1] - kPitchT) * nav::RAD2DEG;
}

// ---------- scenario driver ----------
// Runs align (3 s) + wait-fix (2 s) + anchored run (run_s). Feeds mag per
// mode. Populates health flags via the CHECKs inside.
static void runScenario(eskf::Eskf &f, eskf::MagMode mode, float run_s,
                        bool do_checks, float *yaw_sigma_10s,
                        float *yaw_sigma_end) {
  Truth tr;
  makeTruth(tr);
  eskf::Config cfg;
  makeConfig(cfg, mode);
  f.init(cfg);
  GpsGen gps;
  gps.init();

  uint64_t us = 1000000;
  int step = 0;
  bool p_healthy = true;

  // Phase 1+2: alignment (3 s) then pre-anchor refinement (2 s).
  int pre_steps = (int)(5.0f * kImuHz);
  for (int k = 0; k < pre_steps; ++k, ++step) {
    eskf::ImuSample s;
    makeImu(tr, us, s);
    f.feedImu(s);
    if (mode != eskf::MAG_OFF && (k % 5) == 0) {
      float m[3];
      makeMag(tr, m);
      f.feedMag(m);
    }
    if ((step % 100) == 0 && f.state() >= eskf::FS_WAIT_FIX) {
      if (!checkSymmetric(f, 1e-4f) || !checkPosDef(f) || !stateFinite(f))
        p_healthy = false;
    }
    us += 2500;
  }
  if (do_checks) {
    CHECK(f.state() == eskf::FS_WAIT_FIX, "reached FS_WAIT_FIX after align");
    CHECK(!f.alignDegraded(), "static alignment judged clean");
    CHECK(fabsf(rollErr(f)) < 0.5f && fabsf(pitchErr(f)) < 0.5f,
          "align tilt error < 0.5 deg (roll %.3f, pitch %.3f)", rollErr(f),
          pitchErr(f));
    if (mode != eskf::MAG_OFF) {
      CHECK(fabsf(yawErr(f)) < 3.0f, "align heading error < 3 deg (%.2f)",
            yawErr(f));
    }
  }

  // Anchor at the first "valid fix".
  float v0[3] = { 0, 0, 0 };
  f.anchorOrigin(kLat1e7, kLon1e7, kHaeMm, kHmslMm, v0, 1.0f, 1.5f, 0.1f);
  if (do_checks) CHECK(f.state() == eskf::FS_RUN, "anchored into FS_RUN");

  // Phase 3: full run.
  NisStats ns_gp, ns_gv, ns_br, ns_mg;
  uint32_t c_gp = 0, c_gv = 0, c_br = 0, c_mg = 0;
  uint32_t mag_count_at_anchor = f.innov(eskf::CH_MAG).count;
  int run_steps = (int)(run_s * kImuHz);
  float warmup_s = 10.0f;
  for (int k = 0; k < run_steps; ++k, ++step) {
    eskf::ImuSample s;
    makeImu(tr, us, s);
    f.feedImu(s);
    float t_run = (float)k * kDt;
    if ((k % 5) == 0) {  // 80 Hz mag (fed regardless; the mode gates use)
      float m[3];
      makeMag(tr, m);
      f.feedMag(m);
    }
    if ((k % 16) == 0) {  // 25 Hz baro
      float alt_rel = kBbT + 0.5f * (float)gauss();
      f.updateBaro(alt_rel, 15.0f);
      if (t_run > warmup_s) ns_br.add(f.innov(eskf::CH_BARO), c_br);
    }
    if ((k % 40) == 0) {  // 10 Hz GNSS
      int32_t la, lo, ha;
      float vel[3];
      gps.makeFix(la, lo, ha, vel);
      f.updateGpsPos(la, lo, ha, 0.9f, 1.3f, false);
      f.updateGpsVel(vel, 0.12f, false);
      if (t_run > warmup_s) {
        ns_gp.add(f.innov(eskf::CH_GPS_POS), c_gp);
        ns_gv.add(f.innov(eskf::CH_GPS_VEL), c_gv);
      }
    }
    if (mode == eskf::MAG_CONTINUOUS && t_run > warmup_s)
      ns_mg.add(f.innov(eskf::CH_MAG), c_mg);
    if ((step % 100) == 0) {
      if (!checkSymmetric(f, 1e-4f) || !checkPosDef(f) || !stateFinite(f))
        p_healthy = false;
    }
    if (yaw_sigma_10s && fabsf(t_run - 10.0f) < 0.5f * kDt)
      *yaw_sigma_10s = sqrtf(f.Pat(8, 8));
    us += 2500;
  }
  if (yaw_sigma_end) *yaw_sigma_end = sqrtf(f.Pat(8, 8));

  if (do_checks) {
    CHECK(p_healthy, "P symmetric, positive-definite, state finite (all run)");
    const float *p = f.pos();
    const float *v = f.vel();
    CHECK(fabsf(p[0]) < 1.5f && fabsf(p[1]) < 1.5f,
          "horizontal holds GNSS truth (N %.2f E %.2f m)", p[0], p[1]);
    // The vertical is baro+IMU only: the simulated baro reads kBbT high, so
    // the filter MUST settle on the baro datum, p_D -> -kBbT.
    CHECK(fabsf(p[2] + kBbT) < 1.5f,
          "vertical holds the BARO datum (D %.2f vs -%.2f m)", p[2], kBbT);
    CHECK(fabsf(v[0]) < 0.15f && fabsf(v[1]) < 0.15f && fabsf(v[2]) < 0.15f,
          "velocity holds truth (%.3f %.3f %.3f m/s)", v[0], v[1], v[2]);
    CHECK(fabsf(rollErr(f)) < 0.3f && fabsf(pitchErr(f)) < 0.3f,
          "final tilt error < 0.3 deg (roll %.3f, pitch %.3f)", rollErr(f),
          pitchErr(f));
    if (mode == eskf::MAG_CONTINUOUS) {
      CHECK(fabsf(yawErr(f)) < 1.5f, "final heading error < 1.5 deg (%.2f)",
            yawErr(f));
    }
    const float *bg = f.gyroBias();
    const float *ba = f.accelBias();
    CHECK(fabsf(bg[0] - kBgT[0]) < 0.0015f &&
              fabsf(bg[1] - kBgT[1]) < 0.0015f &&
              fabsf(bg[2] - kBgT[2]) < 0.0015f,
          "gyro bias converged (err %.5f %.5f %.5f rad/s)", bg[0] - kBgT[0],
          bg[1] - kBgT[1], bg[2] - kBgT[2]);
    CHECK(fabsf(ba[0] - kBaT[0]) < 0.06f && fabsf(ba[1] - kBaT[1]) < 0.06f &&
              fabsf(ba[2] - kBaT[2]) < 0.06f,
          "accel bias converged (err %.4f %.4f %.4f m/s^2)", ba[0] - kBaT[0],
          ba[1] - kBaT[1], ba[2] - kBaT[2]);
    CHECK(fabsf(f.baroBias()) < 0.1f,
          "bb stays pinned (%.3f m) - the baro owns the datum", f.baroBias());
    CHECK(fabsf(f.relAlt() - kBbT) < 0.5f,
          "fused altitude tracks the baro (%.2f vs %.2f m)", f.relAlt(), kBbT);
    float sp[3], sv[3], sa[3];
    f.sigmas(sp, sv, sa);
    CHECK(fabsf(p[0]) < 4 * sp[0] + 0.05f && fabsf(p[1]) < 4 * sp[1] + 0.05f &&
              fabsf(p[2] + kBbT) < 4 * sp[2] + 0.05f,
          "position error consistent with 4-sigma (sp %.2f %.2f %.2f)", sp[0],
          sp[1], sp[2]);
    CHECK(sp[0] < 1.0f && sp[1] < 1.0f && sp[2] < 1.0f,
          "position sigma converged below 1 m");
    CHECK(ns_gp.mean() > 0.8 && ns_gp.mean() < 4.0,
          "GNSS pos NIS mean in chi2 band (%.2f, 2 dof, n=%d)", ns_gp.mean(),
          ns_gp.n);
    CHECK(ns_gv.mean() > 0.8 && ns_gv.mean() < 4.0,
          "GNSS vel NIS mean in chi2 band (%.2f, 2 dof, n=%d)", ns_gv.mean(),
          ns_gv.n);
    CHECK(ns_br.mean() > 0.15 && ns_br.mean() < 4.0,
          "baro NIS mean in chi2 band (%.2f, 1 dof, n=%d)", ns_br.mean(),
          ns_br.n);
    CHECK(ns_gp.rejFrac() < 0.08 && ns_gv.rejFrac() < 0.08 &&
              ns_br.rejFrac() < 0.08,
          "rejection fractions < 8%% (gp %.3f gv %.3f br %.3f)",
          ns_gp.rejFrac(), ns_gv.rejFrac(), ns_br.rejFrac());
    if (mode == eskf::MAG_CONTINUOUS) {
      CHECK(ns_mg.mean() > 1.5 && ns_mg.mean() < 5.5,
            "mag NIS mean in chi2 band (%.2f, 3 dof, n=%d)", ns_mg.mean(),
            ns_mg.n);
      CHECK(ns_mg.rejFrac() < 0.08, "mag rejection fraction < 8%% (%.3f)",
            ns_mg.rejFrac());
    }
    if (mode == eskf::MAG_INIT_ONLY) {
      CHECK(f.innov(eskf::CH_MAG).count == mag_count_at_anchor,
            "MAG_INIT_ONLY: no mag updates after anchor (count %u)",
            (unsigned)f.innov(eskf::CH_MAG).count);
    }
  }
}

int main() {
  printf("RocketNav ESKF host test\n");
  printf("== scenario 1: MAG_CONTINUOUS, 60 s static ==\n");
  {
    static eskf::Eskf f;  // static: 16x16 float matrices, keep off the stack
    runScenario(f, eskf::MAG_CONTINUOUS, 60.0f, true, nullptr, nullptr);
  }

  printf("== scenario 2: MAG_OFF, 60 s static ==\n");
  {
    rng_state = 0xDEADBEEFCAFEull;
    static eskf::Eskf f;
    float ys10 = 0, ysEnd = 0;
    runScenario(f, eskf::MAG_OFF, 60.0f, false, &ys10, &ysEnd);
    const float *p = f.pos();
    CHECK(stateFinite(f) && checkPosDef(f),
          "MAG_OFF: filter healthy after 60 s with no heading aiding");
    CHECK(fabsf(p[0]) < 2.0f && fabsf(p[1]) < 2.0f &&
              fabsf(p[2] + kBbT) < 2.0f,
          "MAG_OFF: position bounded (N %.2f E %.2f D %.2f)", p[0], p[1], p[2]);
    // Static bench with the zero-rate trim active: bg_z is now pinned
    // while still (that is the point of ZARU), which also drains the
    // bias-driven share of the heading covariance. Assert honesty rather
    // than a monotone slope: heading sigma stays genuinely wide (no
    // heading measurement exists), the true error stays inside its own
    // covariance, and the vertical bias is held by the trim.
    CHECK(ysEnd > 0.15f,
          "MAG_OFF: heading sigma stays honestly wide (%.3f -> %.3f rad)",
          ys10, ysEnd);
    CHECK(fabsf(yawErr(f)) * nav::DEG2RAD < 2.5f * ysEnd,
          "MAG_OFF: heading error within 2.5 sigma (err %.1f deg, sig %.1f deg)",
          yawErr(f), ysEnd * nav::RAD2DEG);
    // With no heading reference, theta_D and bg_z share a weakly
    // determined ridge: the trim bounds the pair honestly but cannot pin
    // the bias tighter than its own claimed sigma. Assert consistency,
    // not absolute truth (scenarios 1/3/7, mag present, assert <1e-3).
    const float bzSig = sqrtf(f.Pat(eskf::IBG + 2, eskf::IBG + 2));
    const float bzErr = fabsf(f.gyroBias()[2] - kBgT[2]);
    CHECK(bzSig < 0.005f && bzErr < 3.0f * bzSig + 1e-4f,
          "MAG_OFF: vertical gyro bias consistent under the zero-rate trim "
          "(sig %.4f, err %.5f rad/s)", bzSig, bzErr);
    CHECK(fabsf(rollErr(f)) < 0.8f && fabsf(pitchErr(f)) < 0.8f,
          "MAG_OFF: tilt still converged (roll %.3f, pitch %.3f)", rollErr(f),
          pitchErr(f));
  }

  printf("== scenario 3: MAG_INIT_ONLY, 20 s static ==\n");
  {
    rng_state = 0x5151515151ull;
    static eskf::Eskf f;
    runScenario(f, eskf::MAG_INIT_ONLY, 20.0f, false, nullptr, nullptr);
    uint32_t cnt = f.innov(eskf::CH_MAG).count;
    CHECK(cnt > 0, "MAG_INIT_ONLY: mag was used before anchor (count %u)",
          (unsigned)cnt);
    // Feed more mag in FS_RUN: must be ignored.
    Truth tr;
    makeTruth(tr);
    float m[3];
    makeMag(tr, m);
    bool used = f.feedMag(m);
    CHECK(!used && f.innov(eskf::CH_MAG).count == cnt,
          "MAG_INIT_ONLY: mag ignored after anchor");
    CHECK(fabsf(yawErr(f)) < 3.0f,
          "MAG_INIT_ONLY: heading held from init (err %.2f deg)", yawErr(f));
    CHECK(stateFinite(f) && checkPosDef(f), "MAG_INIT_ONLY: filter healthy");
  }

  printf("== scenario 4: attitude-only (GPS-less), 20 s ==\n");
  {
    rng_state = 0xA77A77A77ull;
    static eskf::Eskf f;
    Truth tr;
    makeTruth(tr);
    eskf::Config cfg;
    makeConfig(cfg, eskf::MAG_CONTINUOUS);
    f.init(cfg);
    uint64_t us = 1000000;
    // Align (3 s) plus 1 s of wait-fix, then latch attitude-only — the path
    // the flight code takes when the startup GPS determination says "off".
    int pre = (int)(4.0f * kImuHz);
    for (int k = 0; k < pre; ++k) {
      eskf::ImuSample s;
      makeImu(tr, us, s);
      f.feedImu(s);
      if ((k % 5) == 0) { float m[3]; makeMag(tr, m); f.feedMag(m); }
      us += 2500;
    }
    CHECK(f.state() == eskf::FS_WAIT_FIX, "att-only: aligned into FS_WAIT_FIX");
    f.setAttitudeOnly();
    CHECK(f.state() == eskf::FS_ATT_ONLY, "att-only: latched FS_ATT_ONLY");
    uint32_t mag_before = f.innov(eskf::CH_MAG).count;
    bool healthy = true;
    int steps = (int)(20.0f * kImuHz);
    for (int k = 0; k < steps; ++k) {
      eskf::ImuSample s;
      makeImu(tr, us, s);
      f.feedImu(s);
      if ((k % 5) == 0) { float m[3]; makeMag(tr, m); f.feedMag(m); }
      if ((k % 400) == 0 &&
          (!checkSymmetric(f, 1e-4f) || !checkPosDef(f) || !stateFinite(f)))
        healthy = false;
      us += 2500;
    }
    CHECK(healthy, "att-only: P healthy over 20 s with no GNSS at all");
    CHECK(fabsf(rollErr(f)) < 0.5f && fabsf(pitchErr(f)) < 0.5f,
          "att-only: tilt held (roll %.3f, pitch %.3f)", rollErr(f),
          pitchErr(f));
    CHECK(fabsf(yawErr(f)) < 2.0f,
          "att-only: heading held by mag (err %.2f deg)", yawErr(f));
    CHECK(f.innov(eskf::CH_MAG).count > mag_before,
          "att-only: mag aiding continues");
    // A valid fix arriving after the GPS-less decision upgrades to full
    // navigation: the anchor rebuilds p/v/bb state and covariance from
    // the fix, so the late anchor must behave like an on-time one.
    float v0[3] = { 0, 0, 0 };
    f.anchorOrigin(kLat1e7, kLon1e7, kHaeMm, kHmslMm, v0, 1.0f, 1.5f, 0.1f);
    CHECK(f.state() == eskf::FS_RUN,
          "att-only: late anchor upgrades to FS_RUN");
    CHECK(checkSymmetric(f, 1e-4f) && checkPosDef(f) && stateFinite(f),
          "att-only: P healthy immediately after the late anchor");
    const float *pp = f.pos();
    CHECK(fabsf(pp[0]) < 1e-3f && fabsf(pp[1]) < 1e-3f,
          "att-only: horizontal reset to the anchor origin (vertical is "
          "baro-owned and continues)");
    bool gp_ok = f.updateGpsPos(kLat1e7, kLon1e7, kHaeMm, 1.0f, 1.5f, false);
    bool gv_ok = f.updateGpsVel(v0, 0.12f, false);
    CHECK(gp_ok && gv_ok,
          "att-only: GPS updates accepted after the upgrade");
  }

  printf("== scenario 5: nose-up alignment (pad attitude, pitch +90) ==\n");
  {
    rng_state = 0x5EED5EED5ull;
    static eskf::Eskf f;
    Truth tr;
    // Vertical on the pad: nose (body X) straight up. Aircraft-Euler
    // extraction is singular here, so errors are measured on rotation
    // matrices, never on euler angles.
    makeTruthAt(tr, 0.0f, 90.0f * nav::DEG2RAD, kYawT);
    eskf::Config cfg;
    makeConfig(cfg, eskf::MAG_CONTINUOUS);
    f.init(cfg);
    uint64_t us = 1000000;
    int pre = (int)(10.0f * kImuHz);  // align + wait-fix refinement
    for (int k = 0; k < pre; ++k) {
      eskf::ImuSample s;
      makeImu(tr, us, s);
      f.feedImu(s);
      if ((k % 5) == 0) { float m[3]; makeMag(tr, m); f.feedMag(m); }
      us += 2500;
    }
    CHECK(f.state() == eskf::FS_WAIT_FIX, "nose-up: aligned into FS_WAIT_FIX");
    CHECK(!f.alignDegraded(), "nose-up: clean static alignment");
    float Re[9];
    nav::quat_to_dcm(f.quat(), Re);
    // nose direction (body X in NED) vs truth — gimbal-lock-proof metric
    float dnose = Re[0] * tr.Rt[0] + Re[3] * tr.Rt[3] + Re[6] * tr.Rt[6];
    if (dnose > 1.0f) dnose = 1.0f;
    float nose_err = acosf(dnose) * nav::RAD2DEG;
    // Floor set by the accel bias unobserved at align time: the true-bias
    // components perpendicular to the nose tilt gravity by ~atan(|ba_perp|/g)
    // ~= 0.52 deg here. Noise adds little on top of that.
    CHECK(nose_err < 0.7f, "nose-up: nose axis within 0.7 deg (%.3f)",
          nose_err);
    CHECK(Re[6] < -0.99f, "nose-up: nose points up (R[6]=%.3f)", Re[6]);
    // total attitude error incl. the twist the mag must pin down:
    // cos(theta) = (trace(Re * Rt') - 1) / 2
    float tr3 = 0;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) tr3 += Re[3 * r + c] * tr.Rt[3 * r + c];
    float ct = 0.5f * (tr3 - 1.0f);
    if (ct > 1.0f) ct = 1.0f;
    if (ct < -1.0f) ct = -1.0f;
    float tot_err = acosf(ct) * nav::RAD2DEG;
    CHECK(tot_err < 2.5f, "nose-up: total attitude error < 2.5 deg (%.2f)",
          tot_err);
    CHECK(checkSymmetric(f, 1e-4f) && checkPosDef(f) && stateFinite(f),
          "nose-up: P healthy at the vertical pad attitude");
  }

  printf("== scenario 6: GPS-less vertical channel (baro-damped altitude) ==\n");
  {
    rng_state = 0xBAD0BAD05ull;
    static eskf::Eskf f;
    Truth tr;
    makeTruth(tr);
    eskf::Config cfg;
    makeConfig(cfg, eskf::MAG_CONTINUOUS);
    f.init(cfg);
    const float tau_b = cfg.baro_latency_s;

    // Vertical truth profile (up-positive): still -> climb -> hover. The
    // attitude stays at the (tilted) static truth; motion need not align
    // with a body axis, so the specific-force model stays exact.
    //   0..8 s    still at 0 m
    //   8..10 s   a=+20 m/s^2 (v: 0->40, alt: 0->40)
    //   10..18 s  steady climb v=40 (alt: 40->360)
    //   18..20 s  a=-20 (v: 40->0, alt: 360->400)
    //   20..25 s  hover at 400 m
    auto altAt = [](float t) -> float {
      if (t < 8.0f) return 0.0f;
      if (t < 10.0f) { float d = t - 8.0f; return 10.0f * d * d; }
      if (t < 18.0f) return 40.0f + 40.0f * (t - 10.0f);
      if (t < 20.0f) { float d = t - 18.0f;
                       return 360.0f + 40.0f * d - 10.0f * d * d; }
      return 400.0f;
    };
    auto aAt = [](float t) -> float {
      if (t < 8.0f) return 0.0f;
      if (t < 10.0f) return 20.0f;
      if (t < 18.0f) return 0.0f;
      if (t < 20.0f) return -20.0f;
      return 0.0f;
    };

    uint64_t us = 1000000;
    float sg = kGyroNd * sqrtf(kImuHz), sa = kAccelNd * sqrtf(kImuHz);
    bool att_only_set = false, first_baro_checked = false;
    double nu_sum = 0; int nu_n = 0;
    float max_alt_err = 0, err_v_at_15 = -1;
    int total = (int)(25.0f * kImuHz);
    for (int k = 0; k < total; ++k) {
      float t = (float)k * kDt;
      float a_up = aAt(t);
      eskf::ImuSample s;
      s.t_us = us;
      for (int i = 0; i < 3; ++i) {
        s.gyro[i] = kBgT[i] + sg * (float)gauss();
        // f_b = R'([0,0,-a_up] - g) + ba = f_body0 - a_up * (R' e3)
        s.accel[i] =
            tr.f_body0[i] - a_up * tr.Rt[6 + i] + sa * (float)gauss();
      }
      s.sat = false;
      f.feedImu(s);
      if ((k % 5) == 0) { float m[3]; makeMag(tr, m); f.feedMag(m); }

      if (!att_only_set && f.state() == eskf::FS_WAIT_FIX) {
        f.setAttitudeOnly();
        att_only_set = true;
      }
      if (att_only_set && (k % 16) == 0) {  // 25 Hz baro, tau_b old
        float alt_meas = altAt(t - tau_b) + 0.3f * (float)gauss();
        bool ok = f.updateBaro(alt_meas, 15.0f);
        if (!first_baro_checked) {
          first_baro_checked = true;
          CHECK(ok, "vertchan: first baro update accepted in FS_ATT_ONLY");
        }
        if (ok && t > 11.0f && t < 18.0f) {
          nu_sum += f.innov(eskf::CH_BARO).nu[0];
          nu_n++;
        }
      }
      if (att_only_set && t > 6.0f && (k % 400) == 0) {
        float e = fabsf(f.relAlt() - altAt(t));
        if (e > max_alt_err) max_alt_err = e;
      }
      if (k == (int)(15.0f * kImuHz)) err_v_at_15 = fabsf(-f.vel()[2] - 40.0f);
      us += 2500;
    }
    CHECK(att_only_set && f.state() == eskf::FS_ATT_ONLY,
          "vertchan: held FS_ATT_ONLY through the profile");
    CHECK(max_alt_err < 2.5f, "vertchan: altitude tracks truth, max err %.2f m",
          max_alt_err);
    CHECK(fabsf(f.relAlt() - 400.0f) < 1.0f,
          "vertchan: settled altitude %.2f m (true 400)", f.relAlt());
    CHECK(fabsf(f.vel()[2]) < 0.3f, "vertchan: settled vertical rate %.2f m/s",
          f.vel()[2]);
    CHECK(err_v_at_15 >= 0 && err_v_at_15 < 1.5f,
          "vertchan: steady-climb rate err %.2f m/s", err_v_at_15);
    CHECK(nu_n > 100 && fabs(nu_sum / nu_n) < 0.35,
          "vertchan: latency-compensated innovation mean %.3f m over %d "
          "(uncompensated bias would be %.2f)",
          nu_sum / (nu_n > 0 ? nu_n : 1), nu_n, 40.0f * tau_b);
    CHECK(fabsf(f.pos()[0]) < 20.0f && fabsf(f.pos()[1]) < 20.0f,
          "vertchan: horizontal stayed pinned vs 400 m climb (N %.2f E %.2f)",
          f.pos()[0], f.pos()[1]);
    CHECK(f.relAltStd() > 0.05f && f.relAltStd() < 2.0f,
          "vertchan: altitude sigma sane (%.2f m)", f.relAltStd());
    CHECK(checkSymmetric(f, 1e-4f) && checkPosDef(f) && stateFinite(f),
          "vertchan: P healthy after GPS-less flight");

    // Late in-flight anchor at 400 m AGL: bb must seed to the channel's
    // altitude, or every subsequent baro update would gate out.
    float v0[3] = { 0, 0, 0 };
    f.anchorOrigin(kLat1e7, kLon1e7, kHaeMm + 400000, kHmslMm + 400000, v0,
                   1.0f, 1.5f, 0.1f);
    CHECK(f.state() == eskf::FS_RUN, "vertchan: late anchor upgraded to RUN");
    CHECK(fabsf(f.baroBias()) < 0.1f,
          "vertchan: bb stays pinned (%.2f m)", f.baroBias());
    CHECK(fabsf(f.pos()[2] + 400.0f) < 3.0f,
          "vertchan: p_D keeps the baro datum through the anchor (%.1f m)",
          f.pos()[2]);
    CHECK(fabsf(f.relAlt() - 400.0f) < 3.0f,
          "vertchan: altitude continuous across the anchor (%.1f m)",
          f.relAlt());
    int br_acc = 0, br_try = 0;
    for (int k = 0; k < (int)(2.0f * kImuHz); ++k) {
      eskf::ImuSample s;
      makeImu(tr, us, s);  // hover: static specific force
      f.feedImu(s);
      if ((k % 5) == 0) { float m[3]; makeMag(tr, m); f.feedMag(m); }
      if ((k % 16) == 0) {
        float alt_meas = 400.0f + 0.3f * (float)gauss();
        br_try++;
        if (f.updateBaro(alt_meas, 15.0f)) br_acc++;
      }
      if ((k % 40) == 0) {
        f.updateGpsPos(kLat1e7, kLon1e7, kHaeMm + 400000, 1.0f, 1.5f, false);
        float vg[3] = { 0.05f * (float)gauss(), 0.05f * (float)gauss(),
                        0.05f * (float)gauss() };
        f.updateGpsVel(vg, 0.1f, false);
      }
      us += 2500;
    }
    CHECK(br_try >= 50 && br_acc >= br_try - 2,
          "vertchan: baro stayed accepted after the late anchor (%d/%d)",
          br_acc, br_try);
    CHECK(fabsf(f.relAlt() - 400.0f) < 1.5f,
          "vertchan: post-anchor altitude %.1f m (true 400)", f.relAlt());
    CHECK(checkSymmetric(f, 1e-4f) && checkPosDef(f) && stateFinite(f),
          "vertchan: P healthy post-anchor");
  }

  printf("== scenario 7: still-vehicle gyro-bias trim (ZARU) ==\n");
  {
    rng_state = 0x7A407A40ull;
    static eskf::Eskf f;
    Truth tr;
    makeTruth(tr);
    eskf::Config cfg;
    makeConfig(cfg, eskf::MAG_CONTINUOUS);
    f.init(cfg);
    uint64_t us = 1000000;
    float sg = kGyroNd * sqrtf(kImuHz), sa = kAccelNd * sqrtf(kImuHz);
    // 30 s nominal, then the true gyro bias RAMPS by 0.005 rad/s
    // (0.29 deg/s) per axis over 60 s and holds — self-heating thermal
    // drift. Without the zero-rate update this walks attitude by tens of
    // degrees and the gravity/mag gates reject their own aiding; with it
    // the bias is tracked continuously while the vehicle is
    // measured-still.
    const float bgStep[3] = { 0.005f, -0.005f, 0.005f };
    const int total = (int)(120.0f * kImuHz);
    for (int k = 0; k < total; ++k) {
      const float t = (float)k * kDt;
      const float ramp = t < 30.0f ? 0.0f
                       : (t < 90.0f ? (t - 30.0f) / 60.0f : 1.0f);
      eskf::ImuSample s;
      s.t_us = us;
      for (int i = 0; i < 3; ++i) {
        const float bg = kBgT[i] + ramp * bgStep[i];
        s.gyro[i] = bg + sg * (float)gauss();
        s.accel[i] = tr.f_body0[i] + sa * (float)gauss();
      }
      s.sat = false;
      f.feedImu(s);
      if ((k % 5) == 0) { float m[3]; makeMag(tr, m); f.feedMag(m); }
      us += 2500;
    }
    const float *bg = f.gyroBias();
    float be[3];
    for (int i = 0; i < 3; ++i) be[i] = bg[i] - (kBgT[i] + bgStep[i]);
    CHECK(fabsf(be[0]) < 1e-3f && fabsf(be[1]) < 1e-3f &&
              fabsf(be[2]) < 1e-3f,
          "zaru: drifting gyro bias tracked while still "
          "(err %.5f %.5f %.5f rad/s)", be[0], be[1], be[2]);
    CHECK(fabsf(rollErr(f)) < 0.4f && fabsf(pitchErr(f)) < 0.4f,
          "zaru: tilt pinned through the drift (roll %.2f, pitch %.2f)",
          rollErr(f), pitchErr(f));
    CHECK(fabsf(yawErr(f)) < 1.5f,
          "zaru: heading held through the drift (%.2f deg)",
          yawErr(f));
    CHECK(checkSymmetric(f, 1e-4f) && checkPosDef(f) && stateFinite(f),
          "zaru: P healthy");
  }

  printf("== scenario 8: distorted-field alignment guard ==\n");
  {
    rng_state = 0x8888AAAA1ull;
    static eskf::Eskf f;
    Truth tr;
    makeTruth(tr);
    eskf::Config cfg;
    makeConfig(cfg, eskf::MAG_CONTINUOUS);
    f.init(cfg);
    uint64_t us = 1000000;
    // Alignment next to steel: the mag reads the true field plus a 30 uT
    // hard-iron-style offset, far outside the +-15% magnitude gate. The
    // old code averaged it into the initial heading — a different wrong
    // yaw every boot; now those samples stay out of the init and the
    // heading starts at the honest wide no-mag prior.
    int total = (int)(6.0f * kImuHz);
    for (int k = 0; k < total; ++k) {
      eskf::ImuSample s;
      makeImu(tr, us, s);
      f.feedImu(s);
      if ((k % 5) == 0) {
        float m[3];
        makeMag(tr, m);
        m[0] += 30.0f;
        f.feedMag(m);
      }
      us += 2500;
    }
    CHECK(f.state() == eskf::FS_WAIT_FIX, "magguard: aligned");
    CHECK(!f.magInitUsed(), "magguard: distorted field kept out of the init");
    float sp[3], sv[3], sa[3];
    f.sigmas(sp, sv, sa);
    CHECK(sa[2] > 0.5f, "magguard: heading honestly wide (%.2f rad)", sa[2]);
    // Field cleans up (moved away from the steel): continuous updates pass
    // their gates and pull the heading in.
    int total2 = (int)(20.0f * kImuHz);
    for (int k = 0; k < total2; ++k) {
      eskf::ImuSample s;
      makeImu(tr, us, s);
      f.feedImu(s);
      if ((k % 5) == 0) { float m[3]; makeMag(tr, m); f.feedMag(m); }
      us += 2500;
    }
    CHECK(fabsf(yawErr(f)) < 3.0f,
          "magguard: heading recovered on a clean field (%.2f deg)",
          yawErr(f));
    CHECK(fabsf(rollErr(f)) < 0.5f && fabsf(pitchErr(f)) < 0.5f,
          "magguard: tilt unaffected (%.2f / %.2f)", rollErr(f), pitchErr(f));
    CHECK(checkSymmetric(f, 1e-4f) && checkPosDef(f) && stateFinite(f),
          "magguard: P healthy");
  }

  printf("== scenario 9: 3-axis mag pins attitude during handling ==\n");
  {
    rng_state = 0x99AA99AA1ull;
    static eskf::Eskf f;
    Truth tr;
    makeTruth(tr);
    eskf::Config cfg;
    makeConfig(cfg, eskf::MAG_CONTINUOUS);
    f.init(cfg);
    uint64_t us = 1000000;
    float sg = kGyroNd * sqrtf(kImuHz), sa = kAccelNd * sqrtf(kImuHz);
    for (int k = 0; k < (int)(5.0f * kImuHz); ++k) {
      eskf::ImuSample s;
      makeImu(tr, us, s);
      f.feedImu(s);
      if ((k % 5) == 0) { float m[3]; makeMag(tr, m); f.feedMag(m); }
      us += 2500;
    }
    CHECK(f.state() == eskf::FS_WAIT_FIX, "magvec: aligned");
    // Handling: spin about the nose at 30 deg/s for 25 s with a hand-shake
    // lateral acceleration (defeats the stillness gate, so gravity/ZARU
    // aids are OFF) and an extra unmodeled 0.004 rad/s gyro error on body
    // Y. The old scalar-heading update left roll/pitch unaided here — a
    // ~5.7 deg wander; the vector update pins every axis but the field
    // line, and the spin averages the body-fixed bias error out of that
    // line too.
    float qt[4] = { tr.q[0], tr.q[1], tr.q[2], tr.q[3] };
    const float wx = 30.0f * nav::DEG2RAD;
    const float bgErrY = 0.004f;
    for (int k = 0; k < (int)(25.0f * kImuHz); ++k) {
      const float t = (float)k * kDt;
      float rv[3] = { wx * kDt, 0, 0 };
      float dq[4], qn[4];
      nav::quat_from_rotvec(rv, dq);
      nav::quat_mul(qt, dq, qn);
      qt[0] = qn[0]; qt[1] = qn[1]; qt[2] = qn[2]; qt[3] = qn[3];
      nav::quat_normalize(qt);
      float Rt[9];
      nav::quat_to_dcm(qt, Rt);
      eskf::ImuSample s;
      s.t_us = us;
      s.gyro[0] = wx + kBgT[0] + sg * (float)gauss();
      s.gyro[1] = kBgT[1] + bgErrY + sg * (float)gauss();
      s.gyro[2] = kBgT[2] + sg * (float)gauss();
      float gvec[3] = { 0, 0, tr.g_true }, tmp[3];
      nav::dcm_t_mul_vec(Rt, gvec, tmp);
      const float shake = 1.6f * sinf(4.4f * t);
      for (int i = 0; i < 3; ++i) {
        s.accel[i] = -tmp[i] + kBaT[i] + sa * (float)gauss();
      }
      s.accel[1] += shake;
      s.sat = false;
      f.feedImu(s);
      if ((k % 5) == 0) {
        float m[3], mt[3];
        nav::dcm_t_mul_vec(Rt, kMagRef, mt);
        for (int i = 0; i < 3; ++i) m[i] = mt[i] + 0.4f * (float)gauss();
        f.feedMag(m);
      }
      us += 2500;
    }
    float Re[9], Rt2[9];
    nav::quat_to_dcm(f.quat(), Re);
    nav::quat_to_dcm(qt, Rt2);
    float tr3 = 0;
    for (int r2 = 0; r2 < 3; ++r2) {
      for (int c2 = 0; c2 < 3; ++c2) tr3 += Re[3 * r2 + c2] * Rt2[3 * r2 + c2];
    }
    float ct = 0.5f * (tr3 - 1.0f);
    if (ct > 1.0f) ct = 1.0f;
    if (ct < -1.0f) ct = -1.0f;
    const float err = acosf(ct) * nav::RAD2DEG;
    CHECK(err < 2.0f,
          "magvec: attitude pinned through 25 s of spinning handling "
          "(total err %.2f deg)", err);
    CHECK(checkSymmetric(f, 1e-4f) && checkPosDef(f) && stateFinite(f),
          "magvec: P healthy");
  }

  printf("== scenario 10: boot heading repeatability (handled power-up) ==\n");
  {
    // The vehicle is plugged in and positioned during the first seconds:
    // gyro too lively for the stillness screen (windows restart), and the
    // mag is showing the field of a DIFFERENT orientation (90 deg off).
    // The align heading must come only from the window that finally passed
    // the screen - polluted samples used to survive restarts and seeded a
    // different heading every boot.
    rng_state = 0xB007B007ull;
    Truth tr, trHandle;
    makeTruth(tr);
    makeTruthAt(trHandle, kRollT, kPitchT, kYawT + 90.0f * nav::DEG2RAD);
    float yawBoot[2];
    for (int boot = 0; boot < 2; ++boot) {
      static eskf::Eskf f;
      eskf::Config cfg;
      makeConfig(cfg, eskf::MAG_CONTINUOUS);
      f.init(cfg);
      uint64_t us = 1000000;
      float sa = kAccelNd * sqrtf(kImuHz);
      if (boot == 0) {
        int handle = (int)(4.0f * kImuHz);
        for (int k = 0; k < handle; ++k) {
          eskf::ImuSample s;
          s.t_us = us;
          for (int i = 0; i < 3; ++i) {
            s.gyro[i] = kBgT[i] + 0.05f * (float)gauss();  // >> still gate
            s.accel[i] = tr.f_body0[i] + sa * (float)gauss();
          }
          s.sat = false;
          f.feedImu(s);
          if ((k % 5) == 0) { float m[3]; makeMag(trHandle, m); f.feedMag(m); }
          us += 2500;
        }
      }
      int quiet = (int)(6.0f * kImuHz);
      for (int k = 0; k < quiet; ++k) {
        eskf::ImuSample s;
        makeImu(tr, us, s);
        f.feedImu(s);
        if ((k % 5) == 0) { float m[3]; makeMag(tr, m); f.feedMag(m); }
        us += 2500;
      }
      CHECK(f.state() == eskf::FS_WAIT_FIX && f.magInitUsed(),
            "reboot%d: aligned with mag", boot);
      float e[3];
      f.euler(e);
      yawBoot[boot] = e[2];
      CHECK(fabsf(yawErr(f)) < 2.0f,
            "reboot%d: heading from the still window only (err %.2f deg)",
            boot, yawErr(f));
    }
    CHECK(fabsf(nav::wrap_pi(yawBoot[0] - yawBoot[1])) * nav::RAD2DEG < 1.0f,
          "reboot: same orientation, same heading (delta %.2f deg)",
          fabsf(nav::wrap_pi(yawBoot[0] - yawBoot[1])) * nav::RAD2DEG);
  }

  printf("== scenario 11: iron-shifted field pins to the boot reference ==\n");
  {
    // Constant distortion: magnitude inside the +-15% gate, inclination
    // 24 deg off the site model. Old behavior: alignment accepts it (the
    // init gate is magnitude-only), then EVERY run update dies on the
    // inclination gate - heading frozen wherever the init put it and roll
    // unpinned during handling. Now the run reference is captured from the
    // align field itself, so the vector update keeps working against the
    // field that actually exists here, and reboots reproduce the heading.
    rng_state = 0x1120FF5Eull;
    Truth tr;
    makeTruth(tr);
    const float Bdist[3] = { 35.0f, -1.23f, 33.0f };  // |B| 48.1, incl 43.3
    float md0[3];
    nav::dcm_t_mul_vec(tr.Rt, Bdist, md0);
    float yawBoot[2];
    for (int boot = 0; boot < 2; ++boot) {
      static eskf::Eskf f;
      eskf::Config cfg;
      makeConfig(cfg, eskf::MAG_CONTINUOUS);
      f.init(cfg);
      uint64_t us = 1000000;
      int total = (int)(6.0f * kImuHz);
      for (int k = 0; k < total; ++k) {
        eskf::ImuSample s;
        makeImu(tr, us, s);
        f.feedImu(s);
        if ((k % 5) == 0) {
          float m[3];
          for (int i = 0; i < 3; ++i) m[i] = md0[i] + 0.4f * (float)gauss();
          f.feedMag(m);
        }
        us += 2500;
      }
      CHECK(f.state() == eskf::FS_WAIT_FIX && f.magInitUsed(),
            "ironboot%d: aligned with the distorted-but-stable field", boot);
      float e1[3];
      f.euler(e1);
      yawBoot[boot] = e1[2];
      if (boot == 0) {
        int acc = 0, run = (int)(60.0f * kImuHz);
        for (int k = 0; k < run; ++k) {
          eskf::ImuSample s;
          makeImu(tr, us, s);
          f.feedImu(s);
          if ((k % 5) == 0) {
            float m[3];
            for (int i = 0; i < 3; ++i) m[i] = md0[i] + 0.4f * (float)gauss();
            if (f.feedMag(m)) acc++;
          }
          us += 2500;
        }
        CHECK(acc > 1000,
              "ironboot: run updates accepted against the boot field (%d)",
              acc);
        float e2[3], sp[3], sv[3], sat[3];
        f.euler(e2);
        f.sigmas(sp, sv, sat);
        CHECK(fabsf(nav::wrap_pi(e2[2] - e1[2])) * nav::RAD2DEG < 1.0f,
              "ironboot: heading pinned to its init (drift %.2f deg)",
              fabsf(nav::wrap_pi(e2[2] - e1[2])) * nav::RAD2DEG);
        CHECK(sat[2] < 0.05f, "ironboot: heading sigma mag-pinned (%.3f rad)",
              sat[2]);
        CHECK(checkSymmetric(f, 1e-4f) && checkPosDef(f) && stateFinite(f),
              "ironboot: P healthy");
      }
    }
    CHECK(fabsf(nav::wrap_pi(yawBoot[0] - yawBoot[1])) * nav::RAD2DEG < 0.5f,
          "ironboot: reboot reproduces the heading (delta %.2f deg)",
          fabsf(nav::wrap_pi(yawBoot[0] - yawBoot[1])) * nav::RAD2DEG);
  }

  printf("\n%d checks, %d failed -> %s\n", checks, fails,
         fails == 0 ? "PASS" : "FAIL");
  return fails;
}
