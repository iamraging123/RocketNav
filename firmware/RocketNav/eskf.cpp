// eskf.cpp — error-state EKF implementation. Platform-free. See eskf.h and
// DESIGN.md for the derivations; comments here map code blocks to equations.

#include "eskf.h"
#include <string.h>

namespace eskf {

using nav::v3_norm;
using nav::wrap_pi;

// ---------- dense 16x16 helpers (float32, no allocation) ----------

static void matmul(const float A[NX][NX], const float B[NX][NX],
                   float C[NX][NX]) {
  for (int i = 0; i < NX; ++i) {
    for (int j = 0; j < NX; ++j) {
      float s = 0.0f;
      for (int k = 0; k < NX; ++k) s += A[i][k] * B[k][j];
      C[i][j] = s;
    }
  }
}

static void matmul_abt(const float A[NX][NX], const float B[NX][NX],
                       float C[NX][NX]) {
  for (int i = 0; i < NX; ++i) {
    for (int j = 0; j < NX; ++j) {
      float s = 0.0f;
      for (int k = 0; k < NX; ++k) s += A[i][k] * B[j][k];
      C[i][j] = s;
    }
  }
}

static void symmetrize(float P[NX][NX]) {
  for (int i = 0; i < NX; ++i) {
    for (int j = i + 1; j < NX; ++j) {
      float m = 0.5f * (P[i][j] + P[j][i]);
      P[i][j] = m;
      P[j][i] = m;
    }
    // Numerical floor: a zero/negative diagonal is always a numerics bug and
    // would poison the next gain computation.
    if (P[i][i] < 1e-12f) P[i][i] = 1e-12f;
  }
}

// Closed-form inverse of a small symmetric S (m in 1..3). Returns false on a
// non-positive / near-singular determinant.
static bool small_inv(const float S[MMAX][MMAX], int m, float Si[MMAX][MMAX]) {
  if (m == 1) {
    if (S[0][0] < 1e-12f) return false;
    Si[0][0] = 1.0f / S[0][0];
    return true;
  }
  if (m == 2) {
    float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (det < 1e-12f) return false;
    float inv = 1.0f / det;
    Si[0][0] = S[1][1] * inv;
    Si[1][1] = S[0][0] * inv;
    Si[0][1] = -S[0][1] * inv;
    Si[1][0] = -S[1][0] * inv;
    return true;
  }
  float a = S[0][0], b = S[0][1], c = S[0][2];
  float d = S[1][1], e = S[1][2], f = S[2][2];
  float A = d * f - e * e;
  float B = c * e - b * f;
  float C = b * e - c * d;
  float det = a * A + b * B + c * C;
  if (det < 1e-12f) return false;
  float inv = 1.0f / det;
  Si[0][0] = A * inv;
  Si[0][1] = B * inv;
  Si[0][2] = C * inv;
  Si[1][0] = B * inv;
  Si[1][1] = (a * f - c * c) * inv;
  Si[1][2] = (b * c - a * e) * inv;
  Si[2][0] = C * inv;
  Si[2][1] = (b * c - a * e) * inv;
  Si[2][2] = (a * d - b * b) * inv;
  return true;
}

// ---------- lifecycle ----------

void Eskf::init(const Config &c) {
  cfg_ = c;
  state_ = FS_INIT;
  align_degraded_ = false;
  memset(P_, 0, sizeof(P_));
  for (int i = 0; i < NX; ++i) P_[i][i] = 1e-6f;
  for (int i = 0; i < 3; ++i) { p_[i] = 0; v_[i] = 0; bg_[i] = 0; ba_[i] = 0; }
  nav::quat_identity(q_);
  bb_ = 0;
  vd_off_ = 0;
  g_ned_[0] = 0; g_ned_[1] = 0; g_ned_[2] = cfg_.g_default;
  origin_ = nav::GeoOrigin();
  t_last_us_ = 0;
  prop_count_ = 0;
  for (int i = 0; i < CH_COUNT; ++i) innov_[i] = InnovRecord();
  // Reference-field derived constants for the heading construction.
  mag_bh_ref_ = sqrtf(cfg_.mag_ref_ned[0] * cfg_.mag_ref_ned[0] +
                      cfg_.mag_ref_ned[1] * cfg_.mag_ref_ned[1]);
  if (mag_bh_ref_ < 1e-3f) mag_bh_ref_ = 1e-3f;
  mag_beta_ref_ = atan2f(cfg_.mag_ref_ned[1], cfg_.mag_ref_ned[0]);
  mag_bd_ref_ = cfg_.mag_ref_ned[2];
  mag_bnorm_ref_ = v3_norm(cfg_.mag_ref_ned);
  mag_incl_ref_ = atan2f(mag_bd_ref_, mag_bh_ref_);
  mag_ref_run_[0] = cfg_.mag_ref_ned[0];
  mag_ref_run_[1] = cfg_.mag_ref_ned[1];
  mag_ref_run_[2] = cfg_.mag_ref_ned[2];
  mag_bnorm_run_ = mag_bnorm_ref_;
  mag_incl_run_ = mag_incl_ref_;
  still_streak_ = 0;
  grav_decim_ctr_ = 0;
}

// ---------- alignment ----------

void Eskf::startAlign(uint64_t t_us) {
  align_win_us_ = t_us;
  for (int i = 0; i < 3; ++i) {
    asum_[i] = 0; asum2_[i] = 0; gsum_[i] = 0; gsum2_[i] = 0;
  }
  acount_ = 0;
  // The mag mean is restricted to the SAME window that passes the stillness
  // screen. Iron errors don't care about vibration, but the body-frame mean
  // absolutely cares about rotation: samples taken while the vehicle was
  // still being handled (plugged in, positioned) used to survive window
  // restarts and seeded a different heading every boot.
  msum_[0] = msum_[1] = msum_[2] = 0;
  mcount_ = 0;
}

void Eskf::accumAlign(const ImuSample &s) {
  for (int i = 0; i < 3; ++i) {
    asum_[i] += s.accel[i];
    asum2_[i] += s.accel[i] * s.accel[i];
    gsum_[i] += s.gyro[i];
    gsum2_[i] += s.gyro[i] * s.gyro[i];
  }
  acount_++;
}

void Eskf::finishAlign(bool clean) {
  float n = (acount_ > 0) ? (float)acount_ : 1.0f;
  float abar[3], gbar[3];
  for (int i = 0; i < 3; ++i) { abar[i] = asum_[i] / n; gbar[i] = gsum_[i] / n; }

  // Tilt from gravity: stationary specific force f = -R' g, so the body-frame
  // down unit vector is -f/|f|.
  float fn = v3_norm(abar);
  float d_b[3];
  if (fn > 1e-3f) {
    d_b[0] = -abar[0] / fn; d_b[1] = -abar[1] / fn; d_b[2] = -abar[2] / fn;
  } else {
    d_b[0] = 0; d_b[1] = 0; d_b[2] = 1;  // dead accel: assume upright, degraded
    clean = false;
  }
  float sx = d_b[0];
  if (sx > 1.0f) sx = 1.0f;
  if (sx < -1.0f) sx = -1.0f;
  float roll = atan2f(d_b[1], d_b[2]);
  float pitch = -asinf(sx);

  // Heading from the mag mean, tilt-compensated, else zero with wide sigma.
  float yaw = 0.0f;
  float yaw_std = cfg_.init_yaw_std_nomag;
  if (cfg_.mag_mode != MAG_OFF && mcount_ >= 20) {
    float mbar[3] = { msum_[0] / (float)mcount_, msum_[1] / (float)mcount_,
                      msum_[2] / (float)mcount_ };
    float q0[4], R0[9], m_lvl[3];
    nav::quat_from_euler(roll, pitch, 0.0f, q0);
    nav::quat_to_dcm(q0, R0);
    nav::dcm_mul_vec(R0, mbar, m_lvl);
    float bh = sqrtf(m_lvl[0] * m_lvl[0] + m_lvl[1] * m_lvl[1]);
    if (bh > 1.0f) {  // >1 uT horizontal: bearing is meaningful
      yaw = wrap_pi(mag_beta_ref_ - atan2f(m_lvl[1], m_lvl[0]));
      yaw_std = cfg_.init_yaw_std_mag;
      mag_init_used_ = true;
    }
  }
  nav::quat_from_euler(roll, pitch, yaw, q_);

  // Run-time field reference (see the member comment). With a mag-seeded
  // heading the align mean expressed in the just-chosen frame IS the local
  // field; the vector update then holds attitude to this boot-consistent
  // reference instead of fighting toward a site vector the local field may
  // never match - which either froze heading wherever the gates first
  // tripped or left it run-dependent. Same orientation at the next boot
  // reproduces the same mean, the same reference, the same heading.
  if (mag_init_used_) {
    float mbar[3] = { msum_[0] / (float)mcount_, msum_[1] / (float)mcount_,
                      msum_[2] / (float)mcount_ };
    float Rf[9];
    nav::quat_to_dcm(q_, Rf);
    nav::dcm_mul_vec(Rf, mbar, mag_ref_run_);
    mag_bnorm_run_ = v3_norm(mag_ref_run_);
    float bh_run = sqrtf(mag_ref_run_[0] * mag_ref_run_[0] +
                         mag_ref_run_[1] * mag_ref_run_[1]);
    if (bh_run < 1e-3f) bh_run = 1e-3f;
    mag_incl_run_ = atan2f(mag_ref_run_[2], bh_run);
  } else {
    mag_ref_run_[0] = cfg_.mag_ref_ned[0];
    mag_ref_run_[1] = cfg_.mag_ref_ned[1];
    mag_ref_run_[2] = cfg_.mag_ref_ned[2];
    mag_bnorm_run_ = mag_bnorm_ref_;
    mag_incl_run_ = mag_incl_ref_;
  }

  // Gyro bias from the stationary mean. Earth rotation (15 deg/h = 7.3e-5
  // rad/s) is far below this sensor's bias floor and is absorbed here.
  for (int i = 0; i < 3; ++i) { bg_[i] = gbar[i]; ba_[i] = 0; }
  bb_ = 0;

  float degrade = clean ? 1.0f : 3.0f;  // moving vehicle: honest wide init
  align_degraded_ = !clean;
  memset(P_, 0, sizeof(P_));
  float srp = cfg_.init_rp_std * degrade;
  float syaw = yaw_std * degrade;
  float sbg = cfg_.init_bg_std * degrade;
  P_[ITH + 0][ITH + 0] = srp * srp;
  P_[ITH + 1][ITH + 1] = srp * srp;
  P_[ITH + 2][ITH + 2] = syaw * syaw;
  for (int i = 0; i < 3; ++i) {
    P_[IBG + i][IBG + i] = sbg * sbg;
    P_[IBA + i][IBA + i] = cfg_.init_ba_std * cfg_.init_ba_std;
    // Horizontal p/v are meaningless until the origin anchors; keep them
    // huge so nothing downstream mistakes them for information.
    // anchorOrigin() resets them.
    P_[IP + i][IP + i] = 1e6f;
    P_[IV + i][IV + i] = 9e4f;
  }
  // The vertical channel starts now: the stillness that gated this alignment
  // also pins v_D, so seed it tightly (wide when the init was degraded) and
  // let the first baro update define the p_D datum against its huge prior.
  {
    float svd0 = cfg_.init_vel_std_min * (clean ? 1.0f : 10.0f);
    P_[IV + 2][IV + 2] = svd0 * svd0;
  }
  P_[IBB][IBB] = cfg_.init_bb_std * cfg_.init_bb_std;

  state_ = FS_WAIT_FIX;
  still_streak_ = 0;
}

// ---------- IMU path ----------

void Eskf::feedImu(const ImuSample &s) {
  if (state_ == FS_INIT) {
    align_first_us_ = s.t_us;
    mag_init_used_ = false;  // per-boot flag; startAlign clears the window
    startAlign(s.t_us);
    state_ = FS_ALIGN;
    t_last_us_ = s.t_us;
    return;
  }

  if (state_ == FS_ALIGN) {
    accumAlign(s);
    t_last_us_ = s.t_us;
    float win_s = (float)(s.t_us - align_win_us_) * 1e-6f;
    if (win_s < cfg_.align_duration_s) return;
    // Window complete: evaluate stillness (gyro AND accel — |a| alone is
    // blind to horizontal acceleration).
    float n = (float)acount_;
    if (n < 8) { startAlign(s.t_us); return; }
    float gmax = 0, amax = 0, amean[3];
    for (int i = 0; i < 3; ++i) {
      float gm = gsum_[i] / n;
      float gv = gsum2_[i] / n - gm * gm;
      if (gv < 0) gv = 0;
      if (sqrtf(gv) > gmax) gmax = sqrtf(gv);
      float am = asum_[i] / n;
      amean[i] = am;
      float av = asum2_[i] / n - am * am;
      if (av < 0) av = 0;
      if (sqrtf(av) > amax) amax = sqrtf(av);
    }
    float norm_err = fabsf(v3_norm(amean) - cfg_.g_default);
    bool still = (gmax < cfg_.still_gyro_std_max) &&
                 (amax < cfg_.still_accel_std_max) &&
                 (norm_err < cfg_.still_acc_norm_tol);
    if (still) {
      finishAlign(true);
    } else if ((float)(s.t_us - align_first_us_) * 1e-6f >
               cfg_.align_timeout_s) {
      finishAlign(false);  // never went quiet: accept a degraded init
    } else {
      startAlign(s.t_us);  // restart the window and keep waiting
    }
    return;
  }

  // FS_WAIT_FIX / FS_RUN: propagate.
  if (s.t_us <= t_last_us_) return;  // non-monotonic timestamp: drop
  float dt_raw = (float)(s.t_us - t_last_us_) * 1e-6f;
  t_last_us_ = s.t_us;
  float dt = dt_raw;
  float dt_cap = 4.0f * cfg_.imu_dt_nom;
  if (dt > dt_cap) dt = dt_cap;  // integrate a capped step...
  propagate(s, dt);
  if (dt_raw > dt_cap) {
    // ...and account for the lost time as extra process noise (scaled Q for
    // the un-integrated remainder, capped at 100 ms of growth).
    float extra = dt_raw - dt;
    if (extra > 0.1f) extra = 0.1f;
    float sg = cfg_.gyro_nd * cfg_.vib_gyro_mult;
    float sa = cfg_.accel_nd * cfg_.vib_accel_mult;
    for (int i = 0; i < 3; ++i) {
      P_[IV + i][IV + i] += sa * sa * extra;
      P_[ITH + i][ITH + i] += sg * sg * extra;
    }
  }

  updateStillTracker(s, dt);
  if (state_ == FS_WAIT_FIX || state_ == FS_ATT_ONLY || state_ == FS_RUN) {
    // Measured-stillness aiding, in EVERY post-align state (the gate is
    // sensor statistics, not flight-phase logic — in flight it simply
    // never passes): the gravity-vector update pins roll/pitch, and the
    // zero-angular-rate update trims all three gyro biases, so attitude
    // stops drifting on the pad even while the magnetometer is being
    // rejected. Both are decimated so successive samples aren't treated
    // as independent.
    uint32_t need = (uint32_t)(0.2f / cfg_.imu_dt_nom);
    if (still_streak_ > need) {
      if (++grav_decim_ctr_ >= (uint32_t)cfg_.grav_decim) {
        grav_decim_ctr_ = 0;
        float f_b[3] = { s.accel[0] - ba_[0], s.accel[1] - ba_[1],
                         s.accel[2] - ba_[2] };
        gravityUpdate(f_b);
        zaruUpdate(s.gyro);
      }
    }
  }
}

void Eskf::updateStillTracker(const ImuSample &s, float dt) {
  float alpha = dt / 0.25f;  // ~0.25 s EWMA time constant
  if (alpha > 0.5f) alpha = 0.5f;
  float dev2 = 0;
  for (int i = 0; i < 3; ++i) {
    float w = s.gyro[i] - bg_[i];
    still_gmean_[i] += alpha * (w - still_gmean_[i]);
    float d = w - still_gmean_[i];
    dev2 += d * d;
  }
  still_gdev2_ += alpha * (dev2 - still_gdev2_);
  float fn = v3_norm(s.accel);
  still_fnorm_ += alpha * (fn - still_fnorm_);
  bool quiet = (sqrtf(still_gdev2_) < 1.5f * cfg_.still_gyro_std_max) &&
               (fabsf(still_fnorm_ - cfg_.g_default) <
                3.0f * cfg_.still_acc_norm_tol) &&
               !s.sat;
  if (quiet) {
    if (still_streak_ < 0xFFFFFFFF) still_streak_++;
  } else {
    still_streak_ = 0;
  }
}

void Eskf::propagate(const ImuSample &s, float dt) {
  float w_b[3] = { s.gyro[0] - bg_[0], s.gyro[1] - bg_[1], s.gyro[2] - bg_[2] };
  float f_b[3] = { s.accel[0] - ba_[0], s.accel[1] - ba_[1],
                   s.accel[2] - ba_[2] };
  float R[9];
  nav::quat_to_dcm(q_, R);
  float f_n[3];
  nav::dcm_mul_vec(R, f_b, f_n);
  acc_ned_[0] = f_n[0] + g_ned_[0];
  acc_ned_[1] = f_n[1] + g_ned_[1];
  acc_ned_[2] = f_n[2] + g_ned_[2];

  if (state_ == FS_RUN) {
    for (int i = 0; i < 3; ++i) {
      p_[i] += v_[i] * dt + 0.5f * acc_ned_[i] * dt * dt;
      v_[i] += acc_ned_[i] * dt;
    }
  } else if (state_ == FS_WAIT_FIX || state_ == FS_ATT_ONLY) {
    // Baro-damped vertical channel: p_D/v_D are observable without GNSS, so
    // altitude and vertical rate run from the moment alignment completes.
    // Horizontal states stay pinned — nothing aids them before the anchor,
    // and integrating them would only manufacture garbage.
    p_[2] += v_[2] * dt + 0.5f * acc_ned_[2] * dt * dt;
    v_[2] += acc_ned_[2] * dt;
  }
  // Attitude: body-rate quaternion increment on the right (exact exp map).
  float rv[3] = { w_b[0] * dt, w_b[1] * dt, w_b[2] * dt };
  float dq[4], qn[4];
  nav::quat_from_rotvec(rv, dq);
  nav::quat_mul(q_, dq, qn);
  q_[0] = qn[0]; q_[1] = qn[1]; q_[2] = qn[2]; q_[3] = qn[3];
  nav::quat_normalize(q_);

  // --- exact discrete transition matrix ---
  // With the GLOBAL attitude error the continuous-time F has the dependency
  // chain p<-v<-(theta,ba), theta<-bg and no cycles, so F^4 = 0 and
  // Phi = I + F dt + F^2 dt^2/2 + F^3 dt^3/6 is the exact matrix exponential
  // (see DESIGN.md). Blocks: A = -[f_n]x, B = -R.
  float A[9] = { 0, f_n[2], -f_n[1],
                 -f_n[2], 0, f_n[0],
                 f_n[1], -f_n[0], 0 };  // -[f_n]x
  memset(Phi_, 0, sizeof(Phi_));
  for (int i = 0; i < NX; ++i) Phi_[i][i] = 1.0f;
  float dt2h = 0.5f * dt * dt;
  float dt3s = dt * dt * dt / 6.0f;
  float AB[9];  // A*B = [f_n]x * R  (since (-[f_n]x)(-R))
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      float sab = 0;
      for (int k = 0; k < 3; ++k) sab += A[i * 3 + k] * (-R[k * 3 + j]);
      AB[i * 3 + j] = sab;
      Phi_[IV + i][ITH + j] = A[i * 3 + j] * dt;
      Phi_[IV + i][IBA + j] = -R[i * 3 + j] * dt;
      Phi_[ITH + i][IBG + j] = -R[i * 3 + j] * dt;
      Phi_[IP + i][ITH + j] = A[i * 3 + j] * dt2h;
      Phi_[IP + i][IBA + j] = -R[i * 3 + j] * dt2h;
    }
    Phi_[IP + i][IV + i] = dt;
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Phi_[IV + i][IBG + j] = AB[i * 3 + j] * dt2h;
      Phi_[IP + i][IBG + j] = AB[i * 3 + j] * dt3s;
    }
  }

  // P <- Phi P Phi' + Qd
  matmul(Phi_, P_, T1_);
  matmul_abt(T1_, Phi_, P_);

  float sg = cfg_.gyro_nd * cfg_.vib_gyro_mult;
  float sa = cfg_.accel_nd * cfg_.vib_accel_mult;
  if (s.sat) sa *= cfg_.sat_q_mult;  // saturated accel: trust it far less
  float qv = sa * sa * dt;
  float qth = sg * sg * dt;
  float qbg = cfg_.gyro_bias_rw * cfg_.gyro_bias_rw * dt;
  float qba = cfg_.accel_bias_rw * cfg_.accel_bias_rw * dt;
  float qp = cfg_.pos_q_floor * cfg_.pos_q_floor * dt;
  for (int i = 0; i < 3; ++i) {
    P_[IP + i][IP + i] += qp;
    P_[IV + i][IV + i] += qv;
    P_[ITH + i][ITH + i] += qth;
    P_[IBG + i][IBG + i] += qbg;
    P_[IBA + i][IBA + i] += qba;
  }
  P_[IBB][IBB] += cfg_.baro_bias_rw * cfg_.baro_bias_rw * dt;
  symmetrize(P_);
  // Unaided p/v covariance grows without bound (horizontal channels before
  // the anchor; everything through a long GNSS outage). Cap by scaling the
  // whole row and column: P <- D P D with diagonal D stays PSD, unlike
  // clamping the diagonal entry alone. The caps sit far above any
  // aided-operation variance — numerics, not tuning.
  for (int i = 0; i < 3; ++i) {
    capState(IP + i, 1.0e7f);  // sigma ~3.2 km
    capState(IV + i, 9.0e4f);  // sigma 300 m/s
  }
  // Attitude variance floor: mount alignment, reference-vector and
  // calibration errors are SYSTEMATIC — dense vector aiding must not
  // average them into impossible certainty. Raising a diagonal keeps P
  // PSD, and the floor also keeps float32 Cholesky margins sane next to
  // the huge unaided p/v states.
  for (int i = 0; i < 3; ++i) {
    if (P_[ITH + i][ITH + i] < cfg_.att_var_floor) {
      P_[ITH + i][ITH + i] = cfg_.att_var_floor;
    }
  }
  prop_count_++;
}

void Eskf::capState(int idx, float cap) {
  float d = P_[idx][idx];
  if (d <= cap) return;
  float s = sqrtf(cap / d);
  for (int r = 0; r < NX; ++r) { P_[r][idx] *= s; P_[idx][r] *= s; }
  P_[idx][idx] = cap;  // the row+col pass scaled it by s^2 exactly
}

// ---------- generic Joseph-form update ----------

void Eskf::recordReject(Chan ch, const float *nu, int m, float nis,
                        bool nis_valid, uint64_t t_us) {
  InnovRecord &r = innov_[ch];
  for (int i = 0; i < 3; ++i) r.nu[i] = (i < m) ? nu[i] : 0.0f;
  r.nis = nis;
  r.nis_valid = nis_valid;
  r.accepted = false;
  r.valid = true;
  r.t_us = t_us;
  r.count++;
}

bool Eskf::measUpdate(const float H[MMAX][NX], const float *nu,
                      const float *rdiag, int m, float gate, Chan ch,
                      uint64_t t_us) {
  // PHt = P H'
  float PHt[NX][MMAX];
  for (int i = 0; i < NX; ++i) {
    for (int j = 0; j < m; ++j) {
      float s = 0;
      for (int k = 0; k < NX; ++k) s += P_[i][k] * H[j][k];
      PHt[i][j] = s;
    }
  }
  // S = H PHt + R
  float S[MMAX][MMAX];
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < m; ++j) {
      float s = 0;
      for (int k = 0; k < NX; ++k) s += H[i][k] * PHt[k][j];
      S[i][j] = s + ((i == j) ? rdiag[i] : 0.0f);
    }
  }
  float Si[MMAX][MMAX];
  if (!small_inv(S, m, Si)) {
    recordReject(ch, nu, m, 0.0f, false, t_us);
    return false;
  }
  // NIS = nu' S^-1 nu, chi-square gate.
  float nis = 0;
  for (int i = 0; i < m; ++i) {
    float s = 0;
    for (int j = 0; j < m; ++j) s += Si[i][j] * nu[j];
    nis += nu[i] * s;
  }
  InnovRecord &r = innov_[ch];
  for (int i = 0; i < 3; ++i) r.nu[i] = (i < m) ? nu[i] : 0.0f;
  r.nis = nis;
  r.nis_valid = true;
  r.t_us = t_us;
  r.count++;
  r.valid = true;
  if (nis > gate) {
    r.accepted = false;
    return false;
  }

  // K = PHt S^-1
  float K[NX][MMAX];
  for (int i = 0; i < NX; ++i) {
    for (int j = 0; j < m; ++j) {
      float s = 0;
      for (int k = 0; k < m; ++k) s += PHt[i][k] * Si[k][j];
      K[i][j] = s;
    }
  }
  float dx[NX];
  for (int i = 0; i < NX; ++i) {
    float s = 0;
    for (int j = 0; j < m; ++j) s += K[i][j] * nu[j];
    dx[i] = s;
  }
  // Joseph form: P <- (I-KH) P (I-KH)' + K R K'
  for (int i = 0; i < NX; ++i) {
    for (int j = 0; j < NX; ++j) {
      float s = 0;
      for (int k = 0; k < m; ++k) s += K[i][k] * H[k][j];
      T2_[i][j] = ((i == j) ? 1.0f : 0.0f) - s;
    }
  }
  matmul(T2_, P_, T1_);
  matmul_abt(T1_, T2_, P_);
  for (int i = 0; i < NX; ++i) {
    for (int j = 0; j < NX; ++j) {
      float s = 0;
      for (int k = 0; k < m; ++k) s += K[i][k] * rdiag[k] * K[j][k];
      P_[i][j] += s;
    }
  }
  symmetrize(P_);
  inject(dx);
  r.accepted = true;
  return true;
}

void Eskf::inject(const float dx[NX]) {
  for (int i = 0; i < 3; ++i) {
    p_[i] += dx[IP + i];
    v_[i] += dx[IV + i];
    bg_[i] += dx[IBG + i];
    ba_[i] += dx[IBA + i];
  }
  bb_ += dx[IBB];
  // Global error: q <- dq(dtheta) (x) q  (left multiply).
  float dth[3] = { dx[ITH], dx[ITH + 1], dx[ITH + 2] };
  float dq[4], qn[4];
  nav::quat_from_rotvec(dth, dq);
  nav::quat_mul(dq, q_, qn);
  q_[0] = qn[0]; q_[1] = qn[1]; q_[2] = qn[2]; q_[3] = qn[3];
  nav::quat_normalize(q_);
  // Covariance reset Jacobian for the attitude block:
  // G_theta = I + 0.5 [dtheta]x  (left/global convention; see DESIGN.md).
  float G[9] = { 1.0f, -0.5f * dth[2], 0.5f * dth[1],
                 0.5f * dth[2], 1.0f, -0.5f * dth[0],
                 -0.5f * dth[1], 0.5f * dth[0], 1.0f };
  // P <- Gblk P Gblk' applied to attitude rows then columns.
  for (int j = 0; j < NX; ++j) {
    float a = P_[ITH][j], b = P_[ITH + 1][j], c = P_[ITH + 2][j];
    P_[ITH][j] = G[0] * a + G[1] * b + G[2] * c;
    P_[ITH + 1][j] = G[3] * a + G[4] * b + G[5] * c;
    P_[ITH + 2][j] = G[6] * a + G[7] * b + G[8] * c;
  }
  for (int i = 0; i < NX; ++i) {
    float a = P_[i][ITH], b = P_[i][ITH + 1], c = P_[i][ITH + 2];
    P_[i][ITH] = G[0] * a + G[1] * b + G[2] * c;
    P_[i][ITH + 1] = G[3] * a + G[4] * b + G[5] * c;
    P_[i][ITH + 2] = G[6] * a + G[7] * b + G[8] * c;
  }
  symmetrize(P_);
}

// ---------- measurement models ----------

void Eskf::setAttitudeOnly() {
  if (state_ == FS_WAIT_FIX) state_ = FS_ATT_ONLY;
}

void Eskf::anchorOrigin(int32_t lat1e7, int32_t lon1e7, int32_t hae_mm,
                        int32_t hmsl_mm, const float vel_ned[3], float hacc_m,
                        float vacc_m, float sacc_ms) {
  // FS_ATT_ONLY also accepts an anchor: p/v/bb state and covariance are
  // rebuilt from scratch below, so a fix arriving after the GPS-less
  // decision upgrades to full navigation exactly as if it were on time.
  if (state_ != FS_WAIT_FIX && state_ != FS_ATT_ONLY) return;
  nav::geo_origin_init(origin_, lat1e7, lon1e7, hae_mm, hmsl_mm);
  g_ned_[2] = origin_.gravity;  // site gravity replaces the default constant
  // The vertical is baro+IMU territory: the anchor initializes ONLY the
  // horizontal states. p_D/v_D/bb continue untouched in the pad/baro datum
  // they have carried since alignment — altitude is continuous through the
  // anchor by construction — and vd_off_ maps that datum into the origin
  // frame for geodetic output (pad-consistent for on-time and late anchors
  // alike: origin MSL - (p_D - vd_off_)).
  vd_off_ = p_[2];
  p_[0] = 0; p_[1] = 0;
  v_[0] = vel_ned[0]; v_[1] = vel_ned[1];
  // Reset the horizontal p/v rows+columns: covariance built pre-anchor for
  // these states (and their cross terms) carries no information.
  for (int r = 0; r < NX; ++r) {
    for (int i = 0; i < 2; ++i) {
      P_[IP + i][r] = 0; P_[r][IP + i] = 0;
      P_[IV + i][r] = 0; P_[r][IV + i] = 0;
    }
  }
  float sph = hacc_m > cfg_.init_pos_std_min ? hacc_m : cfg_.init_pos_std_min;
  float sv = sacc_ms > cfg_.init_vel_std_min ? sacc_ms : cfg_.init_vel_std_min;
  (void)vacc_m;   // GNSS vertical accuracy is irrelevant here by design
  P_[IP][IP] = sph * sph;
  P_[IP + 1][IP + 1] = sph * sph;
  P_[IV][IV] = sv * sv;
  P_[IV + 1][IV + 1] = sv * sv;
  state_ = FS_RUN;
}

bool Eskf::updateGpsPos(int32_t lat1e7, int32_t lon1e7, int32_t hae_mm,
                        float hacc_m, float vacc_m, bool reacq_inflate) {
  if (state_ != FS_RUN || !origin_.valid) return false;
  float z[3];
  nav::geo_to_ned(origin_, lat1e7, lon1e7, hae_mm, z);
  // The fix was computed gps_latency_s before the I2C read that timestamped
  // it; compare against the state back-propagated by that latency.
  float tau = cfg_.gps_latency_s;
  // Horizontal only: the vertical channel is baro+IMU exclusively, so GNSS
  // altitude is never fused (it is still logged raw as galt/gvac).
  (void)vacc_m;
  float nu[2], H[MMAX][NX], rdiag[2];
  memset(H, 0, sizeof(H));
  for (int i = 0; i < 2; ++i) {
    nu[i] = z[i] - (p_[i] - v_[i] * tau);
    H[i][IP + i] = 1.0f;
  }
  float sh = hacc_m > cfg_.gps_pos_r_floor ? hacc_m : cfg_.gps_pos_r_floor;
  float mult = reacq_inflate ? cfg_.reacq_r_mult : 1.0f;
  rdiag[0] = sh * sh * mult;
  rdiag[1] = sh * sh * mult;
  return measUpdate(H, nu, rdiag, 2, cfg_.gate_gps_pos, CH_GPS_POS, t_last_us_);
}

bool Eskf::updateGpsVel(const float vel_ned[3], float sacc_ms,
                        bool reacq_inflate) {
  if (state_ != FS_RUN) return false;
  float tau = cfg_.gps_latency_s;
  float nu[2], H[MMAX][NX], rdiag[2];
  memset(H, 0, sizeof(H));
  for (int i = 0; i < 2; ++i) {
    nu[i] = vel_ned[i] - (v_[i] - acc_ned_[i] * tau);
    H[i][IV + i] = 1.0f;
  }
  float sv = sacc_ms > cfg_.gps_vel_r_floor ? sacc_ms : cfg_.gps_vel_r_floor;
  float mult = reacq_inflate ? cfg_.reacq_r_mult : 1.0f;
  rdiag[0] = rdiag[1] = sv * sv * mult;
  return measUpdate(H, nu, rdiag, 2, cfg_.gate_gps_vel, CH_GPS_VEL, t_last_us_);
}

float Eskf::relAltStd() const {
  float var = P_[IP + 2][IP + 2] + P_[IBB][IBB] - 2.0f * P_[IP + 2][IBB];
  return (var > 0) ? sqrtf(var) : 0.0f;
}

bool Eskf::updateBaro(float alt_rel_m, float temp_c) {
  if (state_ != FS_RUN && state_ != FS_WAIT_FIX && state_ != FS_ATT_ONLY)
    return false;
  // h_baro = -p_D + bb  ->  H = [0 0 -1 | 0... | +1]
  // The sample is baro_latency_s old (conversion window + register hold);
  // compare against the state back-propagated by that latency, mirroring
  // the GNSS latency compensation.
  float tau = cfg_.baro_latency_s;
  float nu[1] = { alt_rel_m - (-(p_[2] - v_[2] * tau) + bb_) };
  float H[MMAX][NX];
  memset(H, 0, sizeof(H));
  H[0][IP + 2] = -1.0f;
  H[0][IBB] = 1.0f;
  // R inflation from the estimated state (never flight-phase logic):
  // dynamic-pressure port error grows with speed^2, and a smooth transonic
  // bump covers the shock-driven static-port excursion around Mach 1.
  float spd = v3_norm(v_);
  float a_snd = sqrtf(1.4f * 287.05f * (temp_c + 273.15f));
  if (a_snd < 100.0f) a_snd = 340.0f;  // nonsense temperature: ISA fallback
  float mach = spd / a_snd;
  float dm = (mach - cfg_.transonic_m0) / cfg_.transonic_sigma_m;
  float rvar = cfg_.baro_r_base * cfg_.baro_r_base;
  float dyn = cfg_.baro_r_dyn_k * spd * spd;
  rvar += dyn * dyn;
  rvar += cfg_.transonic_var * expf(-0.5f * dm * dm);
  float rdiag[1] = { rvar };
  return measUpdate(H, nu, rdiag, 1, cfg_.gate_baro, CH_BARO, t_last_us_);
}

bool Eskf::feedMag(const float m_ut[3]) {
  switch (state_) {
    case FS_ALIGN:
      if (cfg_.mag_mode != MAG_OFF) {
        // Field-magnitude gate on the INIT average too. Ungated, a desk's
        // iron-distorted field seeded a different heading every boot and
        // the continuous gates then froze it there; distorted samples now
        // stay out, and a mostly-rejected window falls back to the honest
        // wide no-mag prior instead.
        float n = v3_norm(m_ut);
        if (fabsf(n - mag_bnorm_ref_) <=
            cfg_.mag_norm_tol_frac * mag_bnorm_ref_) {
          msum_[0] += m_ut[0]; msum_[1] += m_ut[1]; msum_[2] += m_ut[2];
          mcount_++;
        }
        return true;
      }
      return false;
    case FS_WAIT_FIX:
    case FS_ATT_ONLY:
      // Mag refinement before anchor counts as initialization, so it
      // runs for MAG_INIT_ONLY as well as MAG_CONTINUOUS — and attitude-only
      // operation is treated as perpetual initialization: without mag
      // aiding, attitude would drift on gyro bias alone.
      if (cfg_.mag_mode != MAG_OFF) return magVectorUpdate(m_ut);
      return false;
    case FS_RUN:
      if (cfg_.mag_mode == MAG_CONTINUOUS) return magVectorUpdate(m_ut);
      return false;
    default:
      return false;
  }
}

bool Eskf::magVectorUpdate(const float m_b[3]) {
  uint64_t t = t_last_us_;
  // Field-magnitude gate against the run reference (boot-captured at a
  // mag-seeded alignment, the configured site field otherwise): hard/soft
  // iron transients (motors, wiring) show up as magnitude anomalies first.
  float bn = v3_norm(m_b);
  float nu[3] = { 0, 0, 0 };
  if (fabsf(bn - mag_bnorm_run_) > cfg_.mag_norm_tol_frac * mag_bnorm_run_) {
    recordReject(CH_MAG, nu, 3, 0.0f, false, t);
    return false;
  }
  float R[9], m_n[3];
  nav::quat_to_dcm(q_, R);
  nav::dcm_mul_vec(R, m_b, m_n);
  float bh = sqrtf(m_n[0] * m_n[0] + m_n[1] * m_n[1]);
  if (bh < 1.0f) {  // degenerate horizontal projection
    recordReject(CH_MAG, nu, 3, 0.0f, false, t);
    return false;
  }
  // Inclination gate: fires on field anomalies and gross attitude error,
  // and is insensitive to rotation about Down.
  float incl = atan2f(m_n[2], bh);
  if (fabsf(incl - mag_incl_run_) > cfg_.mag_incl_tol_rad) {
    recordReject(CH_MAG, nu, 3, 0.0f, false, t);
    return false;
  }
  // FULL 3-axis vector update: nu = R m_b - B_ref. With the GLOBAL attitude
  // error, R m_b = (I - [dtheta]x) B_ref, so nu = [B_ref]x dtheta + n and
  // H = [B_ref]x on dtheta — constant, rank 2: it constrains every attitude
  // axis EXCEPT rotation about the field line (which it leaves untouched,
  // exactly like the gravity update leaves rotation about Down untouched).
  // This is what pins roll/pitch while the vehicle is handled: the
  // stillness-gated gravity/ZARU aids are off during motion, but a stable
  // 3-axis field is not. The scalar heading information is contained in it.
  nu[0] = m_n[0] - mag_ref_run_[0];
  nu[1] = m_n[1] - mag_ref_run_[1];
  nu[2] = m_n[2] - mag_ref_run_[2];
  const float *B = mag_ref_run_;
  float H[MMAX][NX];
  memset(H, 0, sizeof(H));
  H[0][ITH + 1] = -B[2]; H[0][ITH + 2] = B[1];
  H[1][ITH + 0] = B[2];  H[1][ITH + 2] = -B[0];
  H[2][ITH + 0] = -B[1]; H[2][ITH + 1] = B[0];
  float rr = cfg_.mag_noise_ut * cfg_.mag_noise_ut;
  float rdiag[3] = { rr, rr, rr };
  return measUpdate(H, nu, rdiag, 3, cfg_.gate_mag, CH_MAG, t);
}

void Eskf::gravityUpdate(const float f_b[3]) {
  float fn = v3_norm(f_b);
  if (fn < 1.0f) return;
  float d_b[3] = { -f_b[0] / fn, -f_b[1] / fn, -f_b[2] / fn };
  float R[9], d_n[3];
  nav::quat_to_dcm(q_, R);
  nav::dcm_mul_vec(R, d_b, d_n);
  // z = R d_b - e_D = [e_D]x dtheta + n  (N and E rows only: rank-2 tilt
  // observation that cannot touch heading).
  float nu[2] = { d_n[0], d_n[1] };
  float H[MMAX][NX];
  memset(H, 0, sizeof(H));
  H[0][ITH + 1] = -1.0f;  // z_N = -dtheta_E
  H[1][ITH + 0] = 1.0f;   // z_E = +dtheta_N
  float s = cfg_.grav_meas_std;
  float rdiag[2] = { s * s, s * s };
  measUpdate(H, nu, rdiag, 2, cfg_.gate_grav, CH_GRAV, t_last_us_);
}

void Eskf::zaruUpdate(const float w_raw[3]) {
  // Zero-angular-rate update: a measured-still vehicle's gyro reads exactly
  // its bias, so z = w_raw with prediction bg and H = I on the bias block.
  // The chi-square gate is the windup protection: a slow REAL rotation that
  // sneaks under the stillness screen produces a large normalized
  // innovation once P_bg has converged, and is rejected instead of being
  // absorbed into the bias.
  float nu[3] = { w_raw[0] - bg_[0], w_raw[1] - bg_[1], w_raw[2] - bg_[2] };
  float H[MMAX][NX];
  memset(H, 0, sizeof(H));
  H[0][IBG + 0] = 1.0f;
  H[1][IBG + 1] = 1.0f;
  H[2][IBG + 2] = 1.0f;
  float rr = cfg_.zaru_meas_std * cfg_.zaru_meas_std;
  float rdiag[3] = { rr, rr, rr };
  measUpdate(H, nu, rdiag, 3, cfg_.gate_zaru, CH_ZARU, t_last_us_);
}

// ---------- getters ----------

void Eskf::sigmas(float sp[3], float sv[3], float sa_rad[3]) const {
  for (int i = 0; i < 3; ++i) {
    float a = P_[IP + i][IP + i];
    float b = P_[IV + i][IV + i];
    float c = P_[ITH + i][ITH + i];
    sp[i] = a > 0 ? sqrtf(a) : 0;
    sv[i] = b > 0 ? sqrtf(b) : 0;
    sa_rad[i] = c > 0 ? sqrtf(c) : 0;
  }
}

bool Eskf::geodetic(double *lat_deg, double *lon_deg, double *alt_msl) const {
  if (!origin_.valid) return false;
  // p_D lives in the pad/baro datum; vd_off_ (p_D at anchor) shifts it into
  // the origin frame so MSL output is pad-consistent for on-time and late
  // anchors alike.
  const float pn[3] = { p_[0], p_[1], p_[2] - vd_off_ };
  nav::ned_to_geo(origin_, pn, lat_deg, lon_deg, alt_msl);
  return true;
}

}  // namespace eskf
