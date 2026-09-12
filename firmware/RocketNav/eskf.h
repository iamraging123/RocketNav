// eskf.h — error-state (indirect) extended Kalman filter for GNSS/IMU/mag/baro
// navigation. Platform-free (no Arduino dependencies) so it compiles both for
// the STM32F722 target and as a native host binary (test/eskf_test.cpp).
//
// Architecture (fixed design decisions):
//   * Nominal state: position p (NED, m), velocity v (NED, m/s), attitude
//     quaternion q (body->NED, scalar-first), gyro bias bg (rad/s, body),
//     accel bias ba (m/s^2, body), barometric altitude bias bb (m).
//     GNSS observes the horizontal ONLY: the vertical channel (p_D, v_D)
//     is baro+IMU exclusively in every state, so the baro defines the
//     vertical datum and bb is pinned near zero (retained in the state
//     vector for layout stability, not estimated).
//   * Error state (16): [dp(3) dv(3) dtheta(3) dbg(3) dba(3) dbb(1)].
//     dtheta is a GLOBAL (NED-frame) rotation-vector error:
//     q_true = dq(dtheta) (x) q_hat. The global-error choice gives the
//     3-axis magnetometer vector update a CONSTANT H = [B_ref]x (rank 2:
//     every attitude axis except rotation about the field line), and it
//     makes F nilpotent (F^4 = 0) so the discrete transition matrix
//     Phi = I + F dt + F^2 dt^2/2 + F^3 dt^3/6 is the EXACT exponential.
//   * IMU mechanization propagates the nominal state; measurement updates
//     estimate the error state, which is injected and reset immediately
//     (with the reset Jacobian applied to the attitude covariance block).
//   * float32 throughout, Joseph-form covariance update, explicit
//     symmetrization after every update, no dynamic allocation ever.

#pragma once
#include <stdint.h>
#include "nav_frames.h"

namespace eskf {

constexpr int NX = 16;         // error-state dimension
constexpr int MMAX = 3;        // max measurement dimension per update

// Error-state index map.
constexpr int IP = 0;    // position
constexpr int IV = 3;    // velocity
constexpr int ITH = 6;   // attitude (global rotation vector)
constexpr int IBG = 9;   // gyro bias
constexpr int IBA = 12;  // accel bias
constexpr int IBB = 15;  // baro bias

enum MagMode : uint8_t { MAG_OFF = 0, MAG_INIT_ONLY = 1, MAG_CONTINUOUS = 2 };

// FS_INIT: waiting for the first IMU sample.
// FS_ALIGN: accumulating the static-alignment window.
// FS_WAIT_FIX: attitude/bias initialized, propagating attitude and applying
//   stillness-gated gravity (and mag) refinements while waiting for the first
//   valid GNSS fix. The baro-damped vertical channel (p_D, v_D) mechanizes
//   and takes updates here too — altitude never waits for GNSS. Handles both
//   "vehicle not static" (window restarts, degraded fallback) and "fix not
//   yet available".
// FS_RUN: origin anchored, full navigation.
// FS_ATT_ONLY: deliberate GPS-less operation (entered by command from the
//   application's startup GPS-mode determination): identical machinery to
//   FS_WAIT_FIX — attitude propagation with mag heading, stillness-gated
//   gravity aiding, and the baro-damped vertical channel — but as an
//   operating mode, not a wait. Horizontal position/velocity stay undefined
//   unless a valid fix arrives later, in which case anchorOrigin upgrades to
//   FS_RUN (p/v are rebuilt from the fix and bb is re-seeded to preserve the
//   vertical channel's altitude, so a late anchor is identical to an
//   on-time one).
enum FiltState : uint8_t { FS_INIT = 0, FS_ALIGN = 1, FS_WAIT_FIX = 2,
                           FS_RUN = 3, FS_ATT_ONLY = 4 };

// Measurement channels (for innovation/NIS reporting).
enum Chan : uint8_t { CH_GPS_POS = 0, CH_GPS_VEL = 1, CH_BARO = 2, CH_MAG = 3,
                      CH_GRAV = 4, CH_ZARU = 5, CH_COUNT = 6 };

struct Config {
  // --- continuous-time noise densities (see config block for datasheet
  //     provenance; the filter only consumes these numbers) ---
  float gyro_nd;        // rad/s/sqrt(Hz)  gyro rate noise density
  float accel_nd;       // m/s^2/sqrt(Hz)  accel noise density
  float gyro_bias_rw;   // rad/s/sqrt(s)   gyro bias random walk
  float accel_bias_rw;  // m/s^2/sqrt(s)   accel bias random walk
  float baro_bias_rw;   // m/sqrt(s)       baro bias random walk
  float vib_gyro_mult;  // in-flight vibration inflation on gyro_nd
  float vib_accel_mult; // in-flight vibration inflation on accel_nd
  float sat_q_mult;     // extra accel-noise multiplier while accel saturated
  float pos_q_floor;    // m/sqrt(s) tiny direct position noise (numerics)

  // --- initial standard deviations ---
  float init_pos_std_min;   // m, floor under GNSS-reported hAcc/vAcc
  float init_vel_std_min;   // m/s, floor under sAcc
  float init_rp_std;        // rad, roll/pitch after static alignment
  float init_yaw_std_mag;   // rad, heading when magnetometer used
  float init_yaw_std_nomag; // rad, heading with MAG_OFF (large)
  float init_bg_std;        // rad/s
  float init_ba_std;        // m/s^2
  float init_bb_std;        // m

  // --- static alignment ---
  float align_duration_s;    // accumulation window
  float align_timeout_s;     // give up restarting and accept a degraded init
  float still_gyro_std_max;  // rad/s, per-axis stillness limit
  float still_accel_std_max; // m/s^2, per-axis stillness limit
  float still_acc_norm_tol;  // m/s^2, | |f| - g | limit
  float imu_dt_nom;          // s, nominal IMU sample interval

  // --- innovation gates (chi-square thresholds) ---
  float gate_gps_pos;  // 2 dof (N/E only — GNSS never measures the vertical)
  float gate_gps_vel;  // 2 dof
  float gate_baro;     // 1 dof
  float gate_mag;      // 1 dof
  float gate_grav;     // 2 dof

  // --- GNSS ---
  float gps_pos_r_floor;  // m, floor under reported hAcc/vAcc
  float gps_vel_r_floor;  // m/s, floor under reported sAcc
  float gps_latency_s;    // fix age at I2C read time (compensated explicitly)
  float reacq_r_mult;     // R inflation for the first fixes after reacquisition

  // --- barometer ---
  float baro_r_base;       // m, 1-sigma at rest
  float baro_r_dyn_k;      // m per (m/s)^2, dynamic-pressure port error
  float transonic_var;     // m^2, added variance at the transonic peak
  float transonic_m0;      // Mach center of the transonic bump
  float transonic_sigma_m; // Mach width of the transonic bump
  float baro_latency_s;    // s, sample age at read time (conversion window
                           // + register hold), compensated like GNSS latency

  // --- magnetometer ---
  MagMode mag_mode;
  float mag_ref_ned[3];    // uT, local field NED reference vector
  float mag_noise_ut;      // uT, effective sensor+cal noise
  float mag_r_floor;       // retained for config layout; unused by the
                           // 3-axis vector update
  float mag_norm_tol_frac; // fractional field-magnitude gate
  float mag_incl_tol_rad;  // inclination gate half-width

  // --- stillness-gated refinements (every post-align state; the gate is
  //     measured sensor statistics, so in flight they simply never fire) ---
  float grav_meas_std;     // rad, gravity tilt measurement 1-sigma
  int   grav_decim;        // apply every Nth IMU sample while still
  float zaru_meas_std;     // rad/s, zero-angular-rate bias-trim 1-sigma
  float gate_zaru;         // 3 dof chi-square gate (windup protection)
  float att_var_floor;     // rad^2 floor on attitude variance: mount/
                           // reference systematics never average down

  float g_default;         // m/s^2 used before the origin (and its gravity)
};

struct ImuSample {
  uint64_t t_us;    // monotonic acquisition timestamp
  float gyro[3];    // rad/s, body frame, calibrated
  float accel[3];   // m/s^2, body frame, calibrated (specific force)
  bool sat;         // any accel axis at full scale this sample
};

struct InnovRecord {
  float nu[3] = {0, 0, 0};  // innovation (heading channel: rad in nu[0])
  float nis = 0;            // normalized innovation squared
  bool accepted = false;
  bool valid = false;       // at least one update attempted
  bool nis_valid = false;   // false when rejected before NIS (field gates)
  uint64_t t_us = 0;
  uint32_t count = 0;       // update attempts
};

class Eskf {
 public:
  Eskf() {}

  void init(const Config &c);

  // Feed every fresh calibrated IMU sample. Drives alignment, then
  // propagation. Never blocks, never allocates.
  void feedImu(const ImuSample &s);

  // Feed every fresh calibrated magnetometer sample (uT, body frame).
  // Routed by state: alignment accumulation, or the full 3-axis vector
  // update (nu = R m_b - B_ref, H = [B_ref]x) — rank 2, so it constrains
  // roll/pitch/heading about every axis except the field line, including
  // while the vehicle is being handled and the stillness-gated aids are
  // off. Returns true if consumed.
  bool feedMag(const float m_ut[3]);

  // Enter attitude-only (GPS-less) operation. Only valid from FS_WAIT_FIX.
  // A later anchorOrigin still upgrades to FS_RUN (auto-mode late fix).
  void setAttitudeOnly();

  // Anchor the NED origin at the first valid fix; initializes the
  // HORIZONTAL p/v and their covariance — the baro+IMU vertical channel
  // continues untouched, so altitude is continuous through the anchor —
  // and transitions to FS_RUN. Accepted from FS_WAIT_FIX and FS_ATT_ONLY
  // (late-fix upgrade); a no-op in every other state.
  void anchorOrigin(int32_t lat1e7, int32_t lon1e7, int32_t hae_mm,
                    int32_t hmsl_mm, const float vel_ned[3],
                    float hacc_m, float vacc_m, float sacc_ms);

  // Measurement updates (return true when accepted; GNSS require FS_RUN).
  // GNSS position/velocity observe N/E only: the vertical is baro+IMU
  // exclusively, GNSS altitude is logged raw but never fused.
  // reacq_inflate: caller-tracked flag for the first fixes after an outage.
  bool updateGpsPos(int32_t lat1e7, int32_t lon1e7, int32_t hae_mm,
                    float hacc_m, float vacc_m, bool reacq_inflate);
  bool updateGpsVel(const float vel_ned[3], float sacc_ms, bool reacq_inflate);
  // alt_rel_m: baro altitude relative to the alignment reference (m, up).
  // temp_c: static air temperature for the speed-of-sound estimate.
  // Accepted from FS_WAIT_FIX and FS_ATT_ONLY as well as FS_RUN — the
  // vertical channel runs from the moment alignment completes.
  bool updateBaro(float alt_rel_m, float temp_c);

  // --- getters ---
  FiltState state() const { return state_; }
  bool alignDegraded() const { return align_degraded_; }
  // True when the initial heading came from the magnetometer (enough
  // samples passed the field-magnitude gate during alignment). False =
  // heading started at 0 with the wide no-mag prior.
  bool magInitUsed() const { return mag_init_used_; }
  const float *pos() const { return p_; }
  const float *vel() const { return v_; }
  const float *quat() const { return q_; }
  const float *gyroBias() const { return bg_; }
  const float *accelBias() const { return ba_; }
  float baroBias() const { return bb_; }
  // Vertical-channel altitude above the pad baro reference and its 1-sigma.
  // -p_D + bb is the exact combination the baro measures — the altitude
  // output IS the baro+IMU fusion. The anchor never touches the vertical
  // states, so it is continuous across the anchor by construction.
  // Meaningful from FS_WAIT_FIX on.
  float relAlt() const { return -p_[2] + bb_; }
  float relAltStd() const;
  void euler(float e[3]) const { nav::quat_to_euler(q_, e); }
  void sigmas(float sp[3], float sv[3], float sa_rad[3]) const;
  bool geodetic(double *lat_deg, double *lon_deg, double *alt_msl) const;
  const nav::GeoOrigin &origin() const { return origin_; }
  const InnovRecord &innov(Chan c) const { return innov_[c]; }
  uint32_t propCount() const { return prop_count_; }
  float Pat(int r, int c) const { return P_[r][c]; }  // test/diagnostic access
  uint64_t lastImuTime() const { return t_last_us_; }
  // Navigation-frame specific force + gravity from the last propagation.
  const float *lastAccNed() const { return acc_ned_; }

 private:
  // --- internals ---
  void startAlign(uint64_t t_us);
  void accumAlign(const ImuSample &s);
  void finishAlign(bool clean);
  void propagate(const ImuSample &s, float dt);
  bool measUpdate(const float H[MMAX][NX], const float *nu, const float *rdiag,
                  int m, float gate, Chan ch, uint64_t t_us);
  void inject(const float dx[NX]);
  void recordReject(Chan ch, const float *nu, int m, float nis, bool nis_valid,
                    uint64_t t_us);
  bool magVectorUpdate(const float m_ut[3]);
  void gravityUpdate(const float f_b[3]);
  void zaruUpdate(const float w_raw[3]);
  void updateStillTracker(const ImuSample &s, float dt);
  void capState(int idx, float cap);

  Config cfg_{};
  FiltState state_ = FS_INIT;
  bool align_degraded_ = false;

  // Nominal state.
  float p_[3] = {0, 0, 0};
  float v_[3] = {0, 0, 0};
  float q_[4] = {1, 0, 0, 0};
  float bg_[3] = {0, 0, 0};
  float ba_[3] = {0, 0, 0};
  float bb_ = 0;
  float vd_off_ = 0;   // p_D at anchor: maps the pad/baro vertical datum
                       // into the origin frame for geodetic output
  float g_ned_[3] = {0, 0, 9.80665f};

  // Covariance and scratch (static, no allocation).
  float P_[NX][NX];
  float Phi_[NX][NX];
  float T1_[NX][NX];
  float T2_[NX][NX];

  nav::GeoOrigin origin_;
  uint64_t t_last_us_ = 0;
  float acc_ned_[3] = {0, 0, 0};
  uint32_t prop_count_ = 0;
  InnovRecord innov_[CH_COUNT];

  // Alignment accumulators.
  uint64_t align_first_us_ = 0, align_win_us_ = 0;
  float asum_[3], asum2_[3], gsum_[3], gsum2_[3], msum_[3];
  uint32_t acount_ = 0, mcount_ = 0;
  bool mag_init_used_ = false;

  // Stillness tracker (EWMA) for the FS_WAIT_FIX gravity refinement.
  float still_gmean_[3] = {0, 0, 0};
  float still_gdev2_ = 0;
  float still_fnorm_ = 0;
  uint32_t still_streak_ = 0;
  uint32_t grav_decim_ctr_ = 0;

  // Reference field bearings/inclination, precomputed at init.
  float mag_beta_ref_ = 0;   // atan2(E, N) of the reference field
  float mag_bh_ref_ = 1;     // horizontal magnitude of reference field
  float mag_bd_ref_ = 0;     // down component of reference field
  float mag_bnorm_ref_ = 1;  // |reference field|
  // Run-time field reference for the vector update. Captured from the
  // alignment field mean when the mag seeded the heading (its azimuth then
  // equals the configured declination BY CONSTRUCTION, while magnitude and
  // inclination are the measured local values), so the continuous gates and
  // innovation are judged against the field that actually exists at this
  // vehicle - not a site model the local (iron-distorted) field may never
  // match. Falls back to the configured reference when alignment ran
  // without mag.
  float mag_ref_run_[3] = { 0, 0, 1 };
  float mag_bnorm_run_ = 1, mag_incl_run_ = 0;
  float mag_incl_ref_ = 0;   // reference inclination
};

}  // namespace eskf
