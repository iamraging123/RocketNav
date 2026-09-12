// sensors.h — acquisition, validation, sanitization and calibration for all
// four sensors on the shared 400 kHz I2C bus, plus bus-fault recovery.
//
// Correction order (documented, applied in this order everywhere):
//   raw counts -> SI units -> axis remap into body frame -> temperature
//   compensation -> bias/scale -> (mag only) hard-iron then soft-iron.
// Temperature compensation is identity for the ICM-45686 and LIS3MDL (both
// factory-compensate internally; hooks would be dead code) and lives inside
// Bosch's compensation formula for the BME280 (the Adafruit driver applies
// it before we ever see Pa).
//
// Every sample is timestamped at acquisition (the poll that read it), never
// at parse. Library return values are treated as untrusted: everything is
// range-checked, NaN/inf-checked, stuck-checked and (for GNSS) fix-level
// validated before the filter can see it.

#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "icm45686.h"
#include "magfit.h"
#include "eskf.h"

// v2 library: the SAM-M8Q speaks the legacy UBX-CFG interface. The v3
// library is VALSET-only (M9/M10) and cannot talk to an M8 at all — its
// begin() ping fails, which reads as "no module" in the boot header.
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_BME280.h>
#include "lis3mdl.h"

struct SensorsConfig {
  uint8_t imu_addr, mag_addr, baro_addr, gnss_addr;
  uint32_t i2c_hz;
  int16_t sda_pin, scl_pin;  // for the bus-clear routine (-1 = unknown, skip)

  Icm45686::AccelFs accel_fs;
  Icm45686::GyroFs gyro_fs;
  Icm45686::Odr imu_odr;
  Icm45686::LpfBw accel_lpf, gyro_lpf;
  uint32_t imu_discard_ms;      // discard the first samples after power-up

  const float *imu_mount;       // 3x3 row-major, sensor -> body
  const float *mag_mount;       // 3x3 row-major, sensor -> body
  const float *gyro_bias0;      // rad/s, fixed pre-calibration (body frame)
  const float *accel_bias0;     // m/s^2, fixed pre-calibration (body frame)
  const float *accel_scale;     // unitless diag scale (body frame)
  const float *mag_hard_ut;     // uT hard-iron offset (body frame)
  const float *mag_soft;        // 3x3 row-major soft-iron correction (body)

  uint16_t gnss_meas_ms;        // CFG-RATE-MEAS, ms (100 = 10 Hz)
  float gnss_target_hz;         // for the runtime rate verification
  uint8_t gnss_min_sv;
  float gnss_max_hacc_m;
  bool gnss_enabled;            // false (GPS_OFF): never init or poll GNSS
};

struct SensorHealth {
  bool present = false;   // begin() succeeded at some point
  bool fresh = false;     // a valid sample within the staleness window
  bool stale = true;
  bool sat = false;       // latest sample saturated (accel full scale)
  bool stuck = false;     // N identical consecutive raw samples
  bool fault = false;     // bus/init failure, reinit pending
  uint32_t drops = 0;     // samples rejected before reaching the filter
  uint64_t last_fresh_us = 0;
  uint8_t bits() const {
    return (uint8_t)((present ? 1 : 0) | (fresh ? 2 : 0) | (stale ? 4 : 0) |
                     (sat ? 8 : 0) | (stuck ? 16 : 0) | (fault ? 32 : 0));
  }
};

struct GnssFix {
  uint64_t t_us = 0;        // timestamped at I2C read (callback) time
  int32_t lat1e7 = 0, lon1e7 = 0, hae_mm = 0, hmsl_mm = 0;
  float vel_ned[3] = {0, 0, 0};   // m/s
  float hacc_m = 0, vacc_m = 0, sacc_ms = 0;
  uint8_t fix_type = 0, num_sv = 0;
  bool fix_ok = false;
  uint32_t itow = 0;
  bool valid_for_nav = false;  // passed fix-level validation
};

class Sensors {
 public:
  void begin(const SensorsConfig &cfg, uint64_t now_us);

  // Each poll performs at most one bounded I2C transaction group and never
  // blocks beyond it. Returns true only for a fresh, validated, calibrated
  // sample.
  bool pollImu(uint64_t now_us, eskf::ImuSample &out);
  bool pollMag(uint64_t now_us, float mag_ut[3]);
  bool pollBaro(uint64_t now_us, float *press_pa, float *temp_c,
                float *alt_rel_m);
  void pollGnss(uint64_t now_us);
  bool haveNewFix(GnssFix &out);

  // Barometric reference capture (during alignment; ino orchestrates).
  void baroRefCollect(bool enable);
  bool baroRefReady() const { return baro_ref_ready_; }
  float baroRefPa() const { return baro_ref_pa_; }
  float baroRefC() const { return baro_ref_c_; }

  // Watchdogs: staleness flags, per-sensor reinit backoff, bus-clear.
  void service(uint64_t now_us);

  const SensorHealth &imuHealth() const { return h_imu_; }
  const SensorHealth &magHealth() const { return h_mag_; }
  const SensorHealth &baroHealth() const { return h_baro_; }
  const SensorHealth &gnssHealth() const { return h_gnss_; }
  uint32_t i2cErrors() const { return i2c_errors_; }
  uint32_t busResets() const { return bus_resets_; }
  void zeroCounters();   // ground command $zero: drop + bus error counters

  // Hard-iron calibration ($magcal): 30 s least-squares sphere fit over the
  // raw BODY-frame field while the vehicle is tumbled through all
  // orientations (magfit.h); the offset is applied to RAM immediately and
  // the .ino persists it to the flash config store.
  void magCalStart(uint64_t now_us);
  // Overwrite the live hard-iron offset (uT, body frame) - used by the boot
  // restore from flash and by $magclr.
  void magHardSet(const float h[3]) {
    for (int i = 0; i < 3; ++i) mag_hard_ram_[i] = h[i];
  }
  bool magCalActive() const { return magcal_on_; }
  // Time-based sweep completion. Called from the service path so the sweep
  // closes and reports even if the magnetometer dies mid-cal - completion
  // used to live inside pollMag and required a fresh sample AFTER the 30 s
  // deadline, which turned any mid-sweep sensor fault into eternal silence.
  void magCalService(uint64_t now_us);
  // Live sweep state for the ground progress heartbeat. False when idle.
  bool magCalProgress(uint64_t now_us, uint32_t *n, float spread_ut[3],
                      float *last_ut, uint32_t *elapsed_s) const;
  // True exactly once when a sweep finishes. hi_ut = offsets now applied;
  // spread_ut = per-axis field swing seen; held_mask bit i = axis i kept
  // its previous offset (swing too thin to separate iron from Earth field
  // along it - or the whole fit failed, then all three bits are set).
  bool magCalTakeResult(float hi_ut[3], float spread_ut[3],
                        uint8_t *held_mask);
  // Register-level probe for the $magdiag ground command: WHO_AM_I, control
  // registers, STATUS, and the three multi-byte read strategies compared.
  // Bypasses the driver read path so it stays honest when the driver is the
  // suspect. Blocks ~60 ms (explicit ground command only).
  void magDiag(MagDiag &out) { Lis3mdl::diag(Wire, cfg_.mag_addr, out); }
  // Raw field (uT, body frame): scaled + mount-rotated only — before the
  // hard/soft-iron correction, the validity gate, and the filter. Updates on
  // every ZYXDA-fresh register read, so it keeps moving even when
  // calibration or gating freezes lastMag(). False until the first read.
  bool magRawSeen() const { return mag_raw_seen_; }
  const float *lastMagRaw() const { return last_mag_raw_; }
  float gnssRateHz() const { return gnss_rate_hz_; }
  bool constellationsReduced() const { return constellations_reduced_; }
  const GnssFix &lastPvt() const { return last_pvt_; }  // raw, any validity
  bool everHadPvt() const { return ever_pvt_; }
  // Latest calibrated values (for telemetry, even between filter feeds).
  const float *lastAccel() const { return last_accel_; }
  const float *lastGyro() const { return last_gyro_; }
  const float *lastMag() const { return last_mag_; }
  float lastPressPa() const { return last_press_pa_; }
  float lastTempC() const { return last_temp_c_; }

 private:
  bool beginImu();
  bool beginMag();
  bool beginBaro();
  bool beginGnss();
  void noteI2cError();
  void noteI2cOk();
  void busClear();
  static void pvtCallback(UBX_NAV_PVT_data_t *pvt);

  SensorsConfig cfg_{};
  Icm45686 imu_;
  Lis3mdl mag_;
  Adafruit_BME280 bme_;
  SFE_UBLOX_GNSS gnss_;

  SensorHealth h_imu_, h_mag_, h_baro_, h_gnss_;
  uint64_t boot_us_ = 0;

  // IMU
  Icm45686::RawSample imu_prev_{};
  uint32_t imu_stuck_n_ = 0, imu_fail_n_ = 0, baro_fail_n_ = 0;
  uint64_t imu_ready_us_ = 0;  // end of the discard window
  float last_accel_[3] = {0, 0, 0};
  float last_gyro_[3] = {0, 0, 0};

  // Mag
  int16_t mag_prev_[3] = {0, 0, 0};
  uint32_t mag_stuck_n_ = 0, mag_fail_n_ = 0;
  float last_mag_[3] = {0, 0, 0};
  float last_mag_raw_[3] = {0, 0, 0};
  bool mag_raw_seen_ = false;

  // Baro
  float baro_prev_pa_ = 0;
  uint32_t baro_stuck_n_ = 0;
  float last_press_pa_ = 0, last_temp_c_ = 0;
  bool baro_collecting_ = false, baro_ref_ready_ = false;
  double baro_ref_psum_ = 0, baro_ref_tsum_ = 0;
  uint32_t baro_ref_n_ = 0;
  float baro_ref_pa_ = 101325.0f, baro_ref_c_ = 15.0f;

  // GNSS
  volatile uint64_t gnss_poll_us_ = 0;  // acquisition timestamp for callback
  GnssFix pending_fix_{}, last_pvt_{};
  volatile bool new_fix_ = false;
  bool ever_pvt_ = false;
  uint32_t last_used_itow_ = 0xFFFFFFFF;
  uint64_t last_pvt_us_ = 0;
  float gnss_rate_hz_ = 0;
  bool constellations_reduced_ = false;
  bool gnss_configured_ = false;

  // Bus fault management
  uint32_t i2c_errors_ = 0, i2c_consec_ = 0, bus_resets_ = 0;
  bool magcal_on_ = false, magcal_done_ = false;
  uint64_t magcal_t0_ = 0;
  magfit::SphereFit magcal_fit_;
  float magcal_spread_[3] = {0, 0, 0};
  float magcal_last_ut_ = 0;
  uint8_t magcal_held_ = 0;
  float mag_hard_ram_[3] = {0, 0, 0};   // live hard-iron (config at boot)
  uint64_t reinit_next_us_[4] = {0, 0, 0, 0};   // imu, mag, baro, gnss
  uint32_t reinit_backoff_ms_[4] = {1000, 1000, 1000, 1000};

  static Sensors *self_;  // for the C-style GNSS callback
};
