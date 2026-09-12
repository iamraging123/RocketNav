// sensors.cpp — see sensors.h for the correction order and trust model.
// Scheduling contract: every poll*() performs at most one bounded I2C
// transaction group and returns; all delays live in begin/reinit paths
// (setup-time) or the ~100 us bus-clear pulse train.

#include "sensors.h"
#include <math.h>

Sensors *Sensors::self_ = nullptr;

// Accel counts at/above this magnitude are treated as saturated (98% of the
// int16 range: the AAF can shave true full-scale before the ADC clips).
static const int16_t kSatCounts = 32000;
// Mag counts at/above this magnitude are treated as an axis at its range
// limit (~3.95 gauss of the +/-4 gauss full scale — the analog chain can
// clip before the digital rail). Flagged in health, not dropped: the |m|
// validity gate already rejects the sample downstream.
static const int16_t kMagSatCounts = 27000;

static inline void mat3_mul_vec(const float *M, const float *v, float *out) {
  out[0] = M[0] * v[0] + M[1] * v[1] + M[2] * v[2];
  out[1] = M[3] * v[0] + M[4] * v[1] + M[5] * v[2];
  out[2] = M[6] * v[0] + M[7] * v[1] + M[8] * v[2];
}

void Sensors::begin(const SensorsConfig &cfg, uint64_t now_us) {
  cfg_ = cfg;
  for (int i = 0; i < 3; ++i) mag_hard_ram_[i] = cfg.mag_hard_ut[i];
  self_ = this;
  boot_us_ = now_us;
  Wire.begin();
  Wire.setClock(cfg_.i2c_hz);

  h_imu_.present = beginImu();
  h_imu_.fault = !h_imu_.present;
  h_mag_.present = beginMag();
  h_mag_.fault = !h_mag_.present;
  h_baro_.present = beginBaro();
  h_baro_.fault = !h_baro_.present;
  h_gnss_.present = cfg_.gnss_enabled ? beginGnss() : false;
  h_gnss_.fault = cfg_.gnss_enabled && !h_gnss_.present;

  imu_ready_us_ = now_us + (uint64_t)cfg_.imu_discard_ms * 1000ull;
  for (int i = 0; i < 4; ++i) {
    reinit_next_us_[i] = now_us + 1000000ull;
    reinit_backoff_ms_[i] = 1000;
  }
}

bool Sensors::beginImu() {
  return imu_.begin(Wire, cfg_.imu_addr, cfg_.accel_fs, cfg_.gyro_fs,
                    cfg_.imu_odr, cfg_.accel_lpf, cfg_.gyro_lpf);
}

bool Sensors::beginMag() {
  // UHP + 80 Hz continuous, +/-4 gauss: earth field is ~0.52 gauss, hard
  // iron adds margin, and the finest LSB (0.0146 uT) minimizes quantization
  // in the heading innovation. Config values and the verified readback live
  // in the driver (lis3mdl.h).
  return mag_.begin(Wire, cfg_.mag_addr);
}

bool Sensors::beginBaro() {
  if (!bme_.begin(cfg_.baro_addr, &Wire)) return false;
  // NORMAL mode, pressure x8 / temp x1 / humidity off, IIR off, 0.5 ms
  // standby: ~22 ms per conversion (~44 Hz internal), so a 25 Hz poll always
  // sees fresh data. IIR off keeps bandwidth for the vertical channel; the
  // filter's R handles the noise.
  bme_.setSampling(Adafruit_BME280::MODE_NORMAL, Adafruit_BME280::SAMPLING_X1,
                   Adafruit_BME280::SAMPLING_X8, Adafruit_BME280::SAMPLING_NONE,
                   Adafruit_BME280::FILTER_OFF, Adafruit_BME280::STANDBY_MS_0_5);
  return true;
}

bool Sensors::beginGnss() {
  bool ok = false;
  for (int i = 0; i < 2 && !ok; ++i) {
    ok = gnss_.begin(Wire, cfg_.gnss_addr, 300, false);
  }
  if (!ok) return false;
  // SAM-M8Q: configuration through the legacy UBX-CFG messages (CFG-PRT /
  // CFG-NAV5 / CFG-RATE), which is what the SparkFun v2 helpers below emit.
  bool cfg_ok = true;
  cfg_ok &= gnss_.setI2COutput(COM_TYPE_UBX);  // no NMEA on the shared bus
  cfg_ok &= gnss_.setDynamicModel(DYN_MODEL_AIRBORNE4g);
  cfg_ok &= gnss_.setMeasurementRate(cfg_.gnss_meas_ms);  // request 10 Hz
  cfg_ok &= gnss_.setAutoPVTcallbackPtr(&Sensors::pvtCallback);
  gnss_configured_ = cfg_ok;
  if (!cfg_ok) i2c_errors_++;  // visible in telemetry; rate check follows up
  return true;
}

// ---------- IMU ----------

bool Sensors::pollImu(uint64_t now_us, eskf::ImuSample &out) {
  if (!h_imu_.present || h_imu_.fault) return false;
  Icm45686::RawSample rs;
  if (!imu_.readSample(rs)) {
    noteI2cError();
    h_imu_.drops++;
    if (++imu_fail_n_ > 50) h_imu_.fault = true;  // reinit via service()
    return false;
  }
  noteI2cOk();
  imu_fail_n_ = 0;
  if (!rs.drdy) return false;  // stale poll: expected, we poll above the ODR
  if (now_us < imu_ready_us_) return false;  // sensor start-up transient

  // Stuck detection: all six raw axes identical across consecutive fresh
  // samples. Sensor noise (>0.4 LSB rms every axis) makes a long run of
  // exact repeats a wedged-silicon signature, not a quiet bench.
  if (rs.ax == imu_prev_.ax && rs.ay == imu_prev_.ay && rs.az == imu_prev_.az &&
      rs.gx == imu_prev_.gx && rs.gy == imu_prev_.gy && rs.gz == imu_prev_.gz) {
    imu_stuck_n_++;
  } else {
    imu_stuck_n_ = 0;
    h_imu_.stuck = false;
  }
  imu_prev_ = rs;
  if (imu_stuck_n_ > 200) {  // 0.5 s of frozen output
    h_imu_.stuck = true;
    h_imu_.fault = true;
    h_imu_.drops++;
    return false;
  }

  bool sat = (rs.ax <= -kSatCounts || rs.ax >= kSatCounts ||
              rs.ay <= -kSatCounts || rs.ay >= kSatCounts ||
              rs.az <= -kSatCounts || rs.az >= kSatCounts);

  // raw counts -> SI (sensor frame)
  float as[3] = { rs.ax * imu_.accelScale(), rs.ay * imu_.accelScale(),
                  rs.az * imu_.accelScale() };
  float gs[3] = { rs.gx * imu_.gyroScale(), rs.gy * imu_.gyroScale(),
                  rs.gz * imu_.gyroScale() };
  // axis remap into body frame
  float ab[3], gb[3];
  mat3_mul_vec(cfg_.imu_mount, as, ab);
  mat3_mul_vec(cfg_.imu_mount, gs, gb);
  // temperature compensation: identity (factory-compensated on-chip)
  // bias/scale (body frame)
  for (int i = 0; i < 3; ++i) {
    ab[i] = ab[i] * cfg_.accel_scale[i] - cfg_.accel_bias0[i];
    gb[i] = gb[i] - cfg_.gyro_bias0[i];
  }
  for (int i = 0; i < 3; ++i) {
    if (!isfinite(ab[i]) || !isfinite(gb[i])) { h_imu_.drops++; return false; }
  }

  out.t_us = now_us;  // acquisition timestamp (poll that read the burst)
  for (int i = 0; i < 3; ++i) { out.gyro[i] = gb[i]; out.accel[i] = ab[i]; }
  out.sat = sat;
  h_imu_.sat = sat;
  h_imu_.fresh = true;
  h_imu_.stale = false;
  h_imu_.last_fresh_us = now_us;
  for (int i = 0; i < 3; ++i) { last_accel_[i] = ab[i]; last_gyro_[i] = gb[i]; }
  return true;
}

// ---------- magnetometer ----------

bool Sensors::pollMag(uint64_t now_us, float mag_ut[3]) {
  if (!h_mag_.present || h_mag_.fault) return false;
  // One 7-byte burst: STATUS.ZYXDA and the sample it describes, atomically.
  Lis3mdl::RawSample rs;
  if (!mag_.readSample(rs)) {
    noteI2cError();
    h_mag_.drops++;
    if (++mag_fail_n_ > 50) h_mag_.fault = true;  // reinit via service()
    return false;
  }
  noteI2cOk();
  mag_fail_n_ = 0;
  if (!rs.drdy) return false;  // stale poll: expected, we poll above the ODR
  int16_t raw[3] = { rs.mx, rs.my, rs.mz };
  h_mag_.sat = (raw[0] <= -kMagSatCounts || raw[0] >= kMagSatCounts ||
                raw[1] <= -kMagSatCounts || raw[1] >= kMagSatCounts ||
                raw[2] <= -kMagSatCounts || raw[2] >= kMagSatCounts);

  if (raw[0] == mag_prev_[0] && raw[1] == mag_prev_[1] &&
      raw[2] == mag_prev_[2]) {
    mag_stuck_n_++;
  } else {
    mag_stuck_n_ = 0;
    h_mag_.stuck = false;
  }
  mag_prev_[0] = raw[0]; mag_prev_[1] = raw[1]; mag_prev_[2] = raw[2];
  if (mag_stuck_n_ > 200) {
    h_mag_.stuck = true;
    h_mag_.fault = true;
    h_mag_.drops++;
    return false;
  }

  // raw counts -> SI (sensor frame, uT)
  const float k = Lis3mdl::utPerLsb();
  float ms[3] = { raw[0] * k, raw[1] * k, raw[2] * k };
  // axis remap into body frame
  float mb[3], mc[3];
  mat3_mul_vec(cfg_.mag_mount, ms, mb);
  // Raw telemetry tap: every fresh register read lands here, BEFORE
  // calibration and gating — the ground can always see what the sensor
  // itself is doing, even while the calibrated output is frozen by a gate.
  for (int i = 0; i < 3; ++i) last_mag_raw_[i] = mb[i];
  mag_raw_seen_ = true;
  // temperature compensation: identity (LIS3MDL compensates internally)
  // hard-iron then soft-iron, both defined in the BODY frame
  if (magcal_on_) {
    magcal_fit_.add(mb);  // raw (pre-subtract): the fit estimates the total
    magcal_last_ut_ = sqrtf(mb[0] * mb[0] + mb[1] * mb[1] + mb[2] * mb[2]);
    // Completion is NOT checked here: magCalService() closes the window on
    // time so a mid-sweep sensor fault can't leave the cal silently open.
  }
  for (int i = 0; i < 3; ++i) mb[i] -= mag_hard_ram_[i];
  mat3_mul_vec(cfg_.mag_soft, mb, mc);

  float n2 = mc[0] * mc[0] + mc[1] * mc[1] + mc[2] * mc[2];
  if (!isfinite(n2) || n2 < 25.0f || n2 > 40000.0f) {  // |m| in [5, 200] uT
    h_mag_.drops++;
    return false;
  }
  mag_ut[0] = mc[0]; mag_ut[1] = mc[1]; mag_ut[2] = mc[2];
  h_mag_.fresh = true;
  h_mag_.stale = false;
  h_mag_.last_fresh_us = now_us;
  for (int i = 0; i < 3; ++i) last_mag_[i] = mc[i];
  return true;
}

void Sensors::magCalService(uint64_t now_us) {
  if (!magcal_on_ || now_us - magcal_t0_ <= 30000000ull) return;
  magcal_on_ = false;
  // Least-squares sphere center = hard iron. Axes whose field swing stayed
  // thin can't separate iron from Earth field (a flat tabletop spin leaves
  // body-z pinned near B_down = 48 uT here - the min/max midpoint used to
  // swallow that straight into the offset); they hold their previous value
  // and are flagged for the ground message.
  float c[3];
  magcal_held_ = 0;
  if (magcal_fit_.solve(c, magcal_spread_)) {
    for (int i = 0; i < 3; ++i) {
      if (magcal_spread_[i] < 30.0f || fabsf(c[i]) > 300.0f) {
        magcal_held_ |= (uint8_t)(1u << i);
      } else {
        mag_hard_ram_[i] = c[i];
      }
    }
  } else {
    magcal_held_ = 0x7;  // fit failed / too few samples: nothing changes
  }
  magcal_done_ = true;
}

bool Sensors::magCalProgress(uint64_t now_us, uint32_t *n, float spread_ut[3],
                             float *last_ut, uint32_t *elapsed_s) const {
  if (!magcal_on_) return false;
  *n = magcal_fit_.count();
  magcal_fit_.spreads(spread_ut);
  *last_ut = magcal_last_ut_;
  *elapsed_s = (uint32_t)((now_us - magcal_t0_) / 1000000ull);
  return true;
}

void Sensors::magCalStart(uint64_t now_us) {
  magcal_fit_.reset();
  magcal_t0_ = now_us;
  magcal_on_ = true;
  magcal_done_ = false;
}

bool Sensors::magCalTakeResult(float hi_ut[3], float spread_ut[3],
                               uint8_t *held_mask) {
  if (!magcal_done_) return false;
  for (int i = 0; i < 3; ++i) {
    hi_ut[i] = mag_hard_ram_[i];
    spread_ut[i] = magcal_spread_[i];
  }
  *held_mask = magcal_held_;
  magcal_done_ = false;
  return true;
}

void Sensors::zeroCounters() {
  h_imu_.drops = 0;
  h_mag_.drops = 0;
  h_baro_.drops = 0;
  h_gnss_.drops = 0;
  i2c_errors_ = 0;
  bus_resets_ = 0;
}

// ---------- barometer ----------

bool Sensors::pollBaro(uint64_t now_us, float *press_pa, float *temp_c,
                       float *alt_rel_m) {
  if (!h_baro_.present || h_baro_.fault) return false;
  float t = bme_.readTemperature();
  float p = bme_.readPressure();
  if (!isfinite(t) || !isfinite(p) || p < 30000.0f || p > 120000.0f ||
      t < -45.0f || t > 90.0f) {
    h_baro_.drops++;
    if (++baro_fail_n_ > 25) h_baro_.fault = true;
    return false;
  }
  baro_fail_n_ = 0;

  if (p == baro_prev_pa_) {  // bit-exact repeat of a 20-bit ADC + compensation
    baro_stuck_n_++;
  } else {
    baro_stuck_n_ = 0;
    h_baro_.stuck = false;
  }
  baro_prev_pa_ = p;
  if (baro_stuck_n_ > 100) {  // 4 s frozen at 25 Hz
    h_baro_.stuck = true;
    h_baro_.fault = true;
    h_baro_.drops++;
    return false;
  }

  if (baro_collecting_) {
    baro_ref_psum_ += p;
    baro_ref_tsum_ += t;
    baro_ref_n_++;
  }

  *press_pa = p;
  *temp_c = t;
  if (baro_ref_ready_) {
    // Hypsometric altitude relative to the alignment reference, using the
    // measured reference temperature (temperature dependence handled here;
    // the exponent is R_dry*L/g0 = 287.05*0.0065/9.80665).
    float t0k = baro_ref_c_ + 273.15f;
    *alt_rel_m = (t0k / 0.0065f) * (1.0f - powf(p / baro_ref_pa_, 0.190263f));
  } else {
    *alt_rel_m = NAN;
  }
  last_press_pa_ = p;
  last_temp_c_ = t;
  h_baro_.fresh = true;
  h_baro_.stale = false;
  h_baro_.last_fresh_us = now_us;
  return true;
}

void Sensors::baroRefCollect(bool enable) {
  if (enable && !baro_collecting_) {
    baro_ref_psum_ = 0;
    baro_ref_tsum_ = 0;
    baro_ref_n_ = 0;
    baro_collecting_ = true;
  } else if (!enable && baro_collecting_) {
    baro_collecting_ = false;
    if (baro_ref_n_ > 0) {
      baro_ref_pa_ = (float)(baro_ref_psum_ / baro_ref_n_);
      baro_ref_c_ = (float)(baro_ref_tsum_ / baro_ref_n_);
      baro_ref_ready_ = true;
    }
  }
}

// ---------- GNSS ----------

void Sensors::pvtCallback(UBX_NAV_PVT_data_t *pvt) {
  Sensors *s = self_;
  if (s == nullptr || pvt == nullptr) return;
  GnssFix f;
  f.t_us = s->gnss_poll_us_;  // acquisition = the I2C poll that drained it
  f.lat1e7 = pvt->lat;
  f.lon1e7 = pvt->lon;
  f.hae_mm = pvt->height;
  f.hmsl_mm = pvt->hMSL;
  f.vel_ned[0] = pvt->velN * 0.001f;
  f.vel_ned[1] = pvt->velE * 0.001f;
  f.vel_ned[2] = pvt->velD * 0.001f;
  f.hacc_m = pvt->hAcc * 0.001f;
  f.vacc_m = pvt->vAcc * 0.001f;
  f.sacc_ms = pvt->sAcc * 0.001f;
  f.fix_type = pvt->fixType;
  f.num_sv = pvt->numSV;
  f.fix_ok = pvt->flags.bits.gnssFixOK != 0;
  f.itow = pvt->iTOW;
  // Achieved-rate estimate (EWMA of PVT arrival intervals).
  if (s->last_pvt_us_ != 0 && f.t_us > s->last_pvt_us_) {
    float dt = (float)(f.t_us - s->last_pvt_us_) * 1e-6f;
    if (dt > 0.02f && dt < 5.0f) {
      float hz = 1.0f / dt;
      s->gnss_rate_hz_ += 0.2f * (hz - s->gnss_rate_hz_);
    }
  }
  s->last_pvt_us_ = f.t_us;
  s->ever_pvt_ = true;
  s->last_pvt_ = f;
  s->pending_fix_ = f;
  s->new_fix_ = true;
  s->h_gnss_.fresh = true;
  s->h_gnss_.stale = false;
  s->h_gnss_.last_fresh_us = f.t_us;
}

void Sensors::pollGnss(uint64_t now_us) {
  if (!cfg_.gnss_enabled || !h_gnss_.present) return;
  gnss_poll_us_ = now_us;
  gnss_.checkUblox();      // drain the DDC stream (library chunks at 32 B)
  gnss_.checkCallbacks();  // fire the PVT callback for anything parsed

  // Verify the achieved nav rate at runtime instead of assuming it. If the
  // module can't hold the requested rate with every constellation enabled,
  // drop BeiDou and GLONASS (keep GPS + Galileo) via UBX-CFG-GNSS, once.
  if (!constellations_reduced_ && ever_pvt_ &&
      (now_us - boot_us_) > 30000000ull &&
      gnss_rate_hz_ < 0.7f * cfg_.gnss_target_hz) {
    gnss_.enableGNSS(false, SFE_UBLOX_GNSS_ID_BEIDOU);
    gnss_.enableGNSS(false, SFE_UBLOX_GNSS_ID_GLONASS);
    constellations_reduced_ = true;  // one-shot; visible in telemetry
  }
}

bool Sensors::haveNewFix(GnssFix &out) {
  if (!new_fix_) return false;
  new_fix_ = false;
  GnssFix f = pending_fix_;
  // Fix-level validation: a getLatitude()-style zero with no fix must never
  // reach the filter as an equator measurement.
  bool zero_pos = (f.lat1e7 == 0 && f.lon1e7 == 0);
  bool finite_ok = isfinite(f.vel_ned[0]) && isfinite(f.vel_ned[1]) &&
                   isfinite(f.vel_ned[2]) && isfinite(f.hacc_m) &&
                   isfinite(f.vacc_m) && isfinite(f.sacc_ms);
  f.valid_for_nav = (f.fix_type == 3) && f.fix_ok &&
                    (f.num_sv >= cfg_.gnss_min_sv) &&
                    (f.hacc_m <= cfg_.gnss_max_hacc_m) && !zero_pos &&
                    (f.itow != last_used_itow_) && finite_ok;
  if (f.valid_for_nav) {
    last_used_itow_ = f.itow;
  } else {
    h_gnss_.drops++;
  }
  out = f;
  return true;
}

// ---------- watchdogs / recovery ----------

void Sensors::noteI2cError() {
  i2c_errors_++;
  i2c_consec_++;
}

void Sensors::noteI2cOk() { i2c_consec_ = 0; }

void Sensors::busClear() {
  // One wedged device holding SDA low must not take down all four sensors:
  // clock out up to 9 SCL pulses so the slave finishes its byte and releases
  // the line, then issue a STOP and re-init the peripheral. ~100 us of
  // delayMicroseconds, bounded, and only on a detected wedge.
  Wire.end();
  if (cfg_.scl_pin >= 0 && cfg_.sda_pin >= 0) {
    pinMode((uint32_t)cfg_.scl_pin, OUTPUT_OPEN_DRAIN);
    digitalWrite((uint32_t)cfg_.scl_pin, HIGH);
    pinMode((uint32_t)cfg_.sda_pin, INPUT_PULLUP);
    delayMicroseconds(5);
    if (digitalRead((uint32_t)cfg_.sda_pin) == LOW) {
      for (int i = 0; i < 9; ++i) {
        digitalWrite((uint32_t)cfg_.scl_pin, LOW);
        delayMicroseconds(5);
        digitalWrite((uint32_t)cfg_.scl_pin, HIGH);
        delayMicroseconds(5);
        if (digitalRead((uint32_t)cfg_.sda_pin) == HIGH) break;
      }
    }
    // STOP condition: SDA low->high while SCL high.
    pinMode((uint32_t)cfg_.sda_pin, OUTPUT_OPEN_DRAIN);
    digitalWrite((uint32_t)cfg_.sda_pin, LOW);
    delayMicroseconds(5);
    digitalWrite((uint32_t)cfg_.sda_pin, HIGH);
    delayMicroseconds(5);
  }
  Wire.begin();
  Wire.setClock(cfg_.i2c_hz);
  bus_resets_++;
  i2c_consec_ = 0;
  // Force every sensor through the reinit path.
  h_imu_.fault = true;
  h_mag_.fault = true;
  h_baro_.fault = true;
  h_gnss_.fault = true;
  for (int i = 0; i < 4; ++i) reinit_next_us_[i] = 0;
}

void Sensors::service(uint64_t now_us) {
  // Staleness (per-sensor timeouts; a sensor that stops producing valid
  // samples goes stale even if the bus is electrically fine).
  if (h_imu_.present && now_us - h_imu_.last_fresh_us > 50000ull) {
    h_imu_.fresh = false;
    h_imu_.stale = true;
  }
  if (h_mag_.present && now_us - h_mag_.last_fresh_us > 300000ull) {
    h_mag_.fresh = false;
    h_mag_.stale = true;
  }
  if (h_baro_.present && now_us - h_baro_.last_fresh_us > 300000ull) {
    h_baro_.fresh = false;
    h_baro_.stale = true;
  }
  if (h_gnss_.present && now_us - h_gnss_.last_fresh_us > 3000000ull) {
    h_gnss_.fresh = false;
    h_gnss_.stale = true;
  }

  if (i2c_consec_ >= 10) busClear();

  // Reinit with exponential backoff — a dead sensor degrades, never hangs.
  struct Slot { SensorHealth *h; int idx; };
  Slot slots[4] = { { &h_imu_, 0 }, { &h_mag_, 1 }, { &h_baro_, 2 },
                    { &h_gnss_, 3 } };
  for (int i = 0; i < 4; ++i) {
    SensorHealth *h = slots[i].h;
    int k = slots[i].idx;
    if (k == 3 && !cfg_.gnss_enabled) continue;  // GPS_OFF: never retry
    if ((h->present && !h->fault) || now_us < reinit_next_us_[k]) continue;
    bool ok = false;
    switch (k) {
      case 0: ok = beginImu(); break;
      case 1: ok = beginMag(); break;
      case 2: ok = beginBaro(); break;
      case 3: ok = beginGnss(); break;
    }
    if (ok) {
      h->present = true;
      h->fault = false;
      h->stuck = false;
      reinit_backoff_ms_[k] = 1000;
      if (k == 0) {
        imu_fail_n_ = 0;
        imu_stuck_n_ = 0;
        imu_ready_us_ = now_us + (uint64_t)cfg_.imu_discard_ms * 1000ull;
      }
      if (k == 2) { baro_fail_n_ = 0; baro_stuck_n_ = 0; }
      if (k == 1) { mag_stuck_n_ = 0; mag_fail_n_ = 0; }
    } else {
      reinit_backoff_ms_[k] *= 2;
      if (reinit_backoff_ms_[k] > 30000) reinit_backoff_ms_[k] = 30000;
    }
    reinit_next_us_[k] = now_us + (uint64_t)reinit_backoff_ms_[k] * 1000ull;
  }
}
