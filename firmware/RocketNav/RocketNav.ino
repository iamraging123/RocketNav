// ============================================================================
// RocketNav — GNSS/IMU/mag/baro navigation estimator for HPR vehicles.
// Target: STM32F722RETx (Generic STM32F7 / F722RETx, STM32duino core 2.12.0)
//
// Estimates WGS-84 position and attitude by fusing:
//   u-blox SAM-M8Q (UBX NAV-PVT over I2C), TDK ICM-45686 (hand-written
//   driver, icm45686.cpp), ST LIS3MDL (hand-written driver, lis3mdl.cpp —
//   the shelf driver never asserted the I2C multi-byte auto-increment bit),
//   Bosch BME280 (Adafruit driver) — all on the default Wire bus at 400 kHz.
//
// Libraries (installed via arduino-cli, exact Library Manager names):
//   SparkFun u-blox GNSS Arduino Library 2.2.28  (v2 legacy UBX-CFG:
//     the M8 cannot speak the v3 library's VALSET-only interface)
//   Adafruit BME280 Library        2.3.0
//   Adafruit BusIO                 1.17.4   (dependency)
//   Adafruit Unified Sensor        1.1.15   (dependency)
// Matrix math and the ESKF are hand-rolled fixed-size float32 (eskf.cpp);
// no AHRS/fusion library of any kind is used.
//
// Out of scope by design (see the build brief): no flight-phase detection,
// no recovery/pyro, no logging/radio, no control. The estimator runs the
// same code on the pad, in boost, and on the ground.
// ============================================================================

#include <Wire.h>
#include "nav_frames.h"
#include "eskf.h"
#include "icm45686.h"
#include "cfgstore.h"
#include "leds.h"
#include "sensors.h"
#include "telemetry.h"
#include <SPI.h>
#include "control.h"
#include "link.h"
#include "linkcodec.h"
#include "servos.h"
#include "sx1278.h"

// ============================================================================
// CONFIG BLOCK — everything tunable lives here.
// (One compile-time exception: TELEM_RING_BYTES / TELEM_LINE_BYTES are array
// sizes and live at the top of telemetry.h.)
// ============================================================================

#define FW_VERSION "1.0.0"
#define SCHEMA_VERSION 1

// ---- serial output -------------------------------------------------------
// On the GENERIC_F722RETX variant `Serial` is UART4: TX=PA0, RX=PA1.
// Port was unspecified in the wiring table — variant default chosen; change
// to another HardwareSerial instance here if the harness uses different pins.
#define TELEM_SERIAL Serial
#define TELEM_BAUD 921600
#define TELEM_HZ 50            // output decimation, independent of filter rate

// ---- I2C bus -------------------------------------------------------------
// All four sensors share Wire (PB7=SDA, PB6=SCL on this variant), 400 kHz.
// Bus budget at 400 kHz (~22.5 us/byte with ACK bits):
//   IMU  status 4 B * 440 Hz + data 18 B * 400 Hz = ~20%
//   MAG  ~10 B/poll * 100 Hz = ~2.5%
//   BARO ~16 B/poll * 25 Hz  = ~0.9%
//   GNSS ~1 kB/s stream + length polls = ~3.5%
//   total ~27% — comfortable margin; that is why the IMU runs 400 Hz, not
//   the chip's 6.4 kHz maximum.
#define I2C_HZ 400000

// ---- status LEDs (WS2812B, ground-commanded via $led) --------------------
// Bit-banged in the service slot with IRQs masked <=120 us per strip. PC13
// sits in the low-speed RTC domain (~2 MHz output rating) — marginal for
// the 400 ns bit edges but within the WS2812B input threshold; keep that
// strip's wiring short.
#define LED_A_PIN PC10        // strip a: 2 pixels
#define LED_A_COUNT 2
#define LED_B_PIN PC13        // strip b: 4 pixels
#define LED_B_COUNT 4

// ---- LoRa link (Ra-02 / SX1278 on SPI1: PA5 SCK, PA6 MISO, PA7 MOSI) ------
// Air profile (frequency/SF/BW/CR/sync/cadence) lives in linkcodec.h and is
// shared with the base station. DIO1-5 are not wired: DIO0 + flag polls.
#define LORA_NSS_PIN PA4
#define LORA_RESET_PIN PC4
#define LORA_DIO0_PIN PB0
#define LORA_TX_DBM 17           // PA_BOOST; the Ra-02 ceiling w/o PA_DAC
#define LORA_TX_AT_BOOT 0        // bench default: DOWNLINK MUTED at boot so
                                 // an antenna-less Ra-02 never transmits
                                 // (PA damage risk). RX always runs. Enable
                                 // with $lora 1 / the viewer Unmute button,
                                 // or set 1 for field builds.

// ---- canard servos (PCA9685BS on the shared I2C sensor bus) ---------------
#define SERVO_PCA_ADDR 0x40      // all address straps low (verify on PCB)
#define SERVO_FRAME_HZ 100.0f    // MG90S (analog): spec point is 50 Hz, but
                                 // the accepted analog-servo envelope runs
                                 // to ~120 Hz (standard FBL-controller
                                 // setting) and the frame period is the
                                 // dominant command->pulse latency — 100 Hz
                                 // halves it. Watch the first minutes for
                                 // buzz/warmth: $sframe 50 reverts live.
                                 // 200-333 Hz remains digital-servo-only.
#define SERVO_WRITE_PER_FRAME 4  // PCA writes per PWM frame: the write latch
                                 // adds at most frame/4 on top of the frame
                                 // itself (writes still fire only on change)
#define SERVO_MIN_US 900.0f      // absolute pulse guard
#define SERVO_MAX_US 2100.0f
static const uint8_t SERVO_CH[4] = { 0, 1, 2, 3 };
static const float SERVO_CENTER_US[4] = { 1500, 1500, 1500, 1500 };
// Pulse per degree of canard deflection; SIGN = linkage direction. Initial
// values - verify each fin with $servo on the bench before ever arming.
static const float SERVO_US_PER_DEG[4] = { 10.0f, -10.0f, 10.0f, -10.0f };

// ---- roll control (bench-safe initial gains; 6DOF tuning pass to follow) --
// Sized so the proportional path carries the response: 0.08 x 8 = 0.64 deg
// of canard per deg of roll error. The first cut (0.04 / 3.0 = 0.12 deg/deg)
// left almost all authority to the integrator's multi-second crawl, which
// read as lag on the bench roll-hold test. $ctl retunes live (RAM).
#define CTL_KP_RATE 0.08f        // deg deflection per dps of rate error
#define CTL_KI_RATE 0.05f        // deg per dps-second (conditional integ.)
#define CTL_KP_ANG 8.0f          // dps of rate cmd per deg of roll error
#define CTL_RATE_CMD_MAX 180.0f  // dps
#define CTL_DEFL_MAX 10.0f       // deg: authority limit for first flights
#define CTL_SLEW_DPS 600.0f      // deg/s deflection slew = the MG90S's own
                                 // top speed (0.1 s/60 deg at 4.8 V): the
                                 // limiter should protect the linkage, not
                                 // out-brake the servo — at 400 it added
                                 // ~8 ms to every full-authority correction
// Aerodynamic mixing: sign of each canard for a +roll command.
static const float CTL_MIX_SIGN[4] = { 1, 1, 1, 1 };
#define CTL_LAUNCH_ACC_G 3.0f    // sustained axial accel = launch
#define CTL_LAUNCH_HOLD_S 0.1f
#define CTL_SAFE_TILT_DEG 60.0f  // nose off vertical -> stop controlling
#define CTL_SAFE_TIME_S 20.0f
#define CTL_SAFE_VD_MPS 3.0f     // descending -> stop controlling

// ---- sensor addresses (per schematic) ------------------------------------
#define IMU_I2C_ADDR 0x68     // ICM-45686, AD0 grounded, AP_CS high (I2C mode)
#define MAG_I2C_ADDR 0x1C     // LIS3MDL, SDO/SA1 grounded, CS high (I2C mode)
#define BARO_I2C_ADDR 0x77    // BME280, SDO to VDDIO, CSB high (I2C mode)
#define GNSS_I2C_ADDR 0x42    // SAM-M8Q DDC

// ---- IMU configuration ---------------------------------------------------
// ODR 400 Hz: fits the shared-bus budget above with margin, and gives the
// filter a 200 Hz Nyquist against a ~100 Hz airframe. INT/DRDY is NOT wired:
// we timer-poll at 440 Hz (~10% above ODR) and use the DRDY status bit read
// in the same burst as the data; a stale poll is skipped, never integrated.
// Full scales: +/-32 g (boost ~10-17 g for this vehicle class plus chute
// deployment shock — headroom beats the noise penalty of the 32 g range,
// DS-000489 Table 2 note) and +/-2000 dps (5.5 rev/s covers fin-induced
// roll; 4000 dps would halve resolution for rates we never see).
// UI LPF at ODR/4 = 100 Hz on both: anti-alias margin against structural
// vibration while keeping phase lag small at the 400 Hz sample rate.
#define IMU_ODR Icm45686::ODR_400HZ
#define IMU_ODR_HZ 400
#define IMU_POLL_HZ 440
#define IMU_ACCEL_FS Icm45686::AFS_32G
#define IMU_ACCEL_FS_G 32
#define IMU_GYRO_FS Icm45686::GFS_2000DPS
#define IMU_GYRO_FS_DPS 2000
#define IMU_ACCEL_LPF Icm45686::LPF_ODR_DIV_4
#define IMU_GYRO_LPF Icm45686::LPF_ODR_DIV_4
#define IMU_DISCARD_MS 150    // gyro start-up 35 ms (DS Table 1) + settling

// ---- magnetometer mode (compile-time) ------------------------------------
// 0 = MAG_OFF        never used; heading initialized at zero, wide sigma,
//                    heading covariance grows unbounded, nothing diverges.
// 1 = MAG_INIT_ONLY  heading from mag during alignment and until the origin
//                    anchors, then never again.
// 2 = MAG_CONTINUOUS scalar heading updates for the whole flight. Purpose:
//                    bound gyro drift about the vertical/body-roll axis —
//                    this vehicle flies near-vertical, so body-roll drift IS
//                    heading drift and GNSS cannot observe it.
#define MAG_MODE_SEL 2
#define MAG_ODR_HZ 80         // LIS3MDL ODR (UHP mode)
#define MAG_POLL_HZ 100       // poll above ODR; ZYXDA gates freshness

// ---- barometer -----------------------------------------------------------
#define BARO_POLL_HZ 25       // BME280 converts at ~44 Hz with x8/x1 sampling

// ---- GNSS ----------------------------------------------------------------
#define GNSS_MEAS_MS 100      // request 10 Hz nav rate (VALSET CFG-RATE-MEAS)
#define GNSS_TARGET_HZ 10.0f  // runtime-verified; constellations reduced if
                              // the module can't hold it (sensors.cpp)
#define GNSS_POLL_HZ 100      // checkUblox() call rate (library throttles I2C)
#define GNSS_MIN_SV 6
#define GNSS_MAX_HACC_M 25.0f // fix-level validation ceiling for updates
#define ANCHOR_MAX_HACC_M 5.0f// stricter gate to anchor the NED origin
#define GNSS_LATENCY_S 0.06f  // NAV-PVT age at I2C read: ~solution latency +
                              // stream/poll delay; no PPS is wired (TIMEPULSE
                              // only drives an LED) so this constant is the
                              // explicit latency model, compensated in eskf
#define GNSS_REACQ_GAP_S 2.0f // outage longer than this triggers reacq gating
#define GNSS_REACQ_N 3        // fixes that must pass gates at inflated R
#define GNSS_REACQ_R_MULT 25.0f

// ---- GPS usage mode (compile-time) ---------------------------------------
// 0 = GPS_OFF      GNSS never initialized or polled; orientation-only.
// 1 = GPS_AUTO     (default) never blocks on GPS: if no valid fix arrives
//                  within GPS_AUTO_DECIDE_S of boot, drop to orientation-
//                  only (fst 4) so attitude is useful indoors. A valid fix
//                  arriving later still anchors and upgrades to full
//                  navigation (fst 4 -> 3) — cold-start TTFF routinely
//                  exceeds this window, so the late fix is the normal case.
// 2 = GPS_REQUIRED wait for a fix indefinitely (original behavior).
// Orientation-only = attitude/heading with mag + stillness-gated gravity
// aiding, plus the baro-damped vertical channel (altitude + vertical rate);
// horizontal position/velocity stay null and fst reports 4.
#define GPS_MODE_SEL 1
#define GPS_AUTO_DECIDE_S 20.0f

// ---- mounting rotations (per-sensor, sensor frame -> body frame) ---------
// Body frame: bench-verify with the vehicle stationary and upright — the
// calibrated accel must read ~= -9.81 m/s^2 on body Z (NED, Z-down) with
// X/Y near zero, and the calibrated mag vector, rotated by the estimated
// attitude, must match the local NED reference below.
// ICM-45686 (settled 2026-08-25 after three bench iterations — the first
// two "fixes" chased a viewer bug that flipped the screen vertical, not a
// frame error here): the KittyV1-verified mapping stands. The package +Z
// points AFT, so body X (nose) = -sensor Z, body Y = +sensor X,
// body Z = -sensor Y (det = +1). Do not change this without a
// two-orientation static check through a verified display path.
static const float R_MOUNT_IMU[9] = { 0, 0, -1,
                                      1, 0, 0,
                                      0, -1, 0 };
// LIS3MDL (sensor board, measured 2026-08-21): body X = -sensor Z,
// body Y = -sensor X, body Z = +sensor Y (det = +1). Verify heading against
// a known bearing; the inclination gate rejects a wrong mag frame rather
// than letting it corrupt attitude.
static const float R_MOUNT_MAG[9] = { 0, 0, -1,
                                      -1, 0, 0,
                                      0, 1, 0 };

// ---- fixed pre-calibration (applied after remap, body frame) -------------
static const float GYRO_BIAS0[3] = { 0, 0, 0 };   // rad/s (filter estimates
                                                  // the residual live)
static const float ACCEL_BIAS0[3] = { 0, 0, 0 };  // m/s^2
static const float ACCEL_SCALE[3] = { 1, 1, 1 };  // unitless
// Mag hard/soft iron, BODY frame (paste a MAGCAL sweep here).
static const float MAG_HARD_UT[3] = { 0, 0, 0 };
static const float MAG_SOFT[9] = { 1, 0, 0,
                                   0, 1, 0,
                                   0, 0, 1 };

// Live hard-iron offset: restored from the flash config store at boot when
// a $magcal result was saved there, else the compile-time MAG_HARD_UT.
static float mag_hard_active_[3] = { 0, 0, 0 };
static bool mag_hard_stored_ = false;
static uint32_t magcal_prog_next_s_ = 10;  // next progress heartbeat

// ---- local magnetic field (Champaign IL 61822, IGRF-14, Aug 2026) --------
// magnitude 51.86 uT, declination -3.50 deg (west), inclination +67.09 (down)
static const float MAG_REF_NED[3] = { 20.15f, -1.23f, 47.77f };  // uT

// ---- filter noise/covariance configuration -------------------------------
// Every number below is tied to a datasheet figure or a stated derivation.
#define GYRO_ND 6.63e-5f      // rad/s/rtHz = 0.0038 dps/rtHz, ICM-45686
                              // DS-000489 Table 1 "Rate Noise Spectral
                              // Density", typ
#define ACCEL_ND 1.08e-3f     // m/s^2/rtHz = 110 ug/rtHz at the +/-32g FSR,
                              // DS-000489 Table 2 (the 32 g range is noisier
                              // than 70 ug/rtHz at <=8 g — priced in)
#define GYRO_BIAS_RW 1.5e-4f  // rad/s/rt(s): deliberately ABOVE the
                              // DS-000489 tempco floor so the still-vehicle
                              // zero-rate trim can track self-heating drift
                              // — at 2e-5 the bias was so stiff a real
                              // thermal shift out-ran the filter and the
                              // attitude aids gated themselves out
#define ACCEL_BIAS_RW 3e-4f   // m/s^2/rt(s): DS-000489 Table 2 zero-g tempco
                              // +/-0.15 mg/degC, same 5 degC/10 min envelope
#define BARO_BIAS_RW 0.0f     // pinned with INIT_BB_STD: weather drift
                              // moves the datum itself, there is no second
                              // vertical reference to estimate it against
#define VIB_GYRO_MULT 2.0f    // unmodeled flight vibration inflation; the
#define VIB_ACCEL_MULT 5.0f   // datasheet floors are bench numbers — tune
                              // per vehicle from flight NIS
#define SAT_Q_MULT 20.0f      // accel clipped: velocity process noise x400
                              // in variance, degrade gracefully not diverge
#define POS_Q_FLOOR 1e-3f     // m/rt(s) numerical floor on position

#define INIT_POS_STD_MIN 2.0f    // m, floor under reported hAcc/vAcc
#define INIT_VEL_STD_MIN 0.3f    // m/s, floor under reported sAcc
#define INIT_RP_STD 0.035f       // rad (2 deg) post static alignment
#define INIT_YAW_STD_MAG 0.087f  // rad (5 deg): mag heading incl. iron resid
#define INIT_YAW_STD_NOMAG 1.0f  // rad: MAG_OFF, heading unknown
#define INIT_BG_STD 0.01f        // rad/s: DS-000489 Table 1 ZRO +/-0.4 dps
                                 // board-level = 7e-3 rad/s, plus margin
#define INIT_BA_STD 0.2f         // m/s^2: DS-000489 Table 2 zero-g +/-20 mg
#define INIT_BB_STD 0.02f        // m: bb is PINNED — with GNSS out of the
                                 // vertical the baro itself defines the
                                 // datum and a bias state is unobservable

#define ALIGN_DURATION_S 2.5f
#define ALIGN_TIMEOUT_S 20.0f    // then accept a degraded (moving) init
#define STILL_GYRO_STD 0.02f     // rad/s per-axis — stillness needs gyro AND
#define STILL_ACCEL_STD 0.35f    // m/s^2 per-axis — accel; |a|~g alone is
#define STILL_NORM_TOL 0.5f      // m/s^2 — blind to horizontal acceleration

// Innovation gates: chi-square 99% for the measurement dof.
#define GATE_GPS_POS 9.21f       // 2 dof: N/E only — the vertical channel
#define GATE_GPS_VEL 9.21f       // 2 dof   is baro+IMU, GNSS never fuses alt
#define GATE_BARO 6.63f          // 1 dof
#define GATE_MAG 11.34f          // 3 dof (full-vector mag update)
#define GATE_GRAV 9.21f          // 2 dof

#define GPS_POS_R_FLOOR 0.5f     // m — never trust hAcc below this
#define GPS_VEL_R_FLOOR 0.1f     // m/s

// Baro measurement noise model: base is BME280 electrical noise (~0.2 Pa
// RMS at high oversampling, BST-BME280-DS002) plus static-port installation
// error; the speed^2 term models dynamic-pressure port error; the Gaussian
// bump in Mach covers the transonic static-port excursion (tens of meters).
// All of it is R inflation driven by the ESTIMATED state — no flight-phase
// logic anywhere in the baro path.
#define BARO_R_BASE 0.7f         // m
#define BARO_R_DYN_K 2e-4f       // m per (m/s)^2
#define TRANSONIC_VAR 900.0f     // m^2 added at the bump center
#define TRANSONIC_M0 1.0f
#define TRANSONIC_SIGMA_M 0.12f
#define BARO_LATENCY_S 0.02f     // s: the x8/x1 conversion integrates over
                                 // ~20 ms and the data register holds the
                                 // last completed conversion — mean sample
                                 // age ~20 ms at the 25 Hz poll, compensated
                                 // in the filter like the GNSS fix latency

// Mag measurement: sensor RMS noise 3.2 mgauss = 0.32 uT (ST LIS3MDL DS,
// UHP mode) plus calibration residual -> 0.5 uT effective, applied PER AXIS
// by the 3-axis vector update. MAG_R_FLOOR is retained for config layout
// but unused by the vector form.
#define MAG_NOISE_UT 0.5f
#define MAG_R_FLOOR 7.6e-5f      // rad^2
#define MAG_NORM_TOL_FRAC 0.15f  // field-magnitude gate
#define MAG_INCL_TOL_RAD 0.175f  // inclination gate (10 deg)

#define GRAV_MEAS_STD 0.01f      // rad tilt 1-sigma, stillness-gated aiding
#define GRAV_DECIM 40            // ~10 Hz at the 400 Hz IMU rate
#define ZARU_MEAS_STD 0.003f     // rad/s: zero-rate gyro-bias trim while
                                 // still — kills pad attitude drift at the
                                 // source (mag-independent)
#define GATE_ZARU 11.34f         // 3 dof chi-square 99%
#define ATT_STD_FLOOR 2e-3f      // rad (~0.11 deg): systematic mount and
                                 // reference-vector error never averages
                                 // down, however dense the aiding

#define G_DEFAULT 9.80665f       // pre-anchor gravity; replaced by WGS-84
                                 // Somigliana at the anchor latitude

// ============================================================================
// END CONFIG BLOCK
// ============================================================================

static Sensors sensors;
static eskf::Eskf filter;
static eskf::Config eskf_cfg_;   // built once in setup, reused by $cal
static Sx1278 lora;
static Servos servos;
static ctl::Control control;
static uint64_t arm_pend_us_ = 0;  // $arm two-step confirmation window
static int lcal_fin_ = -1;         // linkage-cal session fin, -1 = idle
static LinkageCal lcal_stage_;     // points collected this session
static uint8_t lcal_fins_ = 0;     // fins with a STORED calibration
static int ang_scan_last_ = -1;    // last announced scan channel
static int ctest_last_phase_ = -1; // last announced canard-test phase

// Compile-time mag mode plumbing.
static const eskf::MagMode kMagMode =
    (MAG_MODE_SEL == 0) ? eskf::MAG_OFF
                        : ((MAG_MODE_SEL == 1) ? eskf::MAG_INIT_ONLY
                                               : eskf::MAG_CONTINUOUS);
static const char *kMagModeStr =
    (MAG_MODE_SEL == 0) ? "off" : ((MAG_MODE_SEL == 1) ? "init" : "cont");

// ---- monotonic 64-bit microsecond time base ----
static uint64_t mono_us_ = 0;
static uint32_t last_micros_ = 0;
static uint64_t monoNow() {
  uint32_t m = micros();
  mono_us_ += (uint32_t)(m - last_micros_);  // wrap-safe unsigned delta
  last_micros_ = m;
  return mono_us_;
}

// ---- cooperative scheduler ----
// The IMU poll owns the loop; between IMU due-times exactly one secondary
// task may run, and only if its worst-case bus time fits in the remaining
// slack — sensor I/O never stalls the filter step. Costs in us, measured
// from the transaction arithmetic in the bus-budget comment above.
static const uint32_t IMU_POLL_US = 1000000ul / IMU_POLL_HZ;
static const uint32_t MAG_POLL_US = 1000000ul / MAG_POLL_HZ;
static const uint32_t RADIO_POLL_US = 4000;   // 250 Hz DIO0/IRQ-flag poll
// Servo write cadence: SERVO_WRITE_PER_FRAME per PWM frame (2.5 ms at the
// 100 Hz default), so a fresh control output waits at most a quarter frame
// for its write instead of a whole one. Runtime variable: $sframe rescales
// it together with the frame rate.
static uint32_t servo_poll_us_ =
    (uint32_t)(1000000.0f / (SERVO_FRAME_HZ * (float)SERVO_WRITE_PER_FRAME));
static const uint32_t BARO_POLL_US = 1000000ul / BARO_POLL_HZ;
static const uint32_t GNSS_POLL_US = 1000000ul / GNSS_POLL_HZ;
static const uint32_t TELEM_PERIOD_US = 1000000ul / TELEM_HZ;
static const uint32_t SERVICE_PERIOD_US = 100000ul;  // 10 Hz watchdogs
static const int32_t COST_MAG_US = 500;
static const int32_t COST_BARO_US = 800;
// Must fit the ~1400 us of slack that remains after a 440 Hz IMU poll —
// 1700 could never be satisfied, so the GNSS task starved permanently. A
// worst-case backlog drain can spill ~1 ms into the next IMU slot; the
// DRDY stale-skip and measured-dt logic absorb that.
static const int32_t COST_GNSS_US = 1200;
static const int32_t COST_TELEM_US = 700;
static const int32_t COST_SERVICE_US = 600;  // includes one LED strip push
static const int32_t COST_RADIO_US = 400;    // flag poll or one FIFO load
static const int32_t COST_SERVO_US = 700;    // up to 4 short PCA writes
                                             // (<=120 us, IRQs masked)
static const int32_t SLACK_MARGIN_US = 150;

static uint64_t next_imu_ = 0, next_mag_ = 0, next_baro_ = 0, next_gnss_ = 0,
                next_telem_ = 0, next_service_ = 0, next_radio_ = 0,
                next_servo_ = 0;
// PCA9685 re-probe with backoff (10 Hz service slot): the servo controller
// gets the same recovery machinery every sensor on this bus has, instead of
// one boot probe deciding the whole session.
static uint64_t pca_retry_next_us_ = 0;
static uint32_t pca_retry_backoff_ms_ = 1000;

// ---- bookkeeping ----
static uint32_t seq_ = 0;
static uint32_t lmx_us_ = 0;          // max loop period since last record
static uint64_t loop_prev_us_ = 0;
static uint32_t prop_last_count_ = 0;
static uint64_t prop_last_us_ = 0;
static float fhz_ = 0;
static bool origin_emitted_ = false;
static uint64_t last_gps_accept_us_ = 0;
static int reacq_countdown_ = 0;
static bool gps_decided_off_ = false;  // att-only entered once per boot
                                       // (late-fix upgrade stays allowed)
static uint8_t fst_seen_ = 0;          // last state the startup reporter saw
static uint64_t align_t0_us_ = 0;      // align-window start, for cal duration
static bool cal_emitted_ = false;

// ----------------------------------------------------------------------------

static void buildEskfConfig(eskf::Config &c) {
  c.gyro_nd = GYRO_ND;
  c.accel_nd = ACCEL_ND;
  c.gyro_bias_rw = GYRO_BIAS_RW;
  c.accel_bias_rw = ACCEL_BIAS_RW;
  c.baro_bias_rw = BARO_BIAS_RW;
  c.vib_gyro_mult = VIB_GYRO_MULT;
  c.vib_accel_mult = VIB_ACCEL_MULT;
  c.sat_q_mult = SAT_Q_MULT;
  c.pos_q_floor = POS_Q_FLOOR;
  c.init_pos_std_min = INIT_POS_STD_MIN;
  c.init_vel_std_min = INIT_VEL_STD_MIN;
  c.init_rp_std = INIT_RP_STD;
  c.init_yaw_std_mag = INIT_YAW_STD_MAG;
  c.init_yaw_std_nomag = INIT_YAW_STD_NOMAG;
  c.init_bg_std = INIT_BG_STD;
  c.init_ba_std = INIT_BA_STD;
  c.init_bb_std = INIT_BB_STD;
  c.align_duration_s = ALIGN_DURATION_S;
  c.align_timeout_s = ALIGN_TIMEOUT_S;
  c.still_gyro_std_max = STILL_GYRO_STD;
  c.still_accel_std_max = STILL_ACCEL_STD;
  c.still_acc_norm_tol = STILL_NORM_TOL;
  c.imu_dt_nom = 1.0f / (float)IMU_ODR_HZ;
  c.gate_gps_pos = GATE_GPS_POS;
  c.gate_gps_vel = GATE_GPS_VEL;
  c.gate_baro = GATE_BARO;
  c.gate_mag = GATE_MAG;
  c.gate_grav = GATE_GRAV;
  c.gps_pos_r_floor = GPS_POS_R_FLOOR;
  c.gps_vel_r_floor = GPS_VEL_R_FLOOR;
  c.gps_latency_s = GNSS_LATENCY_S;
  c.reacq_r_mult = GNSS_REACQ_R_MULT;
  c.baro_r_base = BARO_R_BASE;
  c.baro_r_dyn_k = BARO_R_DYN_K;
  c.transonic_var = TRANSONIC_VAR;
  c.transonic_m0 = TRANSONIC_M0;
  c.transonic_sigma_m = TRANSONIC_SIGMA_M;
  c.baro_latency_s = BARO_LATENCY_S;
  c.mag_mode = kMagMode;
  c.mag_ref_ned[0] = MAG_REF_NED[0];
  c.mag_ref_ned[1] = MAG_REF_NED[1];
  c.mag_ref_ned[2] = MAG_REF_NED[2];
  c.mag_noise_ut = MAG_NOISE_UT;
  c.mag_r_floor = MAG_R_FLOOR;
  c.mag_norm_tol_frac = MAG_NORM_TOL_FRAC;
  c.mag_incl_tol_rad = MAG_INCL_TOL_RAD;
  c.grav_meas_std = GRAV_MEAS_STD;
  c.grav_decim = GRAV_DECIM;
  c.zaru_meas_std = ZARU_MEAS_STD;
  c.gate_zaru = GATE_ZARU;
  c.att_var_floor = ATT_STD_FLOOR * ATT_STD_FLOOR;
  c.g_default = G_DEFAULT;
}

static void buildSensorsConfig(SensorsConfig &s) {
  s.imu_addr = IMU_I2C_ADDR;
  s.mag_addr = MAG_I2C_ADDR;
  s.baro_addr = BARO_I2C_ADDR;
  s.gnss_addr = GNSS_I2C_ADDR;
  s.i2c_hz = I2C_HZ;
  // SDA/SCL are the variant's Wire pins (PB7/PB6 here) for the bus-clear.
  s.sda_pin = (int16_t)SDA;
  s.scl_pin = (int16_t)SCL;
  s.accel_fs = IMU_ACCEL_FS;
  s.gyro_fs = IMU_GYRO_FS;
  s.imu_odr = IMU_ODR;
  s.accel_lpf = IMU_ACCEL_LPF;
  s.gyro_lpf = IMU_GYRO_LPF;
  s.imu_discard_ms = IMU_DISCARD_MS;
  s.imu_mount = R_MOUNT_IMU;
  s.mag_mount = R_MOUNT_MAG;
  s.gyro_bias0 = GYRO_BIAS0;
  s.accel_bias0 = ACCEL_BIAS0;
  s.accel_scale = ACCEL_SCALE;
  s.mag_hard_ut = mag_hard_active_;  // flash-restored, else MAG_HARD_UT
  s.mag_soft = MAG_SOFT;
  s.gnss_meas_ms = GNSS_MEAS_MS;
  s.gnss_target_hz = GNSS_TARGET_HZ;
  s.gnss_min_sv = GNSS_MIN_SV;
  s.gnss_max_hacc_m = GNSS_MAX_HACC_M;
  s.gnss_enabled = (GPS_MODE_SEL != 0);
}

// ---- ground command input -------------------------------------------------
// The telemetry serial is bidirectional: newline-terminated "$cmd arg..."
// lines from the ground station. Every command is answered with a "msg"
// record (txt starting "cmd:"), so replies ride the normal NDJSON stream
// into the viewer console and any log. See SCHEMA.md, "Command input".
static char cmd_buf_[64];
static uint8_t cmd_len_ = 0;
static bool cmd_drop_ = false;  // overlong line: discard through its newline

static void startRecal(uint64_t now) {
  // Full navigation restart: alignment (gyro offset + attitude) re-averages,
  // orchestrate() re-collects and re-freezes the baro reference (its state
  // machine keys off FS_ALIGN), and the origin re-anchors on the next valid
  // fix. Identical to a power cycle minus the boot probes.
  telem::emitMsg(now, "cmd: recalibrating - hold still");
  filter.init(eskf_cfg_);
  origin_emitted_ = false;
  gps_decided_off_ = false;
  cal_emitted_ = false;
  align_t0_us_ = 0;
  fst_seen_ = 0;
  reacq_countdown_ = 0;
  last_gps_accept_us_ = 0;
}

static uint8_t parseChan(const char *s) {
  const long v = strtol(s, nullptr, 10);
  return (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v));
}

// Controller authority = min(compile-time clamp, smallest linkage-table
// end). ONE definition, applied at boot and after every linkage change —
// three pasted copies of a safety clamp is how one goes stale.
static float applyAuthority() {
  float auth = servos.authorityDeg();
  if (auth > CTL_DEFL_MAX) auth = CTL_DEFL_MAX;
  control.setDeflMax(auth);
  return auth;
}

// Structured dump of the linkage tables + staged points as "lcal" records;
// the viewer's point list and graph re-render on every one.
static void lcalDump(uint64_t now) {
  for (int i = 0; i < 4; ++i) {
    const LinkageCal &lc = servos.linkage()[i];
    telem::emitLcal(now, (int8_t)i, (lcal_fins_ & (1u << i)) ? 1 : 0, lc.n,
                    lc.deg, lc.us);
  }
  if (lcal_fin_ >= 0) {
    telem::emitLcal(now, (int8_t)lcal_fin_, 2, lcal_stage_.n,
                    lcal_stage_.deg, lcal_stage_.us);
  } else {
    float z[5] = { 0, 0, 0, 0, 0 };
    telem::emitLcal(now, -1, 2, 0, z, z);  // no session: stage cleared
  }
}

static void execCommand(char *line) {
  const uint64_t now = monoNow();
  char *save = nullptr;
  char *cmd = strtok_r(line + 1, " ", &save);
  if (cmd == nullptr) { telem::emitMsg(now, "cmd: unknown"); return; }
  if (strcmp(cmd, "ping") == 0) {
    telem::emitMsg(now, "cmd: pong");
    return;
  }
  if (strcmp(cmd, "cal") == 0) {
    if (control.mode() != ctl::CM_IDLE && control.mode() != ctl::CM_SAFE) {
      telem::emitMsg(now, "cmd: cal refused - $disarm first");
      return;
    }
    startRecal(now);
    return;
  }
  if (strcmp(cmd, "magcal") == 0) {
    if (control.mode() != ctl::CM_IDLE && control.mode() != ctl::CM_SAFE) {
      telem::emitMsg(now, "cmd: magcal refused - $disarm first");
      return;
    }
    sensors.magCalStart(now);
    telem::emitMsg(now,
        "cmd: magcal - rotate slowly through all orientations for 30 s");
    return;
  }
  if (strcmp(cmd, "magdiag") == 0) {
    // Register-level probe, independent of the driver and its init state:
    // raw WHO_AM_I / control / status values plus the same six output
    // registers read three different ways. The read-strategy comparison
    // pins a wrong-looking field on the silicon, the bus protocol, or the
    // driver in one shot. Blocks ~60 ms (bench command).
    MagDiag d;
    sensors.magDiag(d);
    char buf[112];
    snprintf(buf, sizeof(buf), "magdiag: who 0x%02X (%s) buserr %u",
             d.who, d.who_ok ? (d.who == 0x3D ? "ok" : "WRONG - expect 0x3D")
                             : "READ FAILED",
             (unsigned)d.bus_err);
    telem::emitMsg(now, buf);
    snprintf(buf, sizeof(buf),
             "magdiag: ctrl %02X %02X %02X %02X %02X (want 7C 00 00 0C 40) "
             "status %02X",
             d.ctrl[0], d.ctrl[1], d.ctrl[2], d.ctrl[3], d.ctrl[4], d.status);
    telem::emitMsg(now, buf);
    snprintf(buf, sizeof(buf),
             "magdiag: burst-noinc %d %d %d | burst-inc %d %d %d",
             d.noinc[0], d.noinc[1], d.noinc[2], d.inc[0], d.inc[1], d.inc[2]);
    telem::emitMsg(now, buf);
    snprintf(buf, sizeof(buf),
             "magdiag: single %d %d %d | zyxda %u/%u changed %u/%u",
             d.single[0], d.single[1], d.single[2], (unsigned)d.zyxda_n,
             (unsigned)MagDiag::kDiagPolls, (unsigned)d.change_n,
             (unsigned)(MagDiag::kDiagPolls - 1));
    telem::emitMsg(now, buf);
    // Distilled verdict so the bench answer is readable at a glance.
    if (!d.who_ok || d.who != 0x3D) {
      telem::emitMsg(now,
          "magdiag: VERDICT no LIS3MDL answering at 0x1C - wiring/address");
    } else if (d.zyxda_n == 0) {
      telem::emitMsg(now,
          "magdiag: VERDICT sensor not sampling (ZYXDA never set) - config");
    } else if (d.change_n == 0) {
      telem::emitMsg(now,
          "magdiag: VERDICT output frozen while sampling - wedged silicon");
    } else if (d.noinc[0] == d.noinc[1] && d.noinc[1] == d.noinc[2] &&
               !(d.inc[0] == d.inc[1] && d.inc[1] == d.inc[2])) {
      // Without SUB(7) the part serves OUT_X_L for all six bytes: the noinc
      // burst collapses to three identical words while the inc burst shows a
      // real vector. (Noise makes back-to-back bursts differ, so comparing
      // noinc against inc directly would false-alarm.)
      telem::emitMsg(now,
          "magdiag: VERDICT auto-inc bit required - old driver read garbage");
    } else {
      telem::emitMsg(now,
          "magdiag: VERDICT registers healthy - rotate and watch mgr");
    }
    return;
  }
  if (strcmp(cmd, "magclr") == 0) {
    if (control.mode() != ctl::CM_IDLE && control.mode() != ctl::CM_SAFE) {
      telem::emitMsg(now, "cmd: magclr refused - $disarm first");
      return;
    }
    static const float z[3] = { 0, 0, 0 };
    // ~1-2 s stall (sector rewrite); the linkage record is preserved.
    bool ok = cfgstore::saveMagHard(z);
    sensors.magHardSet(z);
    for (int i = 0; i < 3; ++i) mag_hard_active_[i] = 0;
    telem::emitMsg(now, ok ? "cmd: magclr - stored mag cal erased; send "
                             "$cal to re-align"
                           : "cmd: magclr - flash erase FAILED");
    return;
  }
  if (strcmp(cmd, "lcal") == 0) {
    if (control.mode() != ctl::CM_IDLE) {
      telem::emitMsg(now, "cmd: lcal refused - only while IDLE");
      return;
    }
    if (!servos.present()) {
      telem::emitMsg(now, "cmd: lcal - PCA9685 not present");
      return;
    }
    char *a = strtok_r(nullptr, " ", &save);
    if (a == nullptr) {
      telem::emitMsg(now, "cmd: lcal wants start|jog|goto|point|undo|del|"
                          "edit|load|check|save|show|clear|abort|dump|flash");
      return;
    }
    char rbuf[112];
    if (strcmp(a, "show") == 0) {
      for (int i = 0; i < 4; ++i) {
        const LinkageCal &lc = servos.linkage()[i];
        char *p = rbuf;
        char *pe = rbuf + sizeof(rbuf);
        p += snprintf(p, (size_t)(pe - p), "lcal S%d %s n=%d degx10", i,
                      (lcal_fins_ & (1u << i)) ? "CAL" : "default",
                      (int)lc.n);
        for (int k = 0; k < (int)lc.n && p < pe; ++k) {
          p += snprintf(p, (size_t)(pe - p), " %ld",
                        lroundf(lc.deg[k] * 10.0f));
        }
        if (p < pe) p += snprintf(p, (size_t)(pe - p), " us");
        for (int k = 0; k < (int)lc.n && p < pe; ++k) {
          p += snprintf(p, (size_t)(pe - p), " %ld", lroundf(lc.us[k]));
        }
        telem::emitMsg(now, rbuf);
      }
      return;
    }
    if (strcmp(a, "dump") == 0) {
      lcalDump(now);
      return;
    }
    if (strcmp(a, "flash") == 0) {
      // Read the physical sector past the cache: the truth about whether
      // the last save actually persisted (run it after a power cycle).
      cfgstore::FlashInfo fi;
      cfgstore::inspectRaw(&fi);
      const char *ver = fi.version == 2 ? "RNV2" : fi.version == 1 ? "RNV1"
                        : (fi.magic == 0xFFFFFFFFUL ? "blank" : "corrupt");
      snprintf(rbuf, sizeof(rbuf),
               "lcal flash: %s crc %s - fin pts %d %d %d %d - mag %s", ver,
               fi.crc_ok ? "OK" : "bad", fi.fin_n[0], fi.fin_n[1],
               fi.fin_n[2], fi.fin_n[3], fi.mag_ok ? "stored" : "none");
      telem::emitMsg(now, rbuf);
      return;
    }
    if (strcmp(a, "clear") == 0) {
      char *v = strtok_r(nullptr, " ", &save);
      if (v == nullptr) {
        telem::emitMsg(now, "cmd: lcal clear wants <fin 0-3|all>");
        return;
      }
      LinkageCal all[4];
      for (int i = 0; i < 4; ++i) {
        all[i] = servos.linkage()[i];
        if (!(lcal_fins_ & (1u << i))) all[i].n = 0;  // keep defaults out
      }
      if (strcmp(v, "all") == 0) {
        for (int i = 0; i < 4; ++i) all[i].n = 0;
        lcal_fins_ = 0;
      } else {
        int f = atoi(v);
        if (f < 0 || f > 3 || (f == 0 && v[0] != '0')) {
          telem::emitMsg(now, "cmd: lcal clear wants <fin 0-3|all>");
          return;
        }
        all[f].n = 0;
        lcal_fins_ &= (uint8_t)~(1u << f);
      }
      telem::emitMsg(now, "lcal: saving to flash - board freezes ~2 s");
      TELEM_SERIAL.flush();
      bool ok = cfgstore::saveLinkage(all);
      servos.setLinkage(all);
      applyAuthority();
      telem::emitMsg(now, ok ? "lcal: cleared to defaults"
                             : "lcal: flash save FAILED");
      lcalDump(now);
      return;
    }
    if (strcmp(a, "start") == 0) {
      char *v = strtok_r(nullptr, " ", &save);
      int f = (v != nullptr) ? atoi(v) : -1;
      if (f < 0 || f > 3 || (f == 0 && v[0] != '0')) {
        telem::emitMsg(now, "cmd: lcal start wants <fin 0-3>");
        return;
      }
      lcal_fin_ = f;
      lcal_stage_.n = 0;
      servos.testUs(f, servos.deflToUs(f, 0.0f));
      snprintf(rbuf, sizeof(rbuf),
               "lcal: S%d session at %ld us - jog until the canard is flush"
               " (0 deg), then 'point 0'",
               f, lroundf(servos.targetUs()[f]));
      telem::emitMsg(now, rbuf);
      lcalDump(now);
      return;
    }
    if (strcmp(a, "abort") == 0) {
      lcal_fin_ = -1;
      lcal_stage_.n = 0;
      servos.center();
      telem::emitMsg(now, "lcal: session ended - canards centered");
      lcalDump(now);
      return;
    }
    if (lcal_fin_ < 0) {
      telem::emitMsg(now, "cmd: lcal - no session ($lcal start <fin>)");
      return;
    }
    if (strcmp(a, "jog") == 0 || strcmp(a, "goto") == 0) {
      char *v = strtok_r(nullptr, " ", &save);
      if (v == nullptr) {
        telem::emitMsg(now, "cmd: lcal jog/goto wants <us>");
        return;
      }
      float cur = servos.targetUs()[lcal_fin_];
      float us = (a[0] == 'j') ? cur + (float)atof(v) : (float)atof(v);
      servos.testUs(lcal_fin_, us);
      snprintf(rbuf, sizeof(rbuf), "lcal: S%d at %ld us", lcal_fin_,
               lroundf(servos.targetUs()[lcal_fin_]));
      telem::emitMsg(now, rbuf);
      return;
    }
    if (strcmp(a, "point") == 0) {
      char *v = strtok_r(nullptr, " ", &save);
      if (v == nullptr) {
        telem::emitMsg(now, "cmd: lcal point wants <measured canard deg>");
        return;
      }
      if (lcal_stage_.n >= 5) {
        telem::emitMsg(now, "lcal: 5 points max - undo one or save");
        return;
      }
      if (servos.benchBusy()) {
        // A sweep/scan/fin-test owns tgt_us_ right now: the pulse this
        // would stage is not the one holding the fin you measured.
        telem::emitMsg(now,
            "lcal: point refused - a servo test is driving the fins (jog "
            "first or let it finish)");
        return;
      }
      float deg = (float)atof(v);
      if (!(deg > -89.0f && deg < 89.0f)) {
        // Same bound the flash loader enforces (finSane): accepting more
        // here would save a table that silently reverts at reboot.
        telem::emitMsg(now, "lcal: angle out of range (+/-89 deg)");
        return;
      }
      lcal_stage_.deg[lcal_stage_.n] = deg;
      lcal_stage_.us[lcal_stage_.n] = servos.targetUs()[lcal_fin_];
      lcal_stage_.n++;
      snprintf(rbuf, sizeof(rbuf),
               "lcal: point %d of 5 staged - x10 %ld deg at %ld us",
               (int)lcal_stage_.n, lroundf(deg * 10.0f),
               lroundf(servos.targetUs()[lcal_fin_]));
      telem::emitMsg(now, rbuf);
      lcalDump(now);
      return;
    }
    if (strcmp(a, "undo") == 0) {
      if (lcal_stage_.n > 0) lcal_stage_.n--;
      snprintf(rbuf, sizeof(rbuf), "lcal: %d point(s) staged",
               (int)lcal_stage_.n);
      telem::emitMsg(now, rbuf);
      lcalDump(now);
      return;
    }
    if (strcmp(a, "del") == 0) {
      char *v = strtok_r(nullptr, " ", &save);
      int idx = (v != nullptr) ? atoi(v) : -1;
      if (idx < 0 || idx >= (int)lcal_stage_.n || (idx == 0 && v[0] != '0')) {
        telem::emitMsg(now, "cmd: lcal del wants <staged point index>");
        return;
      }
      for (int i = idx; i + 1 < (int)lcal_stage_.n; ++i) {
        lcal_stage_.deg[i] = lcal_stage_.deg[i + 1];
        lcal_stage_.us[i] = lcal_stage_.us[i + 1];
      }
      lcal_stage_.n--;
      snprintf(rbuf, sizeof(rbuf), "lcal: point %d deleted - %d staged", idx,
               (int)lcal_stage_.n);
      telem::emitMsg(now, rbuf);
      lcalDump(now);
      return;
    }
    if (strcmp(a, "edit") == 0) {
      char *v = strtok_r(nullptr, " ", &save);
      char *dv = strtok_r(nullptr, " ", &save);
      int idx = (v != nullptr) ? atoi(v) : -1;
      if (idx < 0 || idx >= (int)lcal_stage_.n || dv == nullptr ||
          (idx == 0 && v[0] != '0')) {
        telem::emitMsg(now, "cmd: lcal edit wants <index> <deg>");
        return;
      }
      float ndeg = (float)atof(dv);
      if (!(ndeg > -89.0f && ndeg < 89.0f)) {
        telem::emitMsg(now, "lcal: angle out of range (+/-89 deg)");
        return;
      }
      lcal_stage_.deg[idx] = ndeg;
      snprintf(rbuf, sizeof(rbuf),
               "lcal: point %d = x10 %ld deg (at %ld us)", idx,
               lroundf(lcal_stage_.deg[idx] * 10.0f),
               lroundf(lcal_stage_.us[idx]));
      telem::emitMsg(now, rbuf);
      lcalDump(now);
      return;
    }
    if (strcmp(a, "load") == 0) {
      lcal_stage_ = servos.linkage()[lcal_fin_];
      snprintf(rbuf, sizeof(rbuf),
               "lcal: loaded S%d active table (%d pts) into the stage",
               lcal_fin_, (int)lcal_stage_.n);
      telem::emitMsg(now, rbuf);
      lcalDump(now);
      return;
    }
    if (strcmp(a, "check") == 0) {
      char *v = strtok_r(nullptr, " ", &save);
      if (v == nullptr) {
        telem::emitMsg(now, "cmd: lcal check wants <deg>");
        return;
      }
      float deg = (float)atof(v);
      float us = servos.deflToUs(lcal_fin_, deg);
      servos.testUs(lcal_fin_, us);
      snprintf(rbuf, sizeof(rbuf),
               "lcal: check S%d x10 %ld deg -> %ld us - measure the canard",
               lcal_fin_, lroundf(deg * 10.0f), lroundf(us));
      telem::emitMsg(now, rbuf);
      return;
    }
    if (strcmp(a, "save") == 0) {
      LinkageCal fin_tab;
      if (lcal_stage_.n == 1) {
        // One measured point = offset-only: keep the default gain, move the
        // zero so the canard reads what you measured. This is the common
        // "just fix my neutral" case, and it now SAVES instead of being
        // rejected for having too few points.
        servos.buildOffsetTable(lcal_fin_, lcal_stage_.deg[0],
                                lcal_stage_.us[0], fin_tab);
      } else {
        fin_tab = lcal_stage_;
      }
      if (!linkage::finalize(fin_tab)) {
        telem::emitMsg(now, "lcal: table invalid - need 1 point (offset) or "
                            "2-5 (angles >=0.5 deg apart, pulses monotonic)");
        return;
      }
      if (linkage::endAuthorityDeg(fin_tab) < 8.0f) {
        telem::emitMsg(now, "lcal: NOTE span under 8 deg per side - saving; "
                            "authority will be limited");
      }
      LinkageCal all[4];
      for (int i = 0; i < 4; ++i) {
        all[i] = servos.linkage()[i];
        // Synthesized defaults must not be baked into flash as if measured.
        if (i != lcal_fin_ && !(lcal_fins_ & (1u << i))) all[i].n = 0;
      }
      all[lcal_fin_] = fin_tab;
      telem::emitMsg(now, "lcal: saving to flash - board freezes ~2 s");
      TELEM_SERIAL.flush();
      bool ok = cfgstore::saveLinkage(all);
      // The measured table is APPLIED to the servos either way, so the
      // CAL/default flag must follow the live table, not the flash write:
      // after a failed save the reply says "RAM only" and $lcal flash shows
      // the truth — reporting the active measured table as "default" sent
      // operators back to recalibrate a fin that was already calibrated.
      lcal_fins_ |= (uint8_t)(1u << lcal_fin_);
      servos.setLinkage(all);
      float auth = applyAuthority();
      snprintf(rbuf, sizeof(rbuf),
               "lcal: S%d %s (%s) - authority x10 = %ld deg; run checks"
               " or $lcal abort",
               lcal_fin_, ok ? "saved" : "flash save FAILED (RAM only)",
               lcal_stage_.n == 1 ? "offset-only, default gain"
                                  : "measured curve",
               lroundf(auth * 10.0f));
      telem::emitMsg(now, rbuf);
      lcal_stage_.n = 0;
      lcalDump(now);
      return;
    }
    telem::emitMsg(now, "cmd: lcal - unknown subcommand");
    return;
  }
  if (strcmp(cmd, "cang") == 0) {
    // Canard angle through the linkage calibration: 0 deg = the calibrated
    // neutral (fin flush / "straight"), not the raw servo center. Fin-
    // indexed (0-3); $ang stays for raw per-channel servo debug.
    if (control.mode() != ctl::CM_IDLE) {
      telem::emitMsg(now, "cmd: cang refused - only while IDLE");
      return;
    }
    if (!servos.present()) {
      telem::emitMsg(now, "cmd: cang - PCA9685 not present");
      return;
    }
    char *a = strtok_r(nullptr, " ", &save);
    if (a == nullptr) {
      telem::emitMsg(now,
                     "cmd: cang wants <fin 0-3> <deg>|off, center, test, alloff");
      return;
    }
    char rbuf[96];
    if (strcmp(a, "center") == 0) {
      servos.center();  // all canards to calibrated 0 deg
      telem::emitMsg(now, "cmd: cang - canards to calibrated neutral");
      return;
    }
    if (strcmp(a, "test") == 0) {
      servos.canardTestStart(now);
      telem::emitMsg(now, "cmd: cang test - all fins +10 -> -10 -> 0 deg "
                          "(watch every fin move)");
      return;
    }
    if (strcmp(a, "alloff") == 0) {
      servos.off();
      telem::emitMsg(now, "cmd: cang - canards released (limp)");
      return;
    }
    int fin = atoi(a);
    if (fin < 0 || fin > 3 || (fin == 0 && a[0] != '0')) {
      telem::emitMsg(now, "cmd: cang - fin 0-3");
      return;
    }
    char *v = strtok_r(nullptr, " ", &save);
    if (v == nullptr) {
      telem::emitMsg(now, "cmd: cang wants <deg> or off");
      return;
    }
    if (strcmp(v, "off") == 0) {
      servos.releaseCanard(fin);
      snprintf(rbuf, sizeof(rbuf), "cmd: cang S%d released (limp)", fin);
      telem::emitMsg(now, rbuf);
      return;
    }
    float deg = (float)atof(v);
    float us = servos.deflToUs(fin, deg);
    servos.testUs(fin, us);
    snprintf(rbuf, sizeof(rbuf),
             "cmd: cang S%d = x10 %ld deg canard -> %ld us", fin,
             lroundf(deg * 10.0f), lroundf(us));
    telem::emitMsg(now, rbuf);
    return;
  }
  if (strcmp(cmd, "ang") == 0) {
    if (control.mode() != ctl::CM_IDLE) {
      telem::emitMsg(now, "cmd: ang refused - only while IDLE");
      return;
    }
    if (!servos.present()) {
      telem::emitMsg(now, "cmd: ang - PCA9685 not present");
      return;
    }
    char *a = strtok_r(nullptr, " ", &save);
    if (a == nullptr) {
      telem::emitMsg(now, "cmd: ang wants <ch 0-15> <deg 0-180>|off, "
                          "alloff, or scan");
      return;
    }
    if (strcmp(a, "alloff") == 0) {
      servos.pca().allOff();
      telem::emitMsg(now, "cmd: ang - all 16 channels released");
      return;
    }
    if (strcmp(a, "scan") == 0) {
      servos.scanStart(now);
      telem::emitMsg(now, "cmd: ang scan - wiggling ch 0..15 in turn "
                          "(~19 s); watch which servo moves on which "
                          "announcement");
      return;
    }
    int ch = atoi(a);
    if (ch < 0 || ch > 15 || (ch == 0 && a[0] != '0')) {
      telem::emitMsg(now, "cmd: ang - channel 0-15");
      return;
    }
    char *v = strtok_r(nullptr, " ", &save);
    if (v == nullptr) {
      telem::emitMsg(now, "cmd: ang wants <deg 0-180> or off");
      return;
    }
    char rbuf[80];
    if (strcmp(v, "off") == 0) {
      bool ok = servos.pca().setPwmUs((uint8_t)ch, 0);
      snprintf(rbuf, sizeof(rbuf), "cmd: ang ch %d released%s", ch,
               ok ? "" : " - FAILED");
      telem::emitMsg(now, rbuf);
      return;
    }
    float deg = (float)atof(v);
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    // Standard hobby mapping: 0-180 deg over 1000-2000 us (90 deg = 1500).
    float us = 1000.0f + deg * (1000.0f / 180.0f);
    bool ok = servos.pca().setPwmUs((uint8_t)ch, us);
    snprintf(rbuf, sizeof(rbuf), "cmd: ang ch %d = %ld deg (%ld us)%s", ch,
             lroundf(deg), lroundf(us), ok ? "" : " - FAILED");
    telem::emitMsg(now, rbuf);
    return;
  }
  if (strcmp(cmd, "servo") == 0) {
    if (control.mode() != ctl::CM_IDLE) {
      telem::emitMsg(now, "cmd: servo refused - only while IDLE");
      return;
    }
    if (!servos.present()) {
      telem::emitMsg(now, "cmd: servo - PCA9685 not present");
      return;
    }
    char *a = strtok_r(nullptr, " ", &save);
    if (a == nullptr) {
      telem::emitMsg(now, "cmd: servo wants <0-3> <us> | center | off | sweep");
      return;
    }
    if (strcmp(a, "center") == 0) {
      servos.center();
      telem::emitMsg(now, "cmd: servos centered");
      return;
    }
    if (strcmp(a, "off") == 0) {
      servos.off();
      telem::emitMsg(now, "cmd: servo pulses off (limp)");
      return;
    }
    if (strcmp(a, "sweep") == 0) {
      servos.sweepStart(now);
      telem::emitMsg(now, "cmd: servo sweep - 6 s, all canards");
      return;
    }
    char *b = strtok_r(nullptr, " ", &save);
    if (b == nullptr) {
      telem::emitMsg(now, "cmd: servo wants <0-3> <us>");
      return;
    }
    // Strict args, same '0'-guard as $cang/$ang: atoi("x") and a mistyped
    // pulse both read as 0, and 0 clamped to min_us slams a servo hard
    // against the guard stop from a typo.
    int idx = atoi(a);
    if (idx < 0 || idx > 3 || (idx == 0 && a[0] != '0')) {
      telem::emitMsg(now, "cmd: servo - canard 0-3");
      return;
    }
    float us = (float)atof(b);
    if (us < SERVO_MIN_US || us > SERVO_MAX_US) {
      telem::emitMsg(now, "cmd: servo - us 900-2100");
      return;
    }
    servos.testUs(idx, us);
    telem::emitMsg(now, "cmd: servo set");
    return;
  }
  if (strcmp(cmd, "arm") == 0) {
    char *a = strtok_r(nullptr, " ", &save);
    if (a != nullptr && strcmp(a, "yes") == 0) {
      if (arm_pend_us_ == 0 || now - arm_pend_us_ > 5000000ull) {
        arm_pend_us_ = 0;
        telem::emitMsg(now, "ctl: arm window expired - send $arm first");
        return;
      }
      arm_pend_us_ = 0;
      if (filter.state() < eskf::FS_WAIT_FIX) {
        telem::emitMsg(now, "ctl: arm refused - filter not aligned");
        return;
      }
      if (!servos.present()) {
        telem::emitMsg(now, "ctl: arm refused - PCA9685 not present");
        return;
      }
      if (!sensors.imuHealth().fresh) {
        telem::emitMsg(now, "ctl: arm refused - IMU not fresh");
        return;
      }
      if (control.armRequest()) {
        servos.center();
        telem::emitMsg(now, "ctl: ARMED - launch detect live");
      } else {
        telem::emitMsg(now, "ctl: arm refused - not IDLE ($disarm first)");
      }
      return;
    }
    arm_pend_us_ = now;
    telem::emitMsg(now, "ctl: send $arm yes within 5 s to ARM");
    return;
  }
  if (strcmp(cmd, "disarm") == 0) {
    control.disarm();
    servos.center();
    arm_pend_us_ = 0;
    telem::emitMsg(now, "ctl: DISARMED - IDLE, canards centered");
    return;
  }
  if (strcmp(cmd, "rolltest") == 0) {
    // Ground roll-hold test: run the real roll PID now, no arm/launch. Same
    // preconditions as $arm (aligned filter, servos present, fresh IMU) so it
    // never drives canards on stale attitude. $disarm stops it.
    if (filter.state() < eskf::FS_WAIT_FIX) {
      telem::emitMsg(now, "ctl: rolltest refused - filter not aligned");
      return;
    }
    if (!servos.present()) {
      telem::emitMsg(now, "ctl: rolltest refused - PCA9685 not present");
      return;
    }
    if (!sensors.imuHealth().fresh) {
      telem::emitMsg(now, "ctl: rolltest refused - IMU not fresh");
      return;
    }
    if (control.rollTestRequest()) {
      servos.center();  // start from calibrated neutral, then track roll
      arm_pend_us_ = 0;
      telem::emitMsg(now,
                     "ctl: ROLL TEST live - twist the airframe, canards fight "
                     "it; $disarm to stop");
    } else {
      telem::emitMsg(now, "ctl: rolltest refused - not IDLE ($disarm first)");
    }
    return;
  }
  if (strcmp(cmd, "ctl") == 0) {
    char *a = strtok_r(nullptr, " ", &save);
    char *b = strtok_r(nullptr, " ", &save);
    char *c = strtok_r(nullptr, " ", &save);
    if (a == nullptr || b == nullptr || c == nullptr) {
      telem::emitMsg(now, "cmd: ctl wants <kp_rate> <ki_rate> <kp_ang>");
      return;
    }
    float kpr = (float)atof(a), kir = (float)atof(b), kpa = (float)atof(c);
    control.setGains(kpr, kir, kpa);
    char gbuf[80];
    snprintf(gbuf, sizeof(gbuf), "ctl: gains x1000 = %ld %ld %ld",
             lroundf(kpr * 1000.0f), lroundf(kir * 1000.0f),
             lroundf(kpa * 1000.0f));
    telem::emitMsg(now, gbuf);
    return;
  }
  if (strcmp(cmd, "sframe") == 0) {
    // Live PWM frame-rate knob: the frame period is the dominant command->
    // pulse latency (a 50 Hz frame alone is up to 20 ms), and how far it can
    // go is a servo-model property nobody has written down - so it is a
    // bench experiment, not a compile-time guess. RAM only; the ship default
    // stays SERVO_FRAME_HZ until edited.
    char *a = strtok_r(nullptr, " ", &save);
    if (control.mode() != ctl::CM_IDLE) {
      telem::emitMsg(now, "cmd: sframe refused while armed/active ($disarm first)");
      return;
    }
    if (!servos.present()) {
      telem::emitMsg(now, "cmd: sframe refused - PCA9685 not present");
      return;
    }
    float hz = (a != nullptr) ? (float)atof(a) : 0.0f;
    if (!(hz >= 24.0f && hz <= 333.0f)) {
      telem::emitMsg(now,
          "cmd: sframe wants <hz> 24..333 (analog servos 50; digital 200-333)");
      return;
    }
    uint32_t per =
        (uint32_t)(1000000.0f / (hz * (float)SERVO_WRITE_PER_FRAME));
    if (per < 2000) per = 2000;  // scheduler-slot sanity floor (500 Hz)
    if (servos.setFrameHz(hz, per)) {
      servo_poll_us_ = per;
      char fb[112];
      snprintf(fb, sizeof(fb),
               "servo: pwm frame %ld Hz, write cadence %ld Hz - ch 4-15 "
               "released; analog servos may buzz/heat",
               lroundf(servos.frameHz()), lroundf(1000000.0f / (float)per));
      telem::emitMsg(now, fb);
    } else {
      telem::emitMsg(now, "servo: frame change FAILED (bus error)");
    }
    return;
  }
  if (strcmp(cmd, "ctlmode") == 0) {
    char *a = strtok_r(nullptr, " ", &save);
    if (a != nullptr && strcmp(a, "angle") == 0) {
      control.setAngleMode(true);
      telem::emitMsg(now, "ctl: angle-hold mode (uses estimated roll)");
    } else if (a != nullptr && strcmp(a, "rate") == 0) {
      control.setAngleMode(false);
      telem::emitMsg(now, "ctl: rate-damping mode (gyro only)");
    } else {
      telem::emitMsg(now, "cmd: ctlmode wants rate|angle");
    }
    return;
  }
  if (strcmp(cmd, "lora?") == 0 || strcmp(cmd, "lora") == 0) {
    char *a = strtok_r(nullptr, " ", &save);
    if (strcmp(cmd, "lora") == 0 && a != nullptr) {
      link::setMute(a[0] == '0');
      telem::emitMsg(now, a[0] == '0' ? "lora: muted" : "lora: transmitting");
      return;
    }
    if (!lora.present()) {
      telem::emitMsg(now, "lora: ABSENT (SX1278 not found at boot)");
      return;
    }
    // Compact so the reply survives the 48-byte LoRa msg frame when asked
    // over RF: "lora: t99999 r9999 c999 rssi -120 snr -20.0 MUTED" = 48.
    // rssi/snr are of the last UPLINK frame; n/a until one has arrived.
    char rs[8] = "n/a", ss[8] = "n/a";
    if (link::rssiValid()) {
      snprintf(rs, sizeof(rs), "%d", (int)link::lastRssiDbm());
      long s10 = lroundf(link::lastSnrDb() * 10.0f);
      snprintf(ss, sizeof(ss), "%s%ld.%ld", s10 < 0 ? "-" : "",
               labs(s10) / 10, labs(s10) % 10);
    }
    char lb[96];
    snprintf(lb, sizeof(lb), "lora: t%lu r%lu c%lu rssi %s snr %s%s",
             (unsigned long)link::txCount(), (unsigned long)link::rxCount(),
             (unsigned long)link::crcErrCount(), rs, ss,
             link::muted() ? " MUTED" : "");
    telem::emitMsg(now, lb);
    return;
  }
  if (strcmp(cmd, "sens?") == 0) {
    char sb[120];
    const SensorHealth &qi = sensors.imuHealth(), &qm = sensors.magHealth(),
                       &qb = sensors.baroHealth(), &qg = sensors.gnssHealth();
    snprintf(sb, sizeof(sb),
             "sens: imu p%df%d mag p%df%d bar p%df%d gps p%df%d pwm %d "
             "lora %d i2c e%lu r%lu",
             (int)qi.present, (int)qi.fresh, (int)qm.present, (int)qm.fresh,
             (int)qb.present, (int)qb.fresh, (int)qg.present, (int)qg.fresh,
             (int)servos.present(), (int)lora.present(),
             (unsigned long)sensors.i2cErrors(),
             (unsigned long)sensors.busResets());
    telem::emitMsg(now, sb);
    return;
  }
  if (strcmp(cmd, "zero") == 0) {
    sensors.zeroCounters();
    telem::emitMsg(now, "cmd: counters zeroed");
    return;
  }
  if (strcmp(cmd, "led") == 0) {
    char *who = strtok_r(nullptr, " ", &save);
    char *rs = strtok_r(nullptr, " ", &save);
    char *gs = strtok_r(nullptr, " ", &save);
    char *bs = strtok_r(nullptr, " ", &save);
    if (who == nullptr || rs == nullptr || gs == nullptr || bs == nullptr) {
      telem::emitMsg(now, "cmd: led wants <a|b|all> <r> <g> <b>");
      return;
    }
    const uint8_t r = parseChan(rs), g = parseChan(gs), b = parseChan(bs);
    if (strcmp(who, "a") == 0) {
      ledsSet(0, r, g, b);
    } else if (strcmp(who, "b") == 0) {
      ledsSet(1, r, g, b);
    } else if (strcmp(who, "all") == 0) {
      ledsSet(0, r, g, b);
      ledsSet(1, r, g, b);
    } else {
      telem::emitMsg(now, "cmd: led wants <a|b|all> <r> <g> <b>");
      return;
    }
    telem::emitMsg(now, "cmd: led set");
    return;
  }
  if (strcmp(cmd, "rst") == 0) {
    telem::emitMsg(now, "cmd: rebooting");
    const uint32_t t0 = millis();
    while (millis() - t0 < 150) telem::drain();  // push the reply out first
    NVIC_SystemReset();
  }
  telem::emitMsg(now, "cmd: unknown");
}

static void pollCommands() {
  int n = 0;
  while (TELEM_SERIAL.available() > 0 && n++ < 48) {
    const char c = (char)TELEM_SERIAL.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (cmd_drop_) {
        cmd_drop_ = false;  // overlong line fully discarded; back in sync
      } else {
        cmd_buf_[cmd_len_] = 0;
        if (cmd_len_ > 0 && cmd_buf_[0] == '$') execCommand(cmd_buf_);
      }
      cmd_len_ = 0;
    } else if (cmd_drop_) {
      // discarding the remainder of an overlong line
    } else if (cmd_len_ < sizeof(cmd_buf_) - 1) {
      cmd_buf_[cmd_len_++] = c;
    } else {
      // Overlong line: drop THE WHOLE LINE, not just the buffered prefix -
      // resetting mid-line let the tail (which can start with '$') execute
      // as its own command off line noise or a doubled uplink.
      cmd_drop_ = true;
      cmd_len_ = 0;
    }
  }
}

// Snapshot for the LoRa state frame: engineering units in, linkcodec
// quantizes. Same sources as buildStateRecord, sampled at the TX slot.
static void fillLinkState(lc::StateFields *f) {
  uint64_t now = monoNow();
  f->ms = (uint32_t)(now / 1000ull);
  f->fst = (uint8_t)filter.state();
  f->cmode = (uint8_t)control.mode();
  float e[3];
  filter.euler(e);
  for (int i = 0; i < 3; ++i) {
    f->eul_deg[i] = e[i] * nav::RAD2DEG;
    f->gyr_dps[i] = sensors.lastGyro()[i] * nav::RAD2DEG;
    f->acc_g[i] = sensors.lastAccel()[i] / 9.80665f;
    f->vel_mps[i] = filter.vel()[i];
  }
  f->ral_m = filter.relAlt();
  double lat = 0, lon = 0, alt = 0;
  bool geo = filter.geodetic(&lat, &lon, &alt);
  f->lat_deg = geo ? lat : 0;
  f->lon_deg = geo ? lon : 0;
  f->alt_msl_m = geo ? (float)alt : 0;
  const GnssFix &pv = sensors.lastPvt();
  f->fix = sensors.everHadPvt() && pv.fix_type >= 3;
  f->sats = pv.num_sv;
  f->hacc_m = pv.hacc_m;
  const SensorHealth &qi = sensors.imuHealth(), &qm = sensors.magHealth(),
                     &qb = sensors.baroHealth(), &qg = sensors.gnssHealth();
  f->health = (uint8_t)((qi.fresh ? 1 : 0) | (qm.fresh ? 2 : 0) |
                        (qb.fresh ? 4 : 0) | (qg.fresh ? 8 : 0));
  for (int i = 0; i < 4; ++i) f->cdef_deg[i] = control.deflDeg()[i];
}

void setup() {
  TELEM_SERIAL.begin(TELEM_BAUD);
  delay(100);  // rail/sensor power settle (setup only — never in loop)

  mag_hard_stored_ = cfgstore::loadMagHard(mag_hard_active_);
  if (!mag_hard_stored_) {
    for (int i = 0; i < 3; ++i) mag_hard_active_[i] = MAG_HARD_UT[i];
  }

  uint64_t now = monoNow();
  SensorsConfig scfg;
  buildSensorsConfig(scfg);
  sensors.begin(scfg, now);

  buildEskfConfig(eskf_cfg_);
  filter.init(eskf_cfg_);
  ledsInit(LED_A_PIN, LED_A_COUNT, LED_B_PIN, LED_B_COUNT);

  SPI.begin();
  if (lora.begin(&SPI, LORA_NSS_PIN, LORA_RESET_PIN, LORA_DIO0_PIN)) {
    lora.configure(LC_LORA_FREQ_HZ, LC_LORA_SF, LC_LORA_BW_HZ,
                   LC_LORA_CR_DENOM, LC_LORA_SYNC, LC_LORA_PREAMBLE,
                   LORA_TX_DBM);
  }
  link::begin(&lora, fillLinkState);
  link::setMute(LORA_TX_AT_BOOT == 0);

  ServoConfig svc;
  svc.addr = SERVO_PCA_ADDR;
  svc.frame_hz = SERVO_FRAME_HZ;
  for (int i = 0; i < 4; ++i) {
    svc.ch[i] = SERVO_CH[i];
    svc.center_us[i] = SERVO_CENTER_US[i];
    svc.us_per_deg[i] = SERVO_US_PER_DEG[i];
  }
  svc.min_us = SERVO_MIN_US;
  svc.max_us = SERVO_MAX_US;
  svc.write_period_us = servo_poll_us_;
  servos.begin(&Wire, svc);
  pca_retry_next_us_ = monoNow() + 1000000ull;  // recovery probe cadence
  {
    LinkageCal lc[4];
    if (cfgstore::loadLinkage(lc)) {
      servos.setLinkage(lc);
      for (int i = 0; i < 4; ++i) {
        if (lc[i].n >= 2) lcal_fins_ |= (uint8_t)(1u << i);
      }
    }
  }
  // Drive every fin to its calibrated 0 deg (neutral) at boot - AFTER the
  // linkage tables are loaded, so "straight" means each fin's designated
  // zero, not the raw 1500 us servo center. The first servo service tick
  // pushes these targets to the PCA.
  servos.center();

  ctl::Config cc;
  cc.kp_rate = CTL_KP_RATE;
  cc.ki_rate = CTL_KI_RATE;
  cc.kp_ang = CTL_KP_ANG;
  cc.rate_cmd_max_dps = CTL_RATE_CMD_MAX;
  cc.defl_max_deg = CTL_DEFL_MAX;  // linkage clamp applied right after init
  cc.slew_dps = CTL_SLEW_DPS;
  for (int i = 0; i < 4; ++i) cc.mix_sign[i] = CTL_MIX_SIGN[i];
  cc.angle_mode = false;  // first flights: gyro-only rate damping
  cc.launch_acc_g = CTL_LAUNCH_ACC_G;
  cc.launch_hold_s = CTL_LAUNCH_HOLD_S;
  cc.safe_tilt_deg = CTL_SAFE_TILT_DEG;
  cc.safe_time_s = CTL_SAFE_TIME_S;
  cc.safe_vd_mps = CTL_SAFE_VD_MPS;
  control.init(cc);
  applyAuthority();  // linkage-limited authority, same rule as $lcal ops

  telem::init(&TELEM_SERIAL);
  TelemetryHeaderInfo h;
  h.us = monoNow();
  h.fw = FW_VERSION;
  h.schema = SCHEMA_VERSION;
  h.imu = sensors.imuHealth().present;
  h.mag = sensors.magHealth().present;
  h.bar = sensors.baroHealth().present;
  h.gps = sensors.gnssHealth().present;
  h.iodr = IMU_ODR_HZ;
  h.afs = IMU_ACCEL_FS_G;
  h.gfs = IMU_GYRO_FS_DPS;
  h.modr = MAG_ODR_HZ;
  h.mmode = kMagModeStr;
  h.gmode = (GPS_MODE_SEL == 0) ? "off"
                                : ((GPS_MODE_SEL == 1) ? "auto" : "req");
  h.bhz = BARO_POLL_HZ;
  h.ghz = (int)GNSS_TARGET_HZ;
  h.ohz = TELEM_HZ;
  telem::emitHeader(h);

  // Startup sequence, part 1: per-sensor probe results as "msg" records —
  // the same NDJSON stream, so they land in any log and in the viewer.
  uint64_t bus = monoNow();
  telem::emitMsg(bus, h.imu ? "imu ICM-45686 @0x68: OK"
                            : "imu ICM-45686 @0x68: NOT FOUND");
  telem::emitMsg(bus, h.mag ? "mag LIS3MDL @0x1C: OK"
                            : "mag LIS3MDL @0x1C: NOT FOUND");
  telem::emitMsg(bus, h.bar ? "baro BME280 @0x77: OK"
                            : "baro BME280 @0x77: NOT FOUND");
  telem::emitMsg(bus, (GPS_MODE_SEL == 0)
                          ? "gnss: disabled (GPS_OFF)"
                          : (h.gps ? "gnss SAM-M8Q @0x42: OK"
                                   : "gnss SAM-M8Q @0x42: not responding (retrying)"));
  if (mag_hard_stored_) {
    char cbuf[80];
    snprintf(cbuf, sizeof(cbuf), "cfg: stored mag hard-iron uT x100 = %ld %ld %ld",
             lroundf(mag_hard_active_[0] * 100.0f),
             lroundf(mag_hard_active_[1] * 100.0f),
             lroundf(mag_hard_active_[2] * 100.0f));
    telem::emitMsg(bus, cbuf);
  } else {
    telem::emitMsg(bus, "cfg: no stored mag cal (compile-time offsets)");
  }
  telem::emitMsg(bus, !lora.present()
                          ? "lora SX1278 (SPI1): NOT FOUND"
                          : (link::muted()
                                 ? "lora SX1278 (SPI1): OK - tx muted "
                                   "($lora 1 to transmit; antenna first)"
                                 : "lora SX1278 (SPI1): OK - transmitting"));
  telem::emitMsg(bus, servos.present() ? "pwm PCA9685 @0x40: OK"
                                       : "pwm PCA9685 @0x40: NOT FOUND");
  {
    int ncal = 0;
    for (int i = 0; i < 4; ++i) {
      if (lcal_fins_ & (1u << i)) ncal++;
    }
    float auth = servos.authorityDeg();
    if (auth > CTL_DEFL_MAX) auth = CTL_DEFL_MAX;
    char lbuf2[96];
    snprintf(lbuf2, sizeof(lbuf2),
             "cfg: linkage cal %d/4 fins%s - authority x10 = %ld deg", ncal,
             ncal ? "" : " (defaults - run $lcal)", lroundf(auth * 10.0f));
    telem::emitMsg(bus, lbuf2);
    telem::emitMsg(bus, servos.present()
                            ? "servo: fins driven to calibrated 0 deg neutral"
                            : "servo: PCA9685 absent - fins not driven");
  }
  lcalDump(bus);  // seed the viewer's linkage list/graph at connect
  // From here on every msg record also rides the LoRa downlink.
  telem::setMsgTap(link::queueMsg);

  now = monoNow();
  next_imu_ = now + IMU_POLL_US;
  next_mag_ = now + 3000;
  next_baro_ = now + 5000;
  next_gnss_ = now + 7000;
  next_radio_ = now + 9000;
  next_servo_ = now + 11000;
  next_telem_ = now;  // first record before any loop sensor I/O: a liveness
                      // breadcrumb that localizes a wedge to a specific poll
  next_service_ = now + SERVICE_PERIOD_US;
  prop_last_us_ = now;
  loop_prev_us_ = now;
}

static void emitOriginRecord(uint64_t now) {
  const nav::GeoOrigin &o = filter.origin();
  double lat, lon, alt;
  float zero[3] = { 0, 0, 0 };
  nav::ned_to_geo(o, zero, &lat, &lon, &alt);
  TelemetryOriginInfo rec;
  rec.us = now;
  rec.lat = lat;
  rec.lon = lon;
  rec.alt = (float)alt;
  rec.hae = (float)o.hae_mm * 1e-3f;
  rec.g0 = o.gravity;
  rec.p0 = sensors.baroRefPa();
  rec.t0 = sensors.baroRefC();
  telem::emitOrigin(rec);
}

static void processFixes(uint64_t now) {
  GnssFix f;
  while (sensors.haveNewFix(f)) {
    if (!f.valid_for_nav) continue;  // counted as a GNSS drop in sensors

    if (filter.state() == eskf::FS_WAIT_FIX ||
        filter.state() == eskf::FS_ATT_ONLY) {  // late-fix upgrade path
      // Stricter anchor gate: the origin is forever.
      if (f.hacc_m <= ANCHOR_MAX_HACC_M) {
        filter.anchorOrigin(f.lat1e7, f.lon1e7, f.hae_mm, f.hmsl_mm,
                            f.vel_ned, f.hacc_m, f.vacc_m, f.sacc_ms);
        last_gps_accept_us_ = now;
        if (!origin_emitted_ && filter.state() == eskf::FS_RUN) {
          emitOriginRecord(now);
          origin_emitted_ = true;
        }
      }
      continue;
    }
    if (filter.state() != eskf::FS_RUN) continue;

    // Reacquisition gating: after an outage (dead-reckoning stretch with
    // covariance growth), the first fixes carry inflated R and must pass
    // their chi-square gates before normal weighting resumes.
    float gap_s = (float)(now - last_gps_accept_us_) * 1e-6f;
    if (last_gps_accept_us_ != 0 && gap_s > GNSS_REACQ_GAP_S &&
        reacq_countdown_ == 0) {
      reacq_countdown_ = GNSS_REACQ_N;
    }
    bool inflate = reacq_countdown_ > 0;
    bool pos_ok = filter.updateGpsPos(f.lat1e7, f.lon1e7, f.hae_mm, f.hacc_m,
                                      f.vacc_m, inflate);
    bool vel_ok = filter.updateGpsVel(f.vel_ned, f.sacc_ms, inflate);
    if (pos_ok || vel_ok) {
      last_gps_accept_us_ = now;
      if (reacq_countdown_ > 0) reacq_countdown_--;
    }
  }
}

static void buildStateRecord(uint64_t now) {
  TelemetryStateInfo r;
  r.us = now;
  r.seq = seq_++;
  r.fst = (uint8_t)filter.state();
  r.alq = !filter.alignDegraded();

  double lat = 0, lon = 0, alt = 0;
  r.geo_valid = filter.geodetic(&lat, &lon, &alt);
  r.lat = lat;
  r.lon = lon;
  r.alt = alt;
  r.nav_valid = (filter.state() == eskf::FS_RUN);
  const float *pp = filter.pos();
  const float *vv = filter.vel();
  const float *qq = filter.quat();
  for (int i = 0; i < 3; ++i) { r.p[i] = pp[i]; r.v[i] = vv[i]; }
  for (int i = 0; i < 4; ++i) r.q[i] = qq[i];
  float eul[3];
  filter.euler(eul);
  for (int i = 0; i < 3; ++i) r.eul_deg[i] = eul[i] * nav::RAD2DEG;
  float sp[3], sv[3], sa[3];
  filter.sigmas(sp, sv, sa);
  for (int i = 0; i < 3; ++i) {
    r.sp[i] = sp[i];
    r.sv[i] = sv[i];
    r.sa_deg[i] = sa[i] * nav::RAD2DEG;
  }
  const float *bg = filter.gyroBias();
  const float *ba = filter.accelBias();
  for (int i = 0; i < 3; ++i) { r.bg[i] = bg[i]; r.ba[i] = ba[i]; }
  r.bb_valid = r.nav_valid;
  r.bb = filter.baroBias();
  r.vch_valid = (r.fst == (uint8_t)eskf::FS_WAIT_FIX ||
                 r.fst == (uint8_t)eskf::FS_RUN ||
                 r.fst == (uint8_t)eskf::FS_ATT_ONLY);
  r.ral = filter.relAlt();
  r.rvs = -vv[2];
  r.sra = filter.relAltStd();
  r.srv = sv[2];

  const SensorHealth &hi = sensors.imuHealth();
  const SensorHealth &hm = sensors.magHealth();
  const SensorHealth &hb = sensors.baroHealth();
  const SensorHealth &hg = sensors.gnssHealth();
  r.imu_valid = hi.present && !hi.stale;
  for (int i = 0; i < 3; ++i) {
    r.acc[i] = sensors.lastAccel()[i];
    r.gyr[i] = sensors.lastGyro()[i];
    r.mag[i] = sensors.lastMag()[i];
  }
  r.mag_valid = hm.present && !hm.stale;
  r.mgr_valid = sensors.magRawSeen();
  for (int i = 0; i < 3; ++i) r.mgr[i] = sensors.lastMagRaw()[i];
  r.baro_valid = hb.present && !hb.stale;
  r.pa = sensors.lastPressPa();
  r.tc = sensors.lastTempC();

  r.gps_seen = sensors.everHadPvt();
  const GnssFix &pv = sensors.lastPvt();
  r.gfix = pv.fix_type;
  r.gsv = pv.num_sv;
  r.ghac = pv.hacc_m;
  r.gvac = pv.vacc_m;
  r.gsac = pv.sacc_ms;
  r.glat = (double)pv.lat1e7 * 1e-7;
  r.glon = (double)pv.lon1e7 * 1e-7;
  r.galt = (float)pv.hmsl_mm * 1e-3f;
  r.gage = r.gps_seen ? (float)(now - pv.t_us) * 1e-6f : 0.0f;

  const eskf::InnovRecord &igp = filter.innov(eskf::CH_GPS_POS);
  const eskf::InnovRecord &igv = filter.innov(eskf::CH_GPS_VEL);
  const eskf::InnovRecord &ibr = filter.innov(eskf::CH_BARO);
  const eskf::InnovRecord &img = filter.innov(eskf::CH_MAG);
  r.igp_v = igp.valid;
  r.igv_v = igv.valid;
  r.img_v = img.valid;
  r.ibr_v = ibr.valid;
  for (int i = 0; i < 3; ++i) { r.igp[i] = igp.nu[i]; r.igv[i] = igv.nu[i]; }
  r.img_deg = img.nu[0];   // vector update: nu[0] is the N component, uT
  r.ibr = ibr.nu[0];
  r.ngp_v = igp.valid && igp.nis_valid;
  r.ngv_v = igv.valid && igv.nis_valid;
  r.nmg_v = img.valid && img.nis_valid;
  r.nbr_v = ibr.valid && ibr.nis_valid;
  r.ngp = igp.nis;
  r.ngv = igv.nis;
  r.nmg = img.nis;
  r.nbr = ibr.nis;
  r.kgp_v = igp.valid;
  r.kgv_v = igv.valid;
  r.kmg_v = img.valid;
  r.kbr_v = ibr.valid;
  r.kgp = igp.accepted;
  r.kgv = igv.accepted;
  r.kmg = img.accepted;
  r.kbr = ibr.accepted;

  r.himu = hi.bits();
  r.hmag = hm.bits();
  r.hbar = hb.bits();
  r.hgps = hg.bits();
  r.dimu = hi.drops;
  r.dmag = hm.drops;
  r.dbar = hb.drops;
  r.dgps = hg.drops;
  r.dtx = telem::droppedRecords();
  r.ei2c = sensors.i2cErrors();
  r.ri2c = sensors.busResets();
  r.grz_v = sensors.everHadPvt();
  r.grz = sensors.gnssRateHz();

  // Achieved filter rate (EWMA over the record period).
  uint32_t props = filter.propCount();
  float dt = (float)(now - prop_last_us_) * 1e-6f;
  if (dt > 0.005f) {
    float hz = (float)(props - prop_last_count_) / dt;
    fhz_ += 0.3f * (hz - fhz_);
    prop_last_count_ = props;
    prop_last_us_ = now;
  }
  r.fhz = fhz_;
  r.lmx = lmx_us_;
  lmx_us_ = 0;

  r.lre = lora.present();
  r.rssi_v = link::rssiValid();
  r.rssi_dbm = (float)link::lastRssiDbm();
  r.snr_db = link::lastSnrDb();
  r.ltx = link::txCount();
  r.lrx = link::rxCount();
  r.lcrc = link::crcErrCount();
  r.cmode = (uint8_t)control.mode();
  for (int i = 0; i < 4; ++i) r.cdef[i] = control.deflDeg()[i];

  telem::emitState(r);
}

static void orchestrate(uint64_t now) {
  // Startup sequence, part 2: watch filter-state transitions and post
  // progress plus the calibration result ("msg"/"cal" records, SCHEMA.md).
  uint8_t fs = (uint8_t)filter.state();
  if (fs != fst_seen_) {
    if (fs == eskf::FS_ALIGN) {
      align_t0_us_ = now;
      telem::emitMsg(now, "cal: averaging gyro + gravity (hold still)");
    }
    if (fst_seen_ == eskf::FS_ALIGN && fs >= eskf::FS_WAIT_FIX &&
        !cal_emitted_) {
      cal_emitted_ = true;
      TelemetryCalInfo c;
      c.us = now;
      const float *bg = filter.gyroBias();
      float eul[3];
      filter.euler(eul);
      for (int i = 0; i < 3; ++i) {
        c.bg_dps[i] = bg[i] * nav::RAD2DEG;
        c.eul_deg[i] = eul[i] * nav::RAD2DEG;
      }
      c.alq = !filter.alignDegraded();
      c.dur_s = (float)(now - align_t0_us_) * 1e-6f;
      telem::emitCal(c);
      {
        // The heading and its source in one line: makes a boot that fell
        // back to the no-mag prior (arbitrary heading) obvious on sight.
        float hdg = c.eul_deg[2] < 0 ? c.eul_deg[2] + 360.0f : c.eul_deg[2];
        char abuf[112];
        snprintf(abuf, sizeof(abuf), "cal: %s - heading %ld deg (%s)",
                 c.alq ? "done (clean static)"
                       : "DEGRADED (vehicle moved; covariance widened)",
                 lroundf(hdg),
                 filter.magInitUsed() ? "mag" : "no mag - arbitrary");
        telem::emitMsg(now, abuf);
      }
      if (kMagMode != eskf::MAG_OFF && sensors.magHealth().present &&
          !filter.magInitUsed()) {
        telem::emitMsg(now,
            "mag: field rejected during align - heading coarse until a "
            "clean field (run $magcal / move from steel)");
      }
      telem::emitMsg(now, (GPS_MODE_SEL == 0) ? "nav: attitude + baro altitude (GPS_OFF)"
                                              : "nav: vertical channel running - waiting for GNSS fix");
    }
    if (fs == eskf::FS_RUN)
      telem::emitMsg(now, "nav: origin anchored - full navigation");
    fst_seen_ = fs;
  }
  // Barometric reference: collect while aligning, freeze once alignment is
  // done — the hypsometric alt_rel is then relative to the pad.
  if (filter.state() == eskf::FS_ALIGN) {
    sensors.baroRefCollect(true);
  } else if (filter.state() >= eskf::FS_WAIT_FIX) {
    sensors.baroRefCollect(false);  // idempotent finalize
  }
  // GPS-usage determination (startup decision, latched for the power cycle).
  if (filter.state() == eskf::FS_WAIT_FIX && !gps_decided_off_) {
    bool go_att_only = (GPS_MODE_SEL == 0);
    if (GPS_MODE_SEL == 1 &&
        now > (uint64_t)(GPS_AUTO_DECIDE_S * 1e6f)) {
      go_att_only = true;  // no usable fix by the deadline: orientation-only
    }
    if (go_att_only) {
      gps_decided_off_ = true;
      filter.setAttitudeOnly();
      if (GPS_MODE_SEL == 1)
        telem::emitMsg(now,
            "gps: no fix by deadline - attitude + baro altitude (a later fix upgrades)");
    }
  }
}

void loop() {
  uint64_t now = monoNow();
  if (loop_prev_us_ != 0) {
    uint32_t d = (uint32_t)(now - loop_prev_us_);
    if (d > lmx_us_) lmx_us_ = d;
  }
  loop_prev_us_ = now;
  pollCommands();
  {
    char lbuf[64];
    if (link::popCommand(lbuf, sizeof(lbuf)) && lbuf[0] == '$') {
      execCommand(lbuf);
    }
  }

  if (now >= next_imu_) {
    next_imu_ += IMU_POLL_US;
    if (now > next_imu_ + 10ull * IMU_POLL_US) next_imu_ = now + IMU_POLL_US;
    eskf::ImuSample s;
    if (sensors.pollImu(now, s)) {
      filter.feedImu(s);
      // Control rides the IMU cadence. ALL phase logic lives in ctl; the
      // filter stays phase-free and nothing here feeds back into it.
      float e[3];
      filter.euler(e);
      float R[9];
      nav::quat_to_dcm(filter.quat(), R);
      float cx = -R[6];  // body-x up component -> nose tilt from vertical
      if (cx > 1.0f) cx = 1.0f;
      if (cx < -1.0f) cx = -1.0f;
      // dt is the sample interval (fresh samples arrive at the sensor ODR),
      // not the poll interval - 1/440 here made every integrator and slew
      // step run 10% slow.
      control.tick(1.0f / (float)IMU_ODR_HZ,
                   (s.gyro[0] - filter.gyroBias()[0]) * nav::RAD2DEG,
                   e[0] * nav::RAD2DEG, acosf(cx) * nav::RAD2DEG,
                   s.accel[0] / 9.80665f, filter.vel()[2],
                   sensors.imuHealth().fresh);
      if (control.mode() == ctl::CM_ACTIVE || control.mode() == ctl::CM_BENCH ||
          control.mode() == ctl::CM_SAFE) {
        servos.setDeflDeg(control.deflDeg());  // IDLE leaves bench commands be
      }
    }
  } else {
    int32_t slack = (int32_t)(int64_t)(next_imu_ - now) - SLACK_MARGIN_US;
    // Priority order matters when the bus is sick: a wedged/half-booted
    // slave turns every Wire transaction into a long HAL timeout, which
    // keeps the I2C tasks perpetually "due". The record builder (pure CPU)
    // and the watchdog service (which owns reinit and bus-clear) must
    // outrank them, or the stream goes silent exactly when its health
    // fields are the only diagnostic — telemetry first, recovery second,
    // sensors after.
    if (now >= next_telem_ && slack > COST_TELEM_US) {
      next_telem_ += TELEM_PERIOD_US;
      if (now > next_telem_ + 10ull * TELEM_PERIOD_US) {
        next_telem_ = now + TELEM_PERIOD_US;
      }
      buildStateRecord(now);
    } else if (now >= next_service_ && slack > COST_SERVICE_US) {
      next_service_ = now + SERVICE_PERIOD_US;
      sensors.service(now);
      orchestrate(now);
      ledsService();
      // PCA9685 recovery: a failed boot probe (slow power-up, bus glitch)
      // or a $sframe sequence that died mid-flight marks the chip absent;
      // re-probe with backoff so servos and arming come back without a
      // power cycle. recover() force-rewrites the canard targets.
      if (!servos.present() && now >= pca_retry_next_us_) {
        if (servos.recover()) {
          pca_retry_backoff_ms_ = 1000;
          telem::emitMsg(now, "pwm PCA9685 @0x40: recovered");
        } else {
          pca_retry_backoff_ms_ *= 2;
          if (pca_retry_backoff_ms_ > 30000) pca_retry_backoff_ms_ = 30000;
        }
        pca_retry_next_us_ = now + (uint64_t)pca_retry_backoff_ms_ * 1000ull;
      }
      sensors.magCalService(now);  // time-based completion (mag can be dead)
      {
        uint32_t mcn = 0, mcel = 0;
        float mcsp[3], mclast = 0;
        if (sensors.magCalProgress(now, &mcn, mcsp, &mclast, &mcel)) {
          if (mcel >= magcal_prog_next_s_) {
            magcal_prog_next_s_ = mcel + 10;
            // Heartbeat: proves collection is alive and shows coverage
            // growing. n stuck at 0 or |m| out of [5,200] uT points
            // straight at the collection path.
            char pbuf[112];
            snprintf(pbuf, sizeof(pbuf),
                     "magcal: %lus - %lu samples, spread uT = %ld %ld %ld, "
                     "|m| %ld",
                     (unsigned long)mcel, (unsigned long)mcn,
                     lroundf(mcsp[0]), lroundf(mcsp[1]), lroundf(mcsp[2]),
                     lroundf(mclast));
            telem::emitMsg(now, pbuf);
          }
        } else {
          magcal_prog_next_s_ = 10;
        }
      }
      float hi[3], msp[3];
      uint8_t mheld = 0;
      if (sensors.magCalTakeResult(hi, msp, &mheld)) {
        // Result FIRST, then the flash save: the sector erase freezes the
        // whole chip ~1-2 s (code runs from the same flash bank), and a
        // save that goes wrong must never eat the result. flush() puts the
        // messages on the wire before the freeze starts.
        for (int i = 0; i < 3; ++i) mag_hard_active_[i] = hi[i];
        // integer centi-uT keeps the text printf-float-free and unambiguous
        char mbuf[112];
        snprintf(mbuf, sizeof(mbuf),
                 "magcal: hard-iron uT x100 = %ld %ld %ld - applied",
                 lroundf(hi[0] * 100.0f), lroundf(hi[1] * 100.0f),
                 lroundf(hi[2] * 100.0f));
        telem::emitMsg(now, mbuf);
        char sbuf[112];
        if (mheld) {
          snprintf(sbuf, sizeof(sbuf),
                   "magcal: thin coverage - held%s%s%s (spread uT = %ld %ld "
                   "%ld); tumble ALL faces and redo $magcal",
                   (mheld & 1) ? " x" : "", (mheld & 2) ? " y" : "",
                   (mheld & 4) ? " z" : "", lroundf(msp[0]), lroundf(msp[1]),
                   lroundf(msp[2]));
        } else {
          snprintf(sbuf, sizeof(sbuf),
                   "magcal: coverage ok (spread uT = %ld %ld %ld)",
                   lroundf(msp[0]), lroundf(msp[1]), lroundf(msp[2]));
        }
        telem::emitMsg(now, sbuf);
        telem::emitMsg(now, "magcal: saving to flash - board freezes ~2 s");
        TELEM_SERIAL.flush();
        bool saved = cfgstore::saveMagHard(hi);
        telem::emitMsg(now, saved
                                ? "magcal: saved to flash; send $cal to re-align"
                                : "magcal: flash save FAILED (RAM only until reboot)");
      }
    } else if (now >= next_radio_ && slack > COST_RADIO_US) {
      next_radio_ = now + RADIO_POLL_US;
      link::service(now);
    } else if (now >= next_gnss_ && slack > COST_GNSS_US) {
      next_gnss_ = now + GNSS_POLL_US;
      sensors.pollGnss(now);
      processFixes(now);
    } else if (now >= next_mag_ && slack > COST_MAG_US) {
      next_mag_ = now + MAG_POLL_US;
      float m[3];
      if (sensors.pollMag(now, m)) filter.feedMag(m);
    } else if (now >= next_baro_ && slack > COST_BARO_US) {
      next_baro_ = now + BARO_POLL_US;
      float pa, tc, alt_rel;
      if (sensors.pollBaro(now, &pa, &tc, &alt_rel)) {
        // alt_rel is NAN until the alignment reference freezes; after that
        // the filter takes it in every state past alignment (the vertical
        // channel runs pre-anchor and in attitude-only operation).
        if (isfinite(alt_rel)) filter.updateBaro(alt_rel, tc);
      }
    } else if (now >= next_servo_ && slack > COST_SERVO_US) {
      next_servo_ = now + servo_poll_us_;
      servos.service(now);
      int sc = servos.scanChannel();
      if (sc != ang_scan_last_) {
        ang_scan_last_ = sc;
        char scb[64];
        if (sc >= 0) {
          snprintf(scb, sizeof(scb), "ang: scan - wiggling ch %d", sc);
          telem::emitMsg(now, scb);
        } else {
          telem::emitMsg(now, "ang: scan done - canards recentered");
        }
      }
      int cp = servos.canardTestPhase();
      if (cp != ctest_last_phase_) {
        ctest_last_phase_ = cp;
        const char *m = (cp == 0) ? "cang test: all fins +10 deg"
                        : (cp == 1) ? "cang test: all fins -10 deg"
                        : (cp == 2) ? "cang test: all fins 0 deg"
                                    : "cang test: done - fins at neutral";
        telem::emitMsg(now, m);
      }
    }
  }
  telem::drain();  // nonblocking, every pass
}
