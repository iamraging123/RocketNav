// telemetry.h — newline-delimited JSON output with a byte ring buffer that
// never blocks: when the link backs up, whole records are dropped and
// counted rather than stalling the filter. Field names are the schema; every
// key emitted here is documented in SCHEMA.md (and nothing else is).
//
// Floats are formatted by a local fixed-point writer because newlib-nano's
// printf has no %f on this core. NaN/inf always serialize as JSON null.
//
// Compile-time sizing lives here (array dimensions can't come from the
// runtime config struct): TELEM_RING_BYTES / TELEM_LINE_BYTES below.

#pragma once
#include <Arduino.h>
#include <stdint.h>

// ~1.1 kB/record at 50 Hz = 55 kB/s against 92 kB/s of 921600 baud wire
// rate; the ring absorbs ~7 records (140 ms) of backpressure before dropping.
#define TELEM_RING_BYTES 8192
#define TELEM_LINE_BYTES 1792

struct TelemetryHeaderInfo {
  uint64_t us;
  const char *fw;
  int schema;
  bool imu, mag, bar, gps;       // sensor presence at boot
  int iodr;                      // IMU ODR, Hz
  int afs;                       // accel full scale, g
  int gfs;                       // gyro full scale, dps
  int modr;                      // mag ODR, Hz
  const char *mmode;             // "cont" | "init" | "off"
  const char *gmode;             // GPS usage: "auto" | "off" | "req"
  int bhz, ghz, ohz;             // baro poll, GNSS nav, output rates (Hz)
};

struct TelemetryCalInfo {
  uint64_t us;
  float bg_dps[3];               // averaged gyro offset (body), deg/s
  float eul_deg[3];              // initial attitude [roll, pitch, yaw], deg
  bool alq;                      // 1 = clean static window
  float dur_s;                   // averaging duration, s
};

struct TelemetryOriginInfo {
  uint64_t us;
  double lat, lon;               // deg
  float alt;                     // m MSL
  float hae;                     // m ellipsoid
  float g0;                      // m/s^2 site gravity
  float p0;                      // Pa baro reference
  float t0;                      // degC baro reference
};

struct TelemetryStateInfo {
  uint64_t us;
  uint32_t seq;
  uint8_t fst;                   // filter state 0..3
  bool alq;                      // alignment quality (1 = clean static)

  bool geo_valid;
  double lat, lon;
  double alt;

  bool nav_valid;                // p/v/sp/sv meaningful (FS_RUN)
  float p[3], v[3];
  float q[4], eul_deg[3];
  float sp[3], sv[3], sa_deg[3];
  float bg[3], ba[3];
  bool bb_valid;
  float bb;
  bool vch_valid;                // vertical channel running (fst 2/3/4)
  float ral;                     // m, altitude above the pad baro reference
  float rvs;                     // m/s, vertical rate, up positive
  float sra, srv;                // 1-sigma of ral / rvs

  bool imu_valid;
  float acc[3], gyr[3];
  bool mag_valid;
  float mag[3];
  bool mgr_valid;                // any raw mag register read ever succeeded
  float mgr[3];                  // raw field: scaled + mount-rotated only
  bool baro_valid;
  float pa, tc;

  bool gps_seen;                 // any PVT ever received
  uint8_t gfix, gsv;
  float ghac, gvac, gsac;
  double glat, glon;
  float galt;
  float gage;                    // s since last PVT

  bool igp_v; float igp[3];
  bool igv_v; float igv[3];
  bool img_v; float img_deg;     // mag vector innovation nu[0], uT
  bool ibr_v; float ibr;
  bool ngp_v; float ngp;
  bool ngv_v; float ngv;
  bool nmg_v; float nmg;
  bool nbr_v; float nbr;
  bool kgp_v; bool kgp;
  bool kgv_v; bool kgv;
  bool kmg_v; bool kmg;
  bool kbr_v; bool kbr;

  uint8_t himu, hmag, hbar, hgps;
  uint32_t dimu, dmag, dbar, dgps, dtx;
  uint32_t ei2c, ri2c;
  bool grz_v; float grz;
  float fhz;
  uint32_t lmx;

  // LoRa link + control
  bool lre;                      // radio present
  bool rssi_v;                   // an uplink frame has been heard
  float rssi_dbm, snr_db;        // of the last uplink frame
  uint32_t ltx, lrx, lcrc;       // frames sent / commands received / CRC rejects
  uint8_t cmode;                 // control mode 0 idle 1 armed 2 active 3 safe
  float cdef[4];                 // canard deflections, deg
};

namespace telem {

void init(HardwareSerial *port);         // port must already be begun
void emitHeader(const TelemetryHeaderInfo &h);
void emitMsg(uint64_t us, const char *txt);  // txt: fixed ASCII literal, no
                                             // quotes/backslashes (no escaper)
void emitCal(const TelemetryCalInfo &c);
void emitOrigin(const TelemetryOriginInfo &o);
void emitState(const TelemetryStateInfo &s);

// Optional tap on every emitMsg text (the LoRa downlink forwards them).
void setMsgTap(void (*tap)(const char *txt));

// Linkage-cal table snapshot ("lcal" record). src: 0 = synthesized default,
// 1 = calibrated table APPLIED to the servos (the flash write's success is
// messaged at save time and $lcal flash shows what persisted - a RAM-only
// table after a failed save still reports 1), 2 = staged points of the
// active $lcal session (fin -1 with src 2 = no session). Arrays are n long.
void emitLcal(uint64_t us, int8_t fin, uint8_t src, uint8_t n,
              const float deg[5], const float pus[5]);
void drain();                            // nonblocking push toward the UART
uint32_t droppedRecords();

}  // namespace telem
