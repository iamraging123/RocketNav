// linkcodec - the LoRa air protocol, pure codec half. No hardware, no
// Arduino: the rocket, the base station, and the host tests all compile this
// same file, which is what keeps the two ends of the link in agreement.
//
// Frame: [0x4B][type][seq][payload...][crc16 hi][crc16 lo], max 64 bytes.
// Types: 'T' packed state snapshot (rocket -> ground, flight profile, 4 Hz)
//        'B' packed position beacon (rocket -> ground, recovery profile)
//        'M' text message, the firmware's msg records (rocket -> ground)
//        'C' command line "$..." (ground -> rocket)
//        'K' keepalive, payload-less (ground -> rocket, every 5 s)
// CRC-16/CCITT-FALSE over everything before the CRC itself.
#pragma once

#include <stdint.h>

namespace lc {

const uint8_t kMagic = 0x4B;
const uint8_t kTypeState = 'T';
const uint8_t kTypeBeacon = 'B';
const uint8_t kTypeMsg = 'M';
const uint8_t kTypeCmd = 'C';
const uint8_t kTypeKeepalive = 'K';
const int kMaxFrame = 64;
const int kMaxText = 48;
const int kStateFrameLen = 54;      // 3 header + 49 payload + 2 crc
const int kBeaconFrameLen = 27;     // 3 header + 22 payload + 2 crc
const int kKeepaliveFrameLen = 5;   // 3 header + 2 crc

// Air profiles BOTH ends must share (PLAN_LORA_RANGE.md). FLIGHT is the
// display link: short frames at 4 Hz slip between the spin-induced fades.
// RECOVERY is the find-the-rocket link after landing: +17 dB of sensitivity
// for a 27-byte beacon of ~0.9 s airtime every 10 s. Symbol time 2^sf/bw;
// the driver turns LowDataRateOptimize on whenever it exceeds 16 ms.
struct Profile {
  uint8_t sf;
  uint32_t bw_hz;
  uint8_t cr_denom;     // 5 = 4/5 ... 8 = 4/8
  uint16_t preamble;    // symbols
  uint32_t period_us;   // downlink cadence
};
const Profile kProfileFlight = { 7, 500000ul, 5, 8, 250000ul };
const Profile kProfileRecovery = { 11, 125000ul, 5, 12, 10000000ul };
const uint8_t kProfFlight = 0;
const uint8_t kProfRecovery = 1;

// Symbol time in microseconds: the LowDataRateOptimize test (> 16 ms) and
// airtime estimates share it.
inline uint32_t symbolTimeUs(uint8_t sf, uint32_t bw_hz) {
  return (uint32_t)((1000000ull << sf) / bw_hz);
}

// The flight profile as macros: the boot-time configure() on both ends and
// the older call sites use these; they MUST equal kProfileFlight.
#define LC_LORA_FREQ_HZ 433500000ul
#define LC_LORA_SF 7
#define LC_LORA_BW_HZ 500000ul
#define LC_LORA_CR_DENOM 5  // 4/5
#define LC_LORA_SYNC 0x4B
#define LC_LORA_PREAMBLE 8
#define LC_TX_PERIOD_US 250000ul  // rocket state-frame cadence (4 Hz)

// Everything the 49-byte state payload carries, in engineering units.
// Quantization (applied by packState, undone by parseState):
//   eul 0.01 deg | gyr 0.1 dps | acc 0.01 g | vel 0.1 m/s | ral 0.5 m
//   lat/lon 1e-7 deg | alt_msl 1 m | hacc 0.1 m (cap 25.5) | cdef 0.25 deg
//   up_rssi 1 dBm as -value in a byte (0 = no uplink heard yet)
struct StateFields {
  uint32_t ms;         // ms since boot
  uint8_t fst;         // filter state 0..4
  bool fix;            // GNSS fix valid
  uint8_t cmode;       // control mode 0..3
  float eul_deg[3];
  float gyr_dps[3];
  float acc_g[3];
  float vel_mps[3];
  float ral_m;         // altitude above pad baro reference
  double lat_deg, lon_deg;
  float alt_msl_m;
  uint8_t sats;
  float hacc_m;
  uint8_t health;      // bit0 imu, 1 mag, 2 baro, 3 gnss (fresh)
  float cdef_deg[4];   // canard deflections
  int16_t up_rssi_dbm; // RSSI of the last uplink frame at the rocket; 0 = none
};

// The 22-byte recovery beacon: position first, attitude as a courtesy.
// Quantization: lat/lon 1e-7 deg | alt_msl 1 m | ral 0.5 m | eul 2 deg.
struct BeaconFields {
  uint32_t ms;
  uint8_t fst;
  bool fix;
  uint8_t cmode;
  double lat_deg, lon_deg;
  float alt_msl_m;
  float ral_m;
  float eul_deg[3];
  uint8_t sats;
  uint8_t health;
};

uint16_t crc16(const uint8_t *p, int n);

// Returns the frame length written into out (kStateFrameLen).
int packState(const StateFields &f, uint8_t seq, uint8_t out[kMaxFrame]);
bool parseState(const uint8_t *buf, int len, StateFields *f, uint8_t *seq);

int packBeacon(const BeaconFields &b, uint8_t seq, uint8_t out[kMaxFrame]);
bool parseBeacon(const uint8_t *buf, int len, BeaconFields *b, uint8_t *seq);

int packKeepalive(uint8_t seq, uint8_t out[kMaxFrame]);
bool parseKeepalive(const uint8_t *buf, int len, uint8_t *seq);

// type is kTypeMsg or kTypeCmd; text is truncated to kMaxText. Returns the
// frame length. parseText accepts either text type and reports which.
int packText(uint8_t type, const char *txt, uint8_t seq,
             uint8_t out[kMaxFrame]);
bool parseText(const uint8_t *buf, int len, char *txt, int txt_cap,
               uint8_t *seq, uint8_t *type);

}  // namespace lc
