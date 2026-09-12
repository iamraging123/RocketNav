// linkcodec - the LoRa air protocol, pure codec half. No hardware, no
// Arduino: the rocket, the base station, and the host tests all compile this
// same file, which is what keeps the two ends of the link in agreement.
//
// Frame: [0x4B][type][seq][payload...][crc16 hi][crc16 lo], max 64 bytes.
// Types: 'T' packed state snapshot (rocket -> ground, 4 Hz)
//        'M' text message, the firmware's msg records (rocket -> ground)
//        'C' command line "$..." (ground -> rocket)
// CRC-16/CCITT-FALSE over everything before the CRC itself.
#pragma once

#include <stdint.h>

namespace lc {

const uint8_t kMagic = 0x4B;
const uint8_t kTypeState = 'T';
const uint8_t kTypeMsg = 'M';
const uint8_t kTypeCmd = 'C';
const int kMaxFrame = 64;
const int kMaxText = 48;
const int kStateFrameLen = 53;  // 3 header + 48 payload + 2 crc

// Radio profile BOTH ends must share (SF7/BW500 = ~27 ms for a state frame,
// ~11% duty at 4 Hz; sensitivity ~ -117 dBm, ~35 dB margin at 5 km LOS).
#define LC_LORA_FREQ_HZ 433500000ul
#define LC_LORA_SF 7
#define LC_LORA_BW_HZ 500000ul
#define LC_LORA_CR_DENOM 5  // 4/5
#define LC_LORA_SYNC 0x4B
#define LC_LORA_PREAMBLE 8
#define LC_TX_PERIOD_US 250000ul  // rocket state-frame cadence (4 Hz)

// Everything the 48-byte state payload carries, in engineering units.
// Quantization (applied by packState, undone by parseState):
//   eul 0.01 deg | gyr 0.1 dps | acc 0.01 g | vel 0.1 m/s | ral 0.5 m
//   lat/lon 1e-7 deg | alt_msl 1 m | hacc 0.1 m (cap 25.5) | cdef 0.25 deg
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
};

uint16_t crc16(const uint8_t *p, int n);

// Returns the frame length written into out (kStateFrameLen).
int packState(const StateFields &f, uint8_t seq, uint8_t out[kMaxFrame]);
bool parseState(const uint8_t *buf, int len, StateFields *f, uint8_t *seq);

// type is kTypeMsg or kTypeCmd; text is truncated to kMaxText. Returns the
// frame length. parseText accepts either text type and reports which.
int packText(uint8_t type, const char *txt, uint8_t seq,
             uint8_t out[kMaxFrame]);
bool parseText(const uint8_t *buf, int len, char *txt, int txt_cap,
               uint8_t *seq, uint8_t *type);

}  // namespace lc
