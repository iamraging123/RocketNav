// cfgstore - on-chip flash record for calibration that must survive reset
// and power cycles: the mag hard-iron offset and the four canard linkage
// tables. One CRC-protected record in the F722's last 128 KB sector
// (sector 7, 0x08060000); the firmware image (~135 KB) never reaches it.
//
// The whole record is cached in RAM at first access and rewritten as a
// unit on every save, so saving one calibration never loses the other.
// A v1 record (mag only, from older firmware) is read transparently; the
// first save upgrades it to v2. Erasing the sector stalls the CPU for up
// to ~2 s (code executes from the same flash bank) - callers only save on
// the bench, on an explicit ground command, never in flight.
#pragma once

#include <stdint.h>

#include "linkage.h"

namespace cfgstore {

// Mag hard-iron (uT, body frame). load returns false when nothing valid is
// stored; hard_ut is untouched then.
bool loadMagHard(float hard_ut[3]);
bool saveMagHard(const float hard_ut[3]);

// Canard linkage tables. load returns false when no stored fin is
// calibrated (lc still receives whatever is stored, n=0 for absent fins).
bool loadLinkage(LinkageCal lc[4]);
bool saveLinkage(const LinkageCal lc[4]);

// Non-destructive report of what is PHYSICALLY in the flash sector right
// now (re-read past the cache) - the on-hardware "did the save persist?"
// diagnostic behind `$lcal flash`.
struct FlashInfo {
  uint32_t magic;   // raw first word (0xFFFFFFFF = blank/erased)
  int version;      // 0 blank/corrupt, 1 = v1, 2 = v2
  bool crc_ok;
  bool mag_ok;
  bool lnk_ok;
  uint8_t fin_n[4]; // stored point count per fin
  float hard[3];
};
bool inspectRaw(FlashInfo *out);

}  // namespace cfgstore
