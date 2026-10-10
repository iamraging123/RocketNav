// cfgcodec - the on-flash config RECORD format, pure and host-testable.
// cfgstore.cpp does the HAL flash I/O; everything that decides what bytes
// mean (layout, CRC, v1->v2->v3 migration, sanity) lives here so it can be
// tested off-target with plain byte buffers. A persistence bug hides in
// exactly this logic, so it is the part under test.
#pragma once

#include <stdint.h>

#include "linkage.h"

namespace cfgcodec {

extern const uint32_t kMagicV1;  // "RNV1": mag hard-iron only (legacy)
extern const uint32_t kMagicV2;  // "RNV2": mag + 4 linkage tables
extern const uint32_t kMagicV3;  // "RNV3": + the canard fin map

struct FinRec {
  uint8_t n;
  uint8_t pad[3];
  float deg[5];
  float us[5];
};

// Which PCA channel drives each canard index, and which way that fin swings
// for a rising pulse. Direction is the SIGN of servo microseconds per canard
// degree; +1 means a positive canard degree swings the leading edge
// clockwise seen from behind the rocket, which is the positive roll sense.
// A calibrated linkage table carries the same direction in its own slope -
// this is what an UNCALIBRATED fin falls back to, and what the bench panel
// records while mapping.
struct FinMap {
  uint8_t ch[4];
  int8_t dir[4];   // +1 / -1; 0 = unset, caller substitutes its compiled default
};

struct Record {
  uint32_t magic;
  float hard[3];
  FinRec fin[4];
  FinMap map;
  uint32_t crc;  // over hard[] + fin[] + map (v3); over hard[] + fin[] in v2
};

uint32_t crc32(const uint8_t *p, uint32_t n);
uint32_t recordCrc(const Record &r);
bool magSane(const float h[3]);
// Channels 0-15 and all four distinct, directions +1/-1. A record that fails
// this is ignored rather than flown: a duplicated channel silently leaves one
// canard unpowered and doubles another.
bool mapSane(const FinMap &m);
void mapDefaults(FinMap *m);   // ch {0,1,2,3}, dir all 0 = unset

// Parse a flash image (>= sizeof(Record) bytes). Always fills out (zeroed,
// magic V3, fins with insane data forced to n=0, map defaulted). Sets
// mag_ok / lnk_ok / map_ok. Returns the detected version: 0 blank/corrupt,
// 1 = v1, 2 = v2, 3 = v3.
int parse(const uint8_t *img, Record *out, bool *mag_ok, bool *lnk_ok,
          bool *map_ok);

// Stamp magic V3 + CRC, ready to write.
void seal(Record *r);

// Convenience conversions to/from the flight LinkageCal type.
void toLinkage(const Record &r, LinkageCal lc[4]);
void fromLinkage(Record *r, const LinkageCal lc[4]);

}  // namespace cfgcodec
