// cfgcodec - the on-flash config RECORD format, pure and host-testable.
// cfgstore.cpp does the HAL flash I/O; everything that decides what bytes
// mean (layout, CRC, v1->v2 migration, sanity) lives here so it can be
// tested off-target with plain byte buffers. A persistence bug hides in
// exactly this logic, so it is the part under test.
#pragma once

#include <stdint.h>

#include "linkage.h"

namespace cfgcodec {

extern const uint32_t kMagicV1;  // "RNV1": mag hard-iron only (legacy)
extern const uint32_t kMagicV2;  // "RNV2": mag + 4 linkage tables

struct FinRec {
  uint8_t n;
  uint8_t pad[3];
  float deg[5];
  float us[5];
};

struct Record {
  uint32_t magic;
  float hard[3];
  FinRec fin[4];
  uint32_t crc;  // over hard[] + fin[]
};

uint32_t crc32(const uint8_t *p, uint32_t n);
uint32_t recordCrc(const Record &r);
bool magSane(const float h[3]);

// Parse a flash image (>= sizeof(Record) bytes). Always fills out (zeroed,
// magic V2, fins with insane data forced to n=0). Sets mag_ok / lnk_ok.
// Returns the detected version: 0 blank/corrupt, 1 = v1, 2 = v2.
int parse(const uint8_t *img, Record *out, bool *mag_ok, bool *lnk_ok);

// Stamp magic V2 + CRC, ready to write.
void seal(Record *r);

// Convenience conversions to/from the flight LinkageCal type.
void toLinkage(const Record &r, LinkageCal lc[4]);
void fromLinkage(Record *r, const LinkageCal lc[4]);

}  // namespace cfgcodec
