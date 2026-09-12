#include "cfgcodec.h"

#include <string.h>

namespace cfgcodec {

const uint32_t kMagicV1 = 0x524E5631UL;  // "RNV1"
const uint32_t kMagicV2 = 0x524E5632UL;  // "RNV2"

uint32_t crc32(const uint8_t *p, uint32_t n) {
  uint32_t c = 0xFFFFFFFFUL;
  for (uint32_t i = 0; i < n; ++i) {
    c ^= p[i];
    for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320UL & (0UL - (c & 1)));
  }
  return c ^ 0xFFFFFFFFUL;
}

uint32_t recordCrc(const Record &r) {
  return crc32((const uint8_t *)&r.hard, sizeof(r.hard) + sizeof(r.fin));
}

bool magSane(const float h[3]) {
  for (int i = 0; i < 3; ++i) {
    if (!(h[i] > -500.0f && h[i] < 500.0f)) return false;
  }
  return true;
}

static bool finSane(const FinRec &f) {
  if (f.n > 5) return false;
  for (int i = 0; i < (int)f.n; ++i) {
    if (!(f.deg[i] > -90.0f && f.deg[i] < 90.0f)) return false;
    if (!(f.us[i] > 500.0f && f.us[i] < 2500.0f)) return false;
  }
  return true;
}

int parse(const uint8_t *img, Record *out, bool *mag_ok, bool *lnk_ok) {
  *mag_ok = false;
  *lnk_ok = false;
  memset(out, 0, sizeof(Record));
  out->magic = kMagicV2;

  uint32_t magic;
  memcpy(&magic, img, sizeof(magic));

  if (magic == kMagicV2) {
    Record r;
    memcpy(&r, img, sizeof(Record));
    if (recordCrc(r) == r.crc && magSane(r.hard)) {
      *out = r;
      out->magic = kMagicV2;
      *mag_ok = true;
      for (int i = 0; i < 4; ++i) {
        if (!finSane(out->fin[i])) out->fin[i].n = 0;
        if (out->fin[i].n >= 2) *lnk_ok = true;
      }
      return 2;
    }
    return 0;  // v2 magic but bad CRC / insane mag: treat as blank
  }

  if (magic == kMagicV1) {
    // Legacy layout: uint32 magic, float hard[3], uint32 crc-over-hard.
    float hard[3];
    uint32_t crc;
    memcpy(hard, img + 4, sizeof(hard));
    memcpy(&crc, img + 16, sizeof(crc));
    if (crc32((const uint8_t *)hard, sizeof(hard)) == crc && magSane(hard)) {
      memcpy(out->hard, hard, sizeof(hard));
      *mag_ok = true;  // linkage stays uncalibrated; first save upgrades
      return 1;
    }
  }
  return 0;
}

void seal(Record *r) {
  r->magic = kMagicV2;
  r->crc = recordCrc(*r);
}

void toLinkage(const Record &r, LinkageCal lc[4]) {
  for (int i = 0; i < 4; ++i) {
    lc[i].n = r.fin[i].n;
    memcpy(lc[i].deg, r.fin[i].deg, sizeof(lc[i].deg));
    memcpy(lc[i].us, r.fin[i].us, sizeof(lc[i].us));
  }
}

void fromLinkage(Record *r, const LinkageCal lc[4]) {
  for (int i = 0; i < 4; ++i) {
    r->fin[i].n = lc[i].n;
    memset(r->fin[i].pad, 0, sizeof(r->fin[i].pad));
    memcpy(r->fin[i].deg, lc[i].deg, sizeof(r->fin[i].deg));
    memcpy(r->fin[i].us, lc[i].us, sizeof(r->fin[i].us));
  }
}

}  // namespace cfgcodec
