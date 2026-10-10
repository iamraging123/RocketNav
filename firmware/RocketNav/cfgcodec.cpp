#include "cfgcodec.h"

#include <string.h>

namespace cfgcodec {

const uint32_t kMagicV1 = 0x524E5631UL;  // "RNV1"
const uint32_t kMagicV2 = 0x524E5632UL;  // "RNV2"
const uint32_t kMagicV3 = 0x524E5633UL;  // "RNV3"

// The v2 image on flash is SHORTER than a v3 Record and puts its CRC where
// v3 keeps the fin map, so a v2 sector must be read through its own layout.
struct RecordV2 {
  uint32_t magic;
  float hard[3];
  FinRec fin[4];
  uint32_t crc;
};

uint32_t crc32(const uint8_t *p, uint32_t n) {
  uint32_t c = 0xFFFFFFFFUL;
  for (uint32_t i = 0; i < n; ++i) {
    c ^= p[i];
    for (int k = 0; k < 8; ++k) c = (c >> 1) ^ (0xEDB88320UL & (0UL - (c & 1)));
  }
  return c ^ 0xFFFFFFFFUL;
}

uint32_t recordCrc(const Record &r) {
  return crc32((const uint8_t *)&r.hard,
               sizeof(r.hard) + sizeof(r.fin) + sizeof(r.map));
}

static uint32_t recordCrcV2(const RecordV2 &r) {
  return crc32((const uint8_t *)&r.hard, sizeof(r.hard) + sizeof(r.fin));
}

bool mapSane(const FinMap &m) {
  for (int i = 0; i < 4; ++i) {
    if (m.ch[i] > 15) return false;
    if (m.dir[i] != 1 && m.dir[i] != -1) return false;
    for (int j = i + 1; j < 4; ++j) {
      if (m.ch[i] == m.ch[j]) return false;
    }
  }
  return true;
}

void mapDefaults(FinMap *m) {
  for (int i = 0; i < 4; ++i) {
    m->ch[i] = (uint8_t)i;
    m->dir[i] = 0;  // unset: the caller substitutes its compiled default
  }
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

static void adoptFins(Record *out, bool *lnk_ok) {
  for (int i = 0; i < 4; ++i) {
    if (!finSane(out->fin[i])) out->fin[i].n = 0;
    if (out->fin[i].n >= 2) *lnk_ok = true;
  }
}

int parse(const uint8_t *img, Record *out, bool *mag_ok, bool *lnk_ok,
          bool *map_ok) {
  *mag_ok = false;
  *lnk_ok = false;
  *map_ok = false;
  memset(out, 0, sizeof(Record));
  out->magic = kMagicV3;
  mapDefaults(&out->map);

  uint32_t magic;
  memcpy(&magic, img, sizeof(magic));

  if (magic == kMagicV3) {
    Record r;
    memcpy(&r, img, sizeof(Record));
    if (recordCrc(r) == r.crc && magSane(r.hard)) {
      *out = r;
      out->magic = kMagicV3;
      *mag_ok = true;
      adoptFins(out, lnk_ok);
      if (mapSane(out->map)) *map_ok = true;
      else mapDefaults(&out->map);
      return 3;
    }
    return 0;
  }

  if (magic == kMagicV2) {
    RecordV2 r;
    memcpy(&r, img, sizeof(RecordV2));
    if (recordCrcV2(r) == r.crc && magSane(r.hard)) {
      memcpy(out->hard, r.hard, sizeof(out->hard));
      memcpy(out->fin, r.fin, sizeof(out->fin));
      *mag_ok = true;
      adoptFins(out, lnk_ok);
      // no map in a v2 record: defaults stand, first save upgrades to v3
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
  r->magic = kMagicV3;
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
