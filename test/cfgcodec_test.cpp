// Host tests for the config-record codec: proves the save/load LOGIC
// (layout, CRC, v1->v2 migration, mag+linkage coexistence, sanity) that a
// "not saving to flash" bug would live in. If these pass and hardware still
// loses data, the fault is the flash I/O / board, not this logic.
// Build:
//   zig c++ -std=c++17 -O2 -I../firmware/RocketNav -o cfgcodec_test.exe \
//     cfgcodec_test.cpp ../firmware/RocketNav/cfgcodec.cpp \
//     ../firmware/RocketNav/linkage.cpp
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "cfgcodec.h"
#include "linkage.h"

static int checks = 0, fails = 0;
#define CHECK(cond, ...)                    \
  do {                                      \
    checks++;                               \
    if (cond) { printf("  ok  : "); }       \
    else { fails++; printf("  FAIL: "); }   \
    printf(__VA_ARGS__);                    \
    printf("\n");                           \
  } while (0)

// A byte buffer standing in for the flash sector.
static uint8_t flash[sizeof(cfgcodec::Record)];

static bool mp = false;   // map_ok out-param for parse()
static void blank() { memset(flash, 0xFF, sizeof(flash)); }
static void writeRecord(const cfgcodec::Record &r) {
  memcpy(flash, &r, sizeof(r));
}

int main() {
  printf("== blank / corrupt ==\n");
  {
    blank();
    cfgcodec::Record r;
    bool m, l;
    int v = cfgcodec::parse(flash, &r, &m, &l, &mp);
    CHECK(v == 0 && !m && !l, "erased sector = blank, nothing stored");
  }

  printf("== v2 round trip: mag + 3-point table ==\n");
  {
    cfgcodec::Record r;
    memset(&r, 0, sizeof(r));
    float hard[3] = { 12.5f, -7.25f, 3.0f };
    LinkageCal lc[4];
    for (int i = 0; i < 4; ++i) lc[i].n = 0;
    lc[1].n = 3;
    lc[1].deg[0] = -12; lc[1].us[0] = 1180;
    lc[1].deg[1] = 0;   lc[1].us[1] = 1462;   // offset neutral
    lc[1].deg[2] = 12;  lc[1].us[2] = 1760;
    memcpy(r.hard, hard, sizeof(hard));
    cfgcodec::fromLinkage(&r, lc);
    cfgcodec::seal(&r);
    writeRecord(r);

    cfgcodec::Record g;
    bool m, l;
    int v = cfgcodec::parse(flash, &g, &m, &l, &mp);
    CHECK(v == 3 && m && l, "parses as v3, mag+linkage present");
    CHECK(fabsf(g.hard[0] - 12.5f) < 1e-6f && fabsf(g.hard[1] + 7.25f) < 1e-6f,
          "hard-iron survives the round trip");
    LinkageCal out[4];
    cfgcodec::toLinkage(g, out);
    CHECK(out[1].n == 3 && out[1].deg[1] == 0 && out[1].us[1] == 1462 &&
              out[0].n == 0,
          "fin 1 table exact, fin 0 stays uncalibrated");
    // The stored neutral really is 1462, not the 1500 default: prove the
    // interpolation reads the calibrated zero (the user's actual symptom).
    CHECK(fabsf(linkage::deflToUs(out[1], 0.0f) - 1462.0f) < 1e-3f,
          "canard 0 deg -> stored 1462 us (offset persisted)");
  }

  printf("== corruption rejected ==\n");
  {
    cfgcodec::Record r;
    memset(&r, 0, sizeof(r));
    r.hard[0] = 5.0f;
    cfgcodec::seal(&r);
    writeRecord(r);
    flash[8] ^= 0x20;  // flip a bit in the payload
    cfgcodec::Record g;
    bool m, l;
    int v = cfgcodec::parse(flash, &g, &m, &l, &mp);
    CHECK(v == 0 && !m, "bit flip fails CRC -> treated as blank");
  }

  printf("== v1 legacy record migrates ==\n");
  {
    // Hand-build the old layout: magic, hard[3], crc32(hard).
    blank();
    uint32_t magic = cfgcodec::kMagicV1;
    float hard[3] = { -3.0f, 44.0f, 9.5f };
    uint32_t crc = cfgcodec::crc32((const uint8_t *)hard, sizeof(hard));
    memcpy(flash + 0, &magic, 4);
    memcpy(flash + 4, hard, 12);
    memcpy(flash + 16, &crc, 4);
    cfgcodec::Record g;
    bool m, l;
    int v = cfgcodec::parse(flash, &g, &m, &l, &mp);
    CHECK(v == 1 && m && !l, "v1 reads as mag-only");
    CHECK(fabsf(g.hard[1] - 44.0f) < 1e-6f, "v1 hard-iron recovered");
    LinkageCal out[4];
    cfgcodec::toLinkage(g, out);
    CHECK(out[0].n == 0 && out[3].n == 0, "no linkage in a v1 record");
  }

  printf("== coexistence: linkage save keeps mag, and vice versa ==\n");
  {
    // Start from a v1 mag-only record (the realistic upgrade path).
    blank();
    uint32_t magic = cfgcodec::kMagicV1;
    float hard[3] = { 1.0f, 2.0f, 3.0f };
    uint32_t crc = cfgcodec::crc32((const uint8_t *)hard, sizeof(hard));
    memcpy(flash, &magic, 4);
    memcpy(flash + 4, hard, 12);
    memcpy(flash + 16, &crc, 4);

    // Load (as cfgstore would), add a linkage table, reseal, rewrite.
    cfgcodec::Record rec;
    bool m, l;
    cfgcodec::parse(flash, &rec, &m, &l, &mp);
    LinkageCal lc[4];
    cfgcodec::toLinkage(rec, lc);
    lc[0].n = 2;
    lc[0].deg[0] = -10; lc[0].us[0] = 1200;
    lc[0].deg[1] = 10;  lc[0].us[1] = 1800;
    cfgcodec::fromLinkage(&rec, lc);
    cfgcodec::seal(&rec);
    writeRecord(rec);

    cfgcodec::Record g;
    cfgcodec::parse(flash, &g, &m, &l, &mp);
    CHECK(m && l && fabsf(g.hard[2] - 3.0f) < 1e-6f && g.fin[0].n == 2,
          "after linkage save: mag STILL there and linkage stored");

    // Now overwrite mag only; linkage must survive (the $magclr path).
    LinkageCal keep[4];
    cfgcodec::toLinkage(g, keep);
    float zero[3] = { 0, 0, 0 };
    memcpy(g.hard, zero, sizeof(zero));
    cfgcodec::seal(&g);
    writeRecord(g);
    cfgcodec::Record g2;
    cfgcodec::parse(flash, &g2, &m, &l, &mp);
    CHECK(l && g2.fin[0].n == 2 && g2.fin[0].us[1] == 1800,
          "after mag rewrite: linkage table SURVIVES");
  }

  printf("== insane fin quarantined, others kept ==\n");
  {
    cfgcodec::Record r;
    memset(&r, 0, sizeof(r));
    LinkageCal lc[4];
    for (int i = 0; i < 4; ++i) lc[i].n = 0;
    lc[2].n = 2; lc[2].deg[0] = -8; lc[2].us[0] = 1250;
    lc[2].deg[1] = 8; lc[2].us[1] = 1750;
    lc[3].n = 9;  // impossible point count
    cfgcodec::fromLinkage(&r, lc);
    cfgcodec::seal(&r);
    writeRecord(r);
    cfgcodec::Record g;
    bool m, l;
    cfgcodec::parse(flash, &g, &m, &l, &mp);
    CHECK(g.fin[2].n == 2 && g.fin[3].n == 0 && l,
          "bad fin 3 dropped to n=0, good fin 2 intact");
  }

  printf("== offset-only table (single point) ==\n");
  {
    // The one-point flow: default gain 10 us/deg, measured neutral 1462 at
    // 0 deg -> deflToUs(0) must be exactly 1462, and it must finalize.
    LinkageCal lc;
    linkage::synthOffset(lc, 0.0f, 1462.0f, 10.0f, 1000.0f, 2000.0f);
    CHECK(linkage::finalize(lc), "offset table finalizes");
    CHECK(fabsf(linkage::deflToUs(lc, 0.0f) - 1462.0f) < 1e-3f,
          "deflToUs(0) = measured neutral 1462");
    CHECK(fabsf(linkage::deflToUs(lc, 5.0f) - 1512.0f) < 1e-3f &&
              fabsf(linkage::deflToUs(lc, -5.0f) - 1412.0f) < 1e-3f,
          "default gain preserved around the new zero");
    // A non-zero measured angle also works (canard flush at a nonzero read).
    linkage::synthOffset(lc, 2.0f, 1600.0f, -12.0f, 1000.0f, 2000.0f);
    CHECK(linkage::finalize(lc) &&
              fabsf(linkage::deflToUs(lc, 2.0f) - 1600.0f) < 1e-3f,
          "offset works with negative gain and nonzero angle");
  }


  printf("== v3 fin map ==\n");
  {
    cfgcodec::Record r;
    memset(&r, 0, sizeof(r));
    float hard[3] = { 1.0f, 2.0f, 3.0f };
    memcpy(r.hard, hard, sizeof(hard));
    cfgcodec::mapDefaults(&r.map);
    uint8_t ch[4] = { 2, 0, 3, 1 };
    int8_t dir[4] = { 1, -1, 1, -1 };
    for (int i = 0; i < 4; ++i) { r.map.ch[i] = ch[i]; r.map.dir[i] = dir[i]; }
    cfgcodec::seal(&r);
    blank();
    writeRecord(r);
    cfgcodec::Record g;
    bool m, l;
    int v = cfgcodec::parse(flash, &g, &m, &l, &mp);
    bool same = true;
    for (int i = 0; i < 4; ++i) {
      if (g.map.ch[i] != ch[i] || g.map.dir[i] != dir[i]) same = false;
    }
    CHECK(v == 3 && mp && same, "fin map round trips through v3");

    // A duplicated channel is refused rather than flown: it would leave one
    // canard unpowered and double another.
    cfgcodec::Record d = r;
    d.map.ch[2] = d.map.ch[0];
    cfgcodec::seal(&d);
    blank();
    writeRecord(d);
    cfgcodec::Record g2;
    cfgcodec::parse(flash, &g2, &m, &l, &mp);
    CHECK(!mp && g2.map.ch[0] == 0 && g2.map.ch[3] == 3,
          "duplicate channel rejected, defaults restored");

    cfgcodec::Record z = r;
    z.map.dir[1] = 0;   // unset direction is not a usable map
    cfgcodec::seal(&z);
    blank();
    writeRecord(z);
    cfgcodec::parse(flash, &g2, &m, &l, &mp);
    CHECK(!mp, "dir 0 (unset) is not a sane map");

    cfgcodec::FinMap ok;
    cfgcodec::mapDefaults(&ok);
    CHECK(!cfgcodec::mapSane(ok), "defaults are not sane until a direction is set");
    for (int i = 0; i < 4; ++i) ok.dir[i] = 1;
    CHECK(cfgcodec::mapSane(ok), "defaults + directions are sane");
  }

  printf("== v2 image upgrades ==\n");
  {
    // A record written by the PREVIOUS firmware: shorter, CRC over
    // hard+fin only, and its crc word sits where v3 keeps the map.
    struct FinRecV2 { uint8_t n; uint8_t pad[3]; float deg[5]; float us[5]; };
    struct RecordV2 {
      uint32_t magic; float hard[3]; FinRecV2 fin[4]; uint32_t crc;
    } o;
    memset(&o, 0, sizeof(o));
    o.magic = cfgcodec::kMagicV2;
    o.hard[0] = 4.5f; o.hard[1] = -6.0f; o.hard[2] = 9.5f;
    o.fin[1].n = 2;
    o.fin[1].deg[0] = -10; o.fin[1].deg[1] = 10;
    o.fin[1].us[0] = 1400; o.fin[1].us[1] = 1600;
    o.crc = cfgcodec::crc32((const uint8_t *)&o.hard,
                            sizeof(o.hard) + sizeof(o.fin));
    blank();
    memcpy(flash, &o, sizeof(o));
    cfgcodec::Record g;
    bool m, l;
    int v = cfgcodec::parse(flash, &g, &m, &l, &mp);
    CHECK(v == 2 && m && l && !mp, "v2 image still readable, no map in it");
    CHECK(fabsf(g.hard[2] - 9.5f) < 1e-6f && g.fin[1].n == 2 &&
              g.fin[1].us[1] == 1600,
          "v2 mag and linkage survive the upgrade read");
    CHECK(g.map.ch[0] == 0 && g.map.ch[3] == 3 && g.map.dir[0] == 0,
          "v2 upgrade leaves the map at defaults, direction unset");
  }
  printf("\n%d checks, %d failed -> %s\n", checks, fails,
         fails ? "FAIL" : "PASS");
  return fails ? 1 : 0;
}
