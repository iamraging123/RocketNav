// Host tests for the LoRa air-protocol codec. Build:
//   zig c++ -std=c++17 -O2 -I../firmware/RocketNav -o link_test.exe \
//     link_test.cpp ../firmware/RocketNav/linkcodec.cpp
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "linkcodec.h"

static int checks = 0, fails = 0;
#define CHECK(cond, ...)                    \
  do {                                      \
    checks++;                               \
    if (cond) {                             \
      printf("  ok  : ");                   \
    } else {                                \
      fails++;                              \
      printf("  FAIL: ");                   \
    }                                       \
    printf(__VA_ARGS__);                    \
    printf("\n");                           \
  } while (0)

int main() {
  printf("== crc16 ==\n");
  {
    // CRC-16/CCITT-FALSE check value for "123456789" is 0x29B1.
    CHECK(lc::crc16((const uint8_t *)"123456789", 9) == 0x29B1,
          "known vector 0x29B1");
  }

  printf("== state frame round trip ==\n");
  {
    lc::StateFields f;
    memset(&f, 0, sizeof(f));
    f.ms = 123456789u;
    f.fst = 3;
    f.fix = true;
    f.cmode = 2;
    float eul[3] = { -179.98f, 45.67f, 91.23f };
    float gyr[3] = { -250.4f, 3.1f, 720.9f };
    float acc[3] = { 15.27f, -0.02f, 1.0f };
    float vel[3] = { 88.4f, -3.3f, -102.7f };
    for (int i = 0; i < 3; ++i) {
      f.eul_deg[i] = eul[i];
      f.gyr_dps[i] = gyr[i];
      f.acc_g[i] = acc[i];
      f.vel_mps[i] = vel[i];
    }
    f.ral_m = 1234.4f;
    f.lat_deg = 40.1164017;
    f.lon_deg = -88.2434000;
    f.alt_msl_m = 222.4f;
    f.sats = 17;
    f.hacc_m = 1.44f;
    f.health = 0x0B;
    float cd[4] = { -9.75f, 3.25f, 0.25f, 10.0f };
    for (int i = 0; i < 4; ++i) f.cdef_deg[i] = cd[i];

    uint8_t buf[lc::kMaxFrame];
    int n = lc::packState(f, 42, buf);
    CHECK(n == lc::kStateFrameLen, "frame length %d", n);

    lc::StateFields g;
    uint8_t seq = 0;
    CHECK(lc::parseState(buf, n, &g, &seq) && seq == 42, "parses, seq kept");
    bool ok = g.ms == f.ms && g.fst == 3 && g.fix && g.cmode == 2 &&
              g.sats == 17 && g.health == 0x0B;
    CHECK(ok, "discrete fields exact");
    float e = 0;
    for (int i = 0; i < 3; ++i) {
      e = fmaxf(e, fabsf(g.eul_deg[i] - eul[i]));
    }
    CHECK(e <= 0.005f + 1e-4f, "euler within half-LSB (%.4f deg)", e);
    e = 0;
    for (int i = 0; i < 3; ++i) e = fmaxf(e, fabsf(g.gyr_dps[i] - gyr[i]));
    CHECK(e <= 0.05f + 1e-3f, "gyro within half-LSB (%.3f dps)", e);
    CHECK(fabsf(g.lat_deg - f.lat_deg) < 1e-6 &&
              fabsf(g.lon_deg - f.lon_deg) < 1e-6,
          "lat/lon to 1e-7 deg (%.8f %.8f)", g.lat_deg, g.lon_deg);
    CHECK(fabsf(g.ral_m - f.ral_m) <= 0.25f && g.alt_msl_m == 222.0f,
          "altitudes quantized as specified");
    CHECK(fabsf(g.hacc_m - 1.4f) < 0.06f, "hacc 0.1 m LSB (%.2f)", g.hacc_m);
    e = 0;
    for (int i = 0; i < 4; ++i) e = fmaxf(e, fabsf(g.cdef_deg[i] - cd[i]));
    CHECK(e <= 0.125f + 1e-4f, "deflections within half-LSB (%.3f deg)", e);

    // Saturation, not wraparound, on out-of-range values.
    f.gyr_dps[0] = 9999.0f;
    lc::packState(f, 1, buf);
    lc::parseState(buf, lc::kStateFrameLen, &g, &seq);
    CHECK(fabsf(g.gyr_dps[0] - 3276.7f) < 0.2f, "overrange saturates (%.1f)",
          g.gyr_dps[0]);

    // Corruption must be caught.
    lc::packState(f, 7, buf);
    buf[20] ^= 0x40;
    CHECK(!lc::parseState(buf, lc::kStateFrameLen, &g, &seq),
          "flipped bit rejected by crc");
    CHECK(!lc::parseState(buf, 30, &g, &seq), "truncated frame rejected");
  }

  printf("== text frames ==\n");
  {
    uint8_t buf[lc::kMaxFrame];
    char out[lc::kMaxText + 1];
    uint8_t seq = 0, type = 0;

    int n = lc::packText(lc::kTypeCmd, "$led all 128 0 255", 9, buf);
    CHECK(lc::parseText(buf, n, out, sizeof(out), &seq, &type) &&
              type == lc::kTypeCmd && seq == 9 &&
              strcmp(out, "$led all 128 0 255") == 0,
          "command round trip");

    // 60-char text truncates to the 48-byte payload cap.
    const char *lng =
        "cal: done (clean static) - heading 275 deg (mag) EXTRA TAIL";
    n = lc::packText(lc::kTypeMsg, lng, 10, buf);
    CHECK(n == 6 + lc::kMaxText, "long text truncated at pack");
    CHECK(lc::parseText(buf, n, out, sizeof(out), &seq, &type) &&
              (int)strlen(out) == lc::kMaxText &&
              strncmp(out, lng, lc::kMaxText) == 0,
          "truncated text round trips");

    buf[5] ^= 0x01;
    CHECK(!lc::parseText(buf, n, out, sizeof(out), &seq, &type),
          "corrupt text frame rejected");

    n = lc::packText(lc::kTypeMsg, "", 3, buf);
    CHECK(n == 6 && lc::parseText(buf, n, out, sizeof(out), &seq, &type) &&
              out[0] == 0,
          "empty text legal");

    // A state frame must not parse as text and vice versa.
    lc::StateFields f;
    memset(&f, 0, sizeof(f));
    uint8_t sbuf[lc::kMaxFrame];
    int sn = lc::packState(f, 0, sbuf);
    CHECK(!lc::parseText(sbuf, sn, out, sizeof(out), &seq, &type),
          "state frame refused by text parser");
    lc::StateFields g;
    CHECK(!lc::parseState(buf, n, &g, &seq),
          "text frame refused by state parser");
  }

  printf("\n%d checks, %d failed -> %s\n", checks, fails,
         fails ? "FAIL" : "PASS");
  return fails ? 1 : 0;
}
