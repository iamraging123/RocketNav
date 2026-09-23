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

  printf("== state frame: uplink rssi + 3-bit cmode ==\n");
  {
    lc::StateFields f, g;
    memset(&f, 0, sizeof(f));
    uint8_t buf[lc::kMaxFrame];
    uint8_t seq = 0;
    f.cmode = 4;  // CM_BENCH: a live wire value that needs the third bit
    f.up_rssi_dbm = -87;
    int n = lc::packState(f, 5, buf);
    CHECK(n == lc::kStateFrameLen && lc::parseState(buf, n, &g, &seq),
          "54-byte frame round trips");
    CHECK(g.cmode == 4, "cmode 4 (BENCH) survives the wire (got %u)", g.cmode);
    CHECK(g.up_rssi_dbm == -87, "uplink rssi -87 round trips (%d)",
          g.up_rssi_dbm);
    f.up_rssi_dbm = 0;
    lc::packState(f, 6, buf);
    lc::parseState(buf, lc::kStateFrameLen, &g, &seq);
    CHECK(g.up_rssi_dbm == 0, "no-uplink marker (0) round trips");
    f.up_rssi_dbm = -300;
    lc::packState(f, 7, buf);
    lc::parseState(buf, lc::kStateFrameLen, &g, &seq);
    CHECK(g.up_rssi_dbm == -255, "uplink rssi saturates at -255 (%d)",
          g.up_rssi_dbm);
    // Legacy 53-byte frame (no uplink-rssi byte): rebuild its CRC and parse.
    f.up_rssi_dbm = -50;
    f.cmode = 2;
    int m = lc::packState(f, 8, buf) - 1;   // drop the rssi byte
    uint16_t c = lc::crc16(buf, m - 2);
    buf[m - 2] = (uint8_t)(c >> 8);
    buf[m - 1] = (uint8_t)(c & 0xFF);
    CHECK(lc::parseState(buf, m, &g, &seq) && g.cmode == 2 &&
              g.up_rssi_dbm == 0 && seq == 8,
          "legacy 53-byte frame still parses (no uplink rssi)");
    CHECK(!lc::parseState(buf, m - 1, &g, &seq), "52 bytes rejected");
  }

  printf("== beacon frame ==\n");
  {
    lc::BeaconFields b, c;
    memset(&b, 0, sizeof(b));
    b.ms = 987654321u;
    b.fst = 4;
    b.fix = true;
    b.cmode = 3;
    b.lat_deg = 40.1164017;
    b.lon_deg = -88.2434000;
    b.alt_msl_m = 231.6f;
    b.ral_m = 12.7f;
    b.eul_deg[0] = -179.0f;
    b.eul_deg[1] = 45.0f;
    b.eul_deg[2] = 123.0f;
    b.sats = 11;
    b.health = 0x0F;
    uint8_t buf[lc::kMaxFrame];
    uint8_t seq = 0;
    int n = lc::packBeacon(b, 200, buf);
    CHECK(n == lc::kBeaconFrameLen, "beacon length %d (27)", n);
    CHECK(lc::parseBeacon(buf, n, &c, &seq) && seq == 200, "parses, seq kept");
    CHECK(c.ms == b.ms && c.fst == 4 && c.fix && c.cmode == 3 &&
              c.sats == 11 && c.health == 0x0F,
          "discrete fields exact");
    CHECK(fabs(c.lat_deg - b.lat_deg) < 1e-6 && fabs(c.lon_deg - b.lon_deg) < 1e-6,
          "lat/lon to 1e-7 deg");
    CHECK(c.alt_msl_m == 232.0f && fabsf(c.ral_m - 12.5f) < 1e-3f,
          "alt 1 m, ral 0.5 m (%.1f %.1f)", c.alt_msl_m, c.ral_m);
    float e = 0;
    for (int i = 0; i < 3; ++i) e = fmaxf(e, fabsf(c.eul_deg[i] - b.eul_deg[i]));
    CHECK(e <= 1.0f + 1e-4f, "euler within 1 deg (2 deg LSB): %.2f", e);
    buf[9] ^= 0x08;
    CHECK(!lc::parseBeacon(buf, n, &c, &seq), "corrupt beacon rejected");
    lc::packBeacon(b, 1, buf);
    CHECK(!lc::parseBeacon(buf, n - 1, &c, &seq), "short beacon rejected");
    lc::StateFields g;
    CHECK(!lc::parseState(buf, n, &g, &seq), "beacon refused by state parser");
    char out[lc::kMaxText + 1];
    uint8_t type = 0;
    CHECK(!lc::parseText(buf, n, out, sizeof(out), &seq, &type),
          "beacon refused by text parser");
  }

  printf("== keepalive frame ==\n");
  {
    uint8_t buf[lc::kMaxFrame];
    uint8_t seq = 0;
    int n = lc::packKeepalive(77, buf);
    CHECK(n == lc::kKeepaliveFrameLen, "keepalive length %d (5)", n);
    CHECK(lc::parseKeepalive(buf, n, &seq) && seq == 77, "round trip");
    buf[2] ^= 0x01;
    CHECK(!lc::parseKeepalive(buf, n, &seq), "corrupt keepalive rejected");
    char out[lc::kMaxText + 1];
    uint8_t type = 0;
    lc::packKeepalive(1, buf);
    CHECK(!lc::parseText(buf, n, out, sizeof(out), &seq, &type),
          "keepalive refused by text parser");
    lc::BeaconFields b;
    CHECK(!lc::parseBeacon(buf, n, &b, &seq), "keepalive refused by beacon parser");
  }

  printf("== profiles ==\n");
  {
    CHECK(lc::kProfileFlight.sf == LC_LORA_SF &&
              lc::kProfileFlight.bw_hz == LC_LORA_BW_HZ &&
              lc::kProfileFlight.cr_denom == LC_LORA_CR_DENOM &&
              lc::kProfileFlight.preamble == LC_LORA_PREAMBLE &&
              lc::kProfileFlight.period_us == LC_TX_PERIOD_US,
          "flight profile equals the boot-time macros");
    CHECK(lc::symbolTimeUs(7, 500000ul) == 256, "SF7/500k symbol 256 us");
    CHECK(lc::symbolTimeUs(11, 125000ul) == 16384,
          "SF11/125k symbol 16384 us (> 16 ms: needs LowDataRateOptimize)");
    CHECK(lc::symbolTimeUs(lc::kProfileRecovery.sf, lc::kProfileRecovery.bw_hz)
              > 16000ul,
          "recovery profile is in the low-data-rate regime");
  }

  printf("\n%d checks, %d failed -> %s\n", checks, fails,
         fails ? "FAIL" : "PASS");
  return fails ? 1 : 0;
}
