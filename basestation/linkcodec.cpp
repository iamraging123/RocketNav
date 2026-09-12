#include "linkcodec.h"

#include <string.h>

namespace lc {

uint16_t crc16(const uint8_t *p, int n) {
  uint16_t c = 0xFFFF;
  for (int i = 0; i < n; ++i) {
    c ^= (uint16_t)p[i] << 8;
    for (int k = 0; k < 8; ++k) {
      c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
  }
  return c;
}

static int16_t sat16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)(v + (v >= 0 ? 0.5f : -0.5f));
}
static int8_t sat8(float v) {
  if (v > 127.0f) return 127;
  if (v < -128.0f) return -128;
  return (int8_t)(v + (v >= 0 ? 0.5f : -0.5f));
}
static void put16(uint8_t *p, int16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((uint16_t)v >> 8);
}
static void put32(uint8_t *p, int32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(((uint32_t)v >> 8) & 0xFF);
  p[2] = (uint8_t)(((uint32_t)v >> 16) & 0xFF);
  p[3] = (uint8_t)(((uint32_t)v >> 24) & 0xFF);
}
static int16_t get16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static int32_t get32(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                   ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

int packState(const StateFields &f, uint8_t seq, uint8_t out[kMaxFrame]) {
  out[0] = kMagic;
  out[1] = kTypeState;
  out[2] = seq;
  uint8_t *p = out + 3;
  put32(p, (int32_t)f.ms); p += 4;
  *p++ = (uint8_t)((f.fst & 0x07) | (f.fix ? 0x08 : 0) |
                   ((f.cmode & 0x03) << 4));
  for (int i = 0; i < 3; ++i) { put16(p, sat16(f.eul_deg[i] * 100.0f)); p += 2; }
  for (int i = 0; i < 3; ++i) { put16(p, sat16(f.gyr_dps[i] * 10.0f)); p += 2; }
  for (int i = 0; i < 3; ++i) { put16(p, sat16(f.acc_g[i] * 100.0f)); p += 2; }
  for (int i = 0; i < 3; ++i) { put16(p, sat16(f.vel_mps[i] * 10.0f)); p += 2; }
  put16(p, sat16(f.ral_m * 2.0f)); p += 2;
  put32(p, (int32_t)(f.lat_deg * 1e7)); p += 4;
  put32(p, (int32_t)(f.lon_deg * 1e7)); p += 4;
  put16(p, sat16(f.alt_msl_m)); p += 2;
  *p++ = f.sats;
  {
    float h = f.hacc_m * 10.0f;
    if (h < 0) h = 0;
    if (h > 255.0f) h = 255.0f;
    *p++ = (uint8_t)(h + 0.5f);
  }
  *p++ = f.health;
  for (int i = 0; i < 4; ++i) *p++ = (uint8_t)sat8(f.cdef_deg[i] * 4.0f);
  int n = (int)(p - out);
  uint16_t c = crc16(out, n);
  out[n] = (uint8_t)(c >> 8);
  out[n + 1] = (uint8_t)(c & 0xFF);
  return n + 2;
}

bool parseState(const uint8_t *buf, int len, StateFields *f, uint8_t *seq) {
  if (len != kStateFrameLen) return false;
  if (buf[0] != kMagic || buf[1] != kTypeState) return false;
  uint16_t c = crc16(buf, len - 2);
  if (buf[len - 2] != (uint8_t)(c >> 8) || buf[len - 1] != (uint8_t)(c & 0xFF))
    return false;
  *seq = buf[2];
  const uint8_t *p = buf + 3;
  f->ms = (uint32_t)get32(p); p += 4;
  f->fst = *p & 0x07;
  f->fix = (*p & 0x08) != 0;
  f->cmode = (*p >> 4) & 0x03;
  p++;
  for (int i = 0; i < 3; ++i) { f->eul_deg[i] = get16(p) * 0.01f; p += 2; }
  for (int i = 0; i < 3; ++i) { f->gyr_dps[i] = get16(p) * 0.1f; p += 2; }
  for (int i = 0; i < 3; ++i) { f->acc_g[i] = get16(p) * 0.01f; p += 2; }
  for (int i = 0; i < 3; ++i) { f->vel_mps[i] = get16(p) * 0.1f; p += 2; }
  f->ral_m = get16(p) * 0.5f; p += 2;
  f->lat_deg = get32(p) * 1e-7; p += 4;
  f->lon_deg = get32(p) * 1e-7; p += 4;
  f->alt_msl_m = (float)get16(p); p += 2;
  f->sats = *p++;
  f->hacc_m = (*p++) * 0.1f;
  f->health = *p++;
  for (int i = 0; i < 4; ++i) f->cdef_deg[i] = (int8_t)(*p++) * 0.25f;
  return true;
}

int packText(uint8_t type, const char *txt, uint8_t seq,
             uint8_t out[kMaxFrame]) {
  int n = 0;
  while (txt[n] != 0 && n < kMaxText) n++;
  out[0] = kMagic;
  out[1] = type;
  out[2] = seq;
  out[3] = (uint8_t)n;
  memcpy(out + 4, txt, (size_t)n);
  int total = 4 + n;
  uint16_t c = crc16(out, total);
  out[total] = (uint8_t)(c >> 8);
  out[total + 1] = (uint8_t)(c & 0xFF);
  return total + 2;
}

bool parseText(const uint8_t *buf, int len, char *txt, int txt_cap,
               uint8_t *seq, uint8_t *type) {
  if (len < 6 || buf[0] != kMagic) return false;
  if (buf[1] != kTypeMsg && buf[1] != kTypeCmd) return false;
  int n = buf[3];
  if (n > kMaxText || len != 6 + n) return false;
  uint16_t c = crc16(buf, len - 2);
  if (buf[len - 2] != (uint8_t)(c >> 8) || buf[len - 1] != (uint8_t)(c & 0xFF))
    return false;
  *type = buf[1];
  *seq = buf[2];
  int m = (n < txt_cap - 1) ? n : txt_cap - 1;
  memcpy(txt, buf + 4, (size_t)m);
  txt[m] = 0;
  return true;
}

}  // namespace lc
