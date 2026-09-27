#include "fm24cl16b.h"

bool Fm24cl16b::begin(TwoWire *w, uint8_t base_addr) {
  w_ = w;
  base_ = base_addr;
  errors_ = 0;
  uint8_t b = 0;
  present_ = true;  // read() refuses when absent; allow the probe through
  present_ = read(0, &b, 1);
  if (!present_) errors_ = 0;  // a missing chip is not an error count
  return present_;
}

bool Fm24cl16b::read(uint16_t addr, uint8_t *buf, uint16_t n) {
  if (!present_ || w_ == nullptr) return false;
  while (n > 0) {
    if (addr >= kSize) return false;
    uint8_t dev = (uint8_t)(base_ | ((addr >> 8) & 0x07));
    uint8_t word = (uint8_t)(addr & 0xFF);
    uint16_t room = (uint16_t)(256 - word);
    uint16_t k = n < room ? n : room;
    if (k > 32) k = 32;
    w_->beginTransmission(dev);
    w_->write(word);
    if (w_->endTransmission(false) != 0) { errors_++; return false; }
    int got = w_->requestFrom((int)dev, (int)k);
    if (got != (int)k) { errors_++; return false; }
    for (uint16_t i = 0; i < k; ++i) buf[i] = (uint8_t)w_->read();
    addr = (uint16_t)(addr + k);
    buf += k;
    n = (uint16_t)(n - k);
  }
  return true;
}

bool Fm24cl16b::write(uint16_t addr, const uint8_t *buf, uint16_t n) {
  if (!present_ || w_ == nullptr) return false;
  while (n > 0) {
    if (addr >= kSize) return false;
    uint8_t dev = (uint8_t)(base_ | ((addr >> 8) & 0x07));
    uint8_t word = (uint8_t)(addr & 0xFF);
    uint16_t room = (uint16_t)(256 - word);
    uint16_t k = n < room ? n : room;
    if (k > kChunk) k = kChunk;
    w_->beginTransmission(dev);
    w_->write(word);
    for (uint16_t i = 0; i < k; ++i) w_->write(buf[i]);
    if (w_->endTransmission(true) != 0) { errors_++; return false; }
    addr = (uint16_t)(addr + k);
    buf += k;
    n = (uint16_t)(n - k);
  }
  return true;
}
