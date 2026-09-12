#include "pca9685.h"

#define REG_MODE1 0x00
#define REG_LED0_ON_L 0x06
#define REG_ALL_LED_OFF_H 0xFD
#define REG_PRESCALE 0xFE

#define MODE1_SLEEP 0x10
#define MODE1_AI 0x20
#define MODE1_RESTART 0x80

bool Pca9685::wr(uint8_t reg, const uint8_t *p, int n) {
  wire_->beginTransmission(addr_);
  wire_->write(reg);
  for (int i = 0; i < n; ++i) wire_->write(p[i]);
  if (wire_->endTransmission() != 0) {
    errors_++;
    return false;
  }
  return true;
}

bool Pca9685::begin(TwoWire *wire, uint8_t addr, float frame_hz) {
  wire_ = wire;
  addr_ = addr;
  frame_hz_ = frame_hz;

  // Presence probe: address a MODE1 read.
  wire_->beginTransmission(addr_);
  wire_->write((uint8_t)REG_MODE1);
  if (wire_->endTransmission(false) != 0 ||
      wire_->requestFrom((int)addr_, 1) != 1) {
    wire_->endTransmission();  // release the bus after the failed repeat-start
    present_ = false;
    return false;
  }
  (void)wire_->read();

  present_ = true;  // probe passed; the prescale writes settle the rest
  if (!setFrameHz(frame_hz)) {
    present_ = false;
    return false;
  }
  return true;
}

bool Pca9685::wake() {
  uint8_t v = MODE1_AI;
  if (!wr(REG_MODE1, &v, 1)) return false;
  delay(1);  // oscillator wake (500 us per datasheet)
  v = MODE1_AI | MODE1_RESTART;
  return wr(REG_MODE1, &v, 1);
}

bool Pca9685::setFrameHz(float frame_hz) {
  if (!present_) return false;
  if (frame_hz < 24.0f) frame_hz = 24.0f;
  if (frame_hz > 1526.0f) frame_hz = 1526.0f;
  // Prescale is only writable asleep: PRE = round(25 MHz / (4096 f)) - 1.
  float pre = 25000000.0f / (4096.0f * frame_hz) - 1.0f;
  if (pre < 3.0f) pre = 3.0f;
  if (pre > 255.0f) pre = 255.0f;
  uint8_t pv = (uint8_t)(pre + 0.5f);

  // frame_hz_ is the basis of every us->counts conversion, so it must
  // track the CHIP, not the request: commit it only when the sequence
  // that changes the chip has fully succeeded, and leave the driver
  // matching whatever state an aborted sequence left behind.
  uint8_t v = MODE1_SLEEP | MODE1_AI;
  if (!wr(REG_MODE1, &v, 1)) return false;  // nothing changed on the chip
  bool pre_ok = wr(REG_PRESCALE, &pv, 1);
  if (!wake()) {
    // Chip is asleep - no pulses at all. Converting through ANY frame
    // would be a lie and silent dead servos are worse: report the part
    // absent so health flags it, arming refuses, and the recovery path
    // re-initializes from scratch.
    present_ = false;
    return false;
  }
  if (!pre_ok) return false;  // awake and unchanged at the old frame
  // Store the ACHIEVED frame, not the request: the integer prescale
  // quantizes it (196.9 Hz for a 200 Hz ask), and at 200 Hz converting
  // through the requested value would put ~2% on every pulse.
  frame_hz_ = 25000000.0f / (4096.0f * ((float)pv + 1.0f));
  return true;
}


bool Pca9685::setPwmUs(uint8_t ch, float us) {
  if (!present_ || ch > 15) return false;
  uint8_t d[4];
  if (us <= 0.0f) {
    d[0] = 0; d[1] = 0; d[2] = 0; d[3] = 0x10;  // channel full-off bit
  } else {
    float counts = us * frame_hz_ * 4096.0f / 1000000.0f;
    if (counts > 4095.0f) counts = 4095.0f;
    uint16_t off = (uint16_t)(counts + 0.5f);
    d[0] = 0; d[1] = 0;                    // ON = 0 (pulse starts at frame)
    d[2] = (uint8_t)(off & 0xFF);
    d[3] = (uint8_t)(off >> 8);
  }
  return wr((uint8_t)(REG_LED0_ON_L + 4 * ch), d, 4);
}

void Pca9685::allOff() {
  if (!present_) return;
  uint8_t v = 0x10;
  wr(REG_ALL_LED_OFF_H, &v, 1);
}
