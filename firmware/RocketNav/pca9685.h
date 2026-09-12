// pca9685 - minimal owned driver for the PCA9685BS 16-channel 12-bit I2C
// PWM controller that drives the canard servos. Lives on the SHARED sensor
// bus, so every transaction is short and bounded; the chip free-runs, so a
// wedged bus leaves servos holding their last pulse width (no flail).
// The chip has no WHO_AM_I: presence = an ACKed MODE1 read.
#pragma once

#include <Arduino.h>
#include <Wire.h>

class Pca9685 {
 public:
  // Probes the chip and programs the PWM frame rate. false = absent.
  bool begin(TwoWire *wire, uint8_t addr, float frame_hz);

  // One channel, pulse width in microseconds (0 = full off for the channel).
  // ~6-byte transaction, ~150 us on a 400 kHz bus.
  bool setPwmUs(uint8_t ch, float us);

  void allOff();  // ALL_LED full-off: pulses stop, servos go limp

  // Program the shared PWM frame rate (24..1526 Hz; begin() routes
  // through this). ALL 16 channels share the one frame.
  bool setFrameHz(float frame_hz);

  bool present() const { return present_; }
  uint32_t errors() const { return errors_; }
  // Achieved PWM frame rate (prescale-quantized), the basis of the us->count
  // conversion in setPwmUs.
  float frameHz() const { return frame_hz_; }

 private:
  bool wr(uint8_t reg, const uint8_t *p, int n);
  bool wake();  // clear SLEEP + RESTART sequence (resumes PWM output)

  TwoWire *wire_ = nullptr;
  uint8_t addr_ = 0x40;
  float frame_hz_ = 50.0f;
  bool present_ = false;
  uint32_t errors_ = 0;
};
