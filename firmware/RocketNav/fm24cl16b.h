// fm24cl16b - owned driver for the Cypress/Infineon FM24CL16B 16 kbit I2C
// FRAM (2048 bytes, 8 pages of 256). The page index rides in the device
// address (1010 A10 A9 A8), the byte address in the first data byte; the
// part writes at bus speed with no erase and no write delay, and its
// endurance (1e14 cycles) makes it the only store on this board that can be
// written continuously in flight. WP is tied low on the PCB.
//
// Every call is one or more short bounded Wire transactions (<= 28 data
// bytes: the Arduino Wire buffer is 32 including the address byte, and a
// transaction never crosses a page). Nothing here retries; the caller
// counts errors and backs off.
#pragma once

#include <Arduino.h>
#include <Wire.h>

class Fm24cl16b {
 public:
  static const uint16_t kSize = 2048;
  static const uint8_t kChunk = 28;

  // Probes by reading byte 0. false = no ACK (chip absent).
  bool begin(TwoWire *w, uint8_t base_addr = 0x50);

  bool read(uint16_t addr, uint8_t *buf, uint16_t n);
  bool write(uint16_t addr, const uint8_t *buf, uint16_t n);

  bool present() const { return present_; }
  uint32_t errors() const { return errors_; }

 private:
  TwoWire *w_ = nullptr;
  uint8_t base_ = 0x50;
  bool present_ = false;
  uint32_t errors_ = 0;
};
