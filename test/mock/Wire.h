// Host-test mock of TwoWire: records every I2C transaction byte-for-byte
// so tests can assert exactly what would hit the PCA9685's pins.
#pragma once

#include <stdint.h>

#include <vector>

class TwoWire {
 public:
  struct Txn {
    uint8_t addr;
    std::vector<uint8_t> bytes;  // register byte first, then data
  };

  void beginTransmission(uint8_t a) {
    cur_.addr = a;
    cur_.bytes.clear();
  }
  size_t write(uint8_t b) {
    cur_.bytes.push_back(b);
    return 1;
  }
  uint8_t endTransmission(bool stop = true) {
    (void)stop;
    if (!ack) return 4;
    txns.push_back(cur_);
    return 0;
  }
  int requestFrom(int, int n) { return ack ? n : 0; }
  int read() { return 0x20; }  // MODE1 readback for the presence probe

  std::vector<Txn> txns;
  bool ack = true;

 private:
  Txn cur_;
};
