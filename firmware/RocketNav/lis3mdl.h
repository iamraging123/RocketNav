// lis3mdl.h — minimal from-scratch I2C driver for the ST LIS3MDL 3-axis
// magnetometer, written directly against Wire. No vendor library is linked.
//
// Register addresses and encodings are from the LIS3MDL datasheet
// (DocID023312). The one protocol fact that motivated replacing the shelf
// driver: on I2C, reading MULTIPLE bytes requires SUB(7) = 1 in the
// sub-address byte ("to read multiple bytes it is necessary to assert the
// most significant bit of the subaddress field") — without it the device
// serves the same register for every clocked byte. The previous library
// stack only applied that bit on its SPI path. Every multi-byte read here
// ORs in kAutoInc explicitly, and begin() PROVES the mechanism works on the
// soldered part: the config readback is a 5-byte auto-increment burst that
// can only match the 5 distinct values just written if the address pointer
// actually advanced.
//
// Design notes for this board:
//  * No DRDY pin is wired: we poll. Each poll is ONE 7-byte burst from
//    STATUS through OUT_Z_H, so the ZYXDA freshness flag and the sample it
//    describes are captured in the same bus transaction.
//  * BDU is set (CTRL_REG5): output registers hold until both bytes of an
//    axis are read, so an 80 Hz update landing mid-burst can never tear the
//    low/high bytes of one axis apart.
//  * diag() is a static register-level probe used by the $magdiag ground
//    command. It deliberately bypasses this driver's read path (and works
//    with the device unconfigured) so its output stays trustworthy even
//    when the suspect IS the driver or the init sequence.

#pragma once
#include <Arduino.h>
#include <Wire.h>

// Raw register-level snapshot for the $magdiag ground command. Filled by
// Lis3mdl::diag(); formatted into msg records by the .ino.
struct MagDiag {
  bool who_ok = false;       // WHO_AM_I transaction completed
  uint8_t who = 0;           // WHO_AM_I value (expect 0x3D)
  bool ctrl_ok = false;      // CTRL_REG1..5 single-byte reads completed
  uint8_t ctrl[5] = {0, 0, 0, 0, 0};
  uint8_t status = 0;        // STATUS register at entry
  int16_t noinc[3] = {0, 0, 0};   // 6-byte burst at OUT_X_L, SUB(7)=0 (the
                                  // old library's exact transaction shape)
  int16_t inc[3] = {0, 0, 0};     // 6-byte burst at OUT_X_L | 0x80
  int16_t single[3] = {0, 0, 0};  // six 1-byte reads, 0x28..0x2D
  uint8_t zyxda_n = 0;       // of kDiagPolls status reads ~5 ms apart, how
                             // many had ZYXDA set
  uint8_t change_n = 0;      // of kDiagPolls auto-inc bursts, how many
                             // differed from the previous one
  uint8_t bus_err = 0;       // failed transactions during the probe
  static const uint8_t kDiagPolls = 12;
};

class Lis3mdl {
 public:
  struct RawSample {
    int16_t mx, my, mz;  // field counts (0.14620 uT/LSB at +/-4 gauss)
    bool drdy;           // STATUS.ZYXDA captured in the same burst
  };

  // Full init: probe, soft reset, UHP / 80 Hz / +/-4 gauss / BDU /
  // continuous, then verify the whole config with an auto-increment burst
  // readback. Blocking (setup/reinit path only). Returns false on any bus
  // failure or readback mismatch — a mag that cannot prove its config is
  // reported absent rather than trusted.
  bool begin(TwoWire &wire, uint8_t addr);

  // Non-blocking-safe poll: one 7-byte burst (STATUS + XYZ), no delays.
  // Returns false on a bus error; out.drdy tells staleness.
  bool readSample(RawSample &out);

  bool probe();  // WHO_AM_I check only
  uint32_t busErrors() const { return bus_errors_; }
  static float utPerLsb() { return 100.0f / 6842.0f; }  // +/-4 gauss, DS Tbl 3

  // Register-level ground diagnostic (see MagDiag). Static: runs on a bare
  // bus+address, independent of any driver instance or its init state.
  // Blocks ~60 ms (kDiagPolls status/data polls at ~5 ms spacing).
  static void diag(TwoWire &wire, uint8_t addr, MagDiag &out);

 private:
  bool writeReg(uint8_t reg, uint8_t val);
  bool readRegs(uint8_t reg, uint8_t *buf, uint8_t n);  // ORs kAutoInc if n>1
  static bool rawRead(TwoWire &wire, uint8_t addr, uint8_t sub, uint8_t *buf,
                      uint8_t n);  // sends sub EXACTLY as given (diag only)

  TwoWire *wire_ = nullptr;
  uint8_t addr_ = 0x1C;
  uint32_t bus_errors_ = 0;

  // Register map (DocID023312).
  static constexpr uint8_t REG_WHO_AM_I = 0x0F;   // = 0x3D
  static constexpr uint8_t REG_CTRL1 = 0x20;
  static constexpr uint8_t REG_CTRL2 = 0x21;
  static constexpr uint8_t REG_CTRL3 = 0x22;
  static constexpr uint8_t REG_CTRL4 = 0x23;
  static constexpr uint8_t REG_CTRL5 = 0x24;
  static constexpr uint8_t REG_STATUS = 0x27;
  static constexpr uint8_t REG_OUT_X_L = 0x28;
  static constexpr uint8_t WHO_AM_I_VALUE = 0x3D;
  static constexpr uint8_t kAutoInc = 0x80;       // SUB(7): address auto-inc
  static constexpr uint8_t kZyxda = 0x08;         // STATUS bit 3

  // Configuration written by begin() and verified by burst readback:
  //  CTRL1 0x7C: TEMP_EN=0, OM=UHP(11), DO=80 Hz(111), FAST_ODR=0, ST=0
  //  CTRL2 0x00: FS=+/-4 gauss
  //  CTRL3 0x00: continuous-conversion mode
  //  CTRL4 0x0C: OMZ=UHP(11), BLE=0 (little-endian data)
  //  CTRL5 0x40: BDU=1 (no torn axis reads), FAST_READ=0
  static constexpr uint8_t kCfg[5] = { 0x7C, 0x00, 0x00, 0x0C, 0x40 };
};
