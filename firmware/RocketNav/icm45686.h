// icm45686.h — minimal from-scratch I2C driver for the TDK ICM-45686 6-axis
// IMU, written directly against Wire. No vendor library is linked.
//
// Register addresses and field encodings below are from the ICM-45686
// datasheet DS-000489 (register map, sections 3.1/3.2 for full-scale and ODR
// encodings). Access model: direct registers 0x00..0x7F over I2C; the UI
// low-pass filter selects live in indirect register space (IPREG_SYS1/2)
// reached through IREG_ADDR_15_8 / IREG_ADDR_7_0 / IREG_DATA with ~4 us
// settling between operations (DS "indirect register access").
//
// Design notes for this board:
//  * INT/DRDY pin is NOT wired: we poll. The data-ready flag in INT1_STATUS0
//    only latches if DRDY is enabled as an INT1 source (INT1_CONFIG0 bit2),
//    so begin() enables it even though the pin goes nowhere.
//  * Each poll reads INT1_STATUS0 (1 byte, clear-on-read) and, only when
//    DRDY is set, the 14-byte data block 0x00..0x0D. A single sweep from
//    0x00 through 0x19 would pass THROUGH FIFO_COUNT/FIFO_DATA (0x12-0x14);
//    reading FIFO_DATA outside FIFO operation is undefined on this part and
//    can stall the serial interface, so the data window stops at TEMP_DATA0.
//    Freshness stays race-free: DRDY latches per sample and the data
//    registers are shadowed against mid-burst update, so drdy==1 guarantees
//    the block that follows is one complete, fresh sample.
//  * SREG_CTRL is explicitly programmed to little-endian data so the byte
//    order is deterministic rather than reset-default-dependent.

#pragma once
#include <Arduino.h>
#include <Wire.h>

class Icm45686 {
 public:
  // Full-scale encodings, DS-000489 Tables 1 & 2 (FS_SEL values).
  enum AccelFs : uint8_t { AFS_32G = 0x0, AFS_16G = 0x1, AFS_8G = 0x2,
                           AFS_4G = 0x3, AFS_2G = 0x4 };
  enum GyroFs : uint8_t { GFS_4000DPS = 0x0, GFS_2000DPS = 0x1,
                          GFS_1000DPS = 0x2, GFS_500DPS = 0x3,
                          GFS_250DPS = 0x4 };
  // ODR encodings (same ladder both sensors); 1000 Hz is NOT a valid step.
  enum Odr : uint8_t { ODR_6400HZ = 0x3, ODR_3200HZ = 0x4, ODR_1600HZ = 0x5,
                       ODR_800HZ = 0x6, ODR_400HZ = 0x7, ODR_200HZ = 0x8,
                       ODR_100HZ = 0x9 };
  // UI LPF bandwidth select (GYRO_UI_LPFBW_SEL / ACCEL_UI_LPFBW_SEL).
  enum LpfBw : uint8_t { LPF_BYPASS = 0x0, LPF_ODR_DIV_4 = 0x1,
                         LPF_ODR_DIV_8 = 0x2, LPF_ODR_DIV_16 = 0x3,
                         LPF_ODR_DIV_32 = 0x4, LPF_ODR_DIV_64 = 0x5,
                         LPF_ODR_DIV_128 = 0x6 };

  struct RawSample {
    int16_t ax, ay, az;   // accel counts
    int16_t gx, gy, gz;   // gyro counts
    int16_t temp;         // temp counts (degC = counts/132.48 + 25)
    bool drdy;            // INT1_STATUS0.drdy captured in the same burst
  };

  // Full init: probe, soft reset, endianness, DRDY source, FS/ODR/LPF,
  // low-noise power-up. Blocking (setup/reinit path only). Returns false and
  // leaves the device untouched-as-possible on any bus failure.
  bool begin(TwoWire &wire, uint8_t addr, AccelFs afs, GyroFs gfs, Odr odr,
             LpfBw accel_lpf, LpfBw gyro_lpf);

  // Non-blocking-safe poll (single burst transaction, no delays). Returns
  // false on a bus error; out.drdy tells staleness.
  bool readSample(RawSample &out);

  bool probe();  // WHO_AM_I check only
  uint32_t busErrors() const { return bus_errors_; }
  float accelScale() const { return accel_scale_; }  // m/s^2 per count
  float gyroScale() const { return gyro_scale_; }    // rad/s per count

 private:
  bool writeReg(uint8_t reg, uint8_t val);
  bool readRegs(uint8_t reg, uint8_t *buf, uint8_t n);
  bool iregRead(uint16_t addr, uint8_t *val);
  bool iregWrite(uint16_t addr, uint8_t val);

  TwoWire *wire_ = nullptr;
  uint8_t addr_ = 0x68;
  uint32_t bus_errors_ = 0;
  float accel_scale_ = 0;
  float gyro_scale_ = 0;

  // Register map (DS-000489).
  static constexpr uint8_t REG_ACCEL_DATA_X1 = 0x00;  // 12 data bytes start
  static constexpr uint8_t REG_TEMP_DATA1 = 0x0C;
  static constexpr uint8_t REG_PWR_MGMT0 = 0x10;
  static constexpr uint8_t REG_INT1_CONFIG0 = 0x16;
  static constexpr uint8_t REG_INT1_STATUS0 = 0x19;
  static constexpr uint8_t REG_ACCEL_CONFIG0 = 0x1B;
  static constexpr uint8_t REG_GYRO_CONFIG0 = 0x1C;
  static constexpr uint8_t REG_WHO_AM_I = 0x72;
  static constexpr uint8_t REG_IREG_ADDR_15_8 = 0x7C;
  static constexpr uint8_t REG_IREG_DATA = 0x7E;
  static constexpr uint8_t REG_MISC2 = 0x7F;
  static constexpr uint8_t WHO_AM_I_VALUE = 0xE9;
  static constexpr uint16_t IREG_SREG_CTRL = 0xA267;       // bit0: endianness
  static constexpr uint16_t IREG_SYS1_REG_172 = 0xA4AC;    // gyro UI LPF [2:0]
  static constexpr uint16_t IREG_SYS2_REG_131 = 0xA583;    // accel UI LPF [2:0]
  static constexpr uint8_t DATA_LEN = 14;  // 0x00..0x0D: accel, gyro, temp
};
