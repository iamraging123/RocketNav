// icm45686.cpp — see icm45686.h for the register-map provenance and the
// polling/DRDY design. begin() may delay (setup/reinit path); readSample()
// never delays or blocks beyond one bounded I2C transaction.

#include "icm45686.h"

bool Icm45686::writeReg(uint8_t reg, uint8_t val) {
  wire_->beginTransmission(addr_);
  wire_->write(reg);
  wire_->write(val);
  if (wire_->endTransmission() != 0) { bus_errors_++; return false; }
  return true;
}

bool Icm45686::readRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  wire_->beginTransmission(addr_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) {  // repeated start, keep the bus
    bus_errors_++;
    return false;
  }
  uint8_t got = wire_->requestFrom(addr_, n);
  if (got != n) { bus_errors_++; return false; }
  for (uint8_t i = 0; i < n; ++i) buf[i] = wire_->read();
  return true;
}

// Indirect register access per DS-000489: write the 16-bit address to
// IREG_ADDR_15_8/7_0 (auto-increment), data through IREG_DATA, with ~4 us
// settling between bus operations.
bool Icm45686::iregRead(uint16_t addr, uint8_t *val) {
  wire_->beginTransmission(addr_);
  wire_->write(REG_IREG_ADDR_15_8);
  wire_->write((uint8_t)(addr >> 8));
  wire_->write((uint8_t)(addr & 0xFF));
  if (wire_->endTransmission() != 0) { bus_errors_++; return false; }
  delayMicroseconds(4);
  return readRegs(REG_IREG_DATA, val, 1);
}

bool Icm45686::iregWrite(uint16_t addr, uint8_t val) {
  wire_->beginTransmission(addr_);
  wire_->write(REG_IREG_ADDR_15_8);
  wire_->write((uint8_t)(addr >> 8));
  wire_->write((uint8_t)(addr & 0xFF));
  wire_->write(val);  // third byte after the address pair is the first datum
  if (wire_->endTransmission() != 0) { bus_errors_++; return false; }
  delayMicroseconds(4);
  return true;
}

bool Icm45686::probe() {
  uint8_t id = 0;
  if (!readRegs(REG_WHO_AM_I, &id, 1)) return false;
  return id == WHO_AM_I_VALUE;
}

bool Icm45686::begin(TwoWire &wire, uint8_t addr, AccelFs afs, GyroFs gfs,
                     Odr odr, LpfBw accel_lpf, LpfBw gyro_lpf) {
  wire_ = &wire;
  addr_ = addr;

  bool found = false;
  for (int i = 0; i < 3 && !found; ++i) {
    found = probe();
    if (!found) delay(10);
  }
  if (!found) return false;

  // Soft reset (REG_MISC2 bit1), then allow the register file to reload.
  // DS-000489 quotes ~1 ms; 10 ms is comfortable and setup-time only.
  if (!writeReg(REG_MISC2, 0x02)) return false;
  delay(10);
  if (!probe()) return false;

  // Deterministic byte order: SREG_CTRL bit0 = 0 -> little-endian data regs.
  uint8_t sreg = 0;
  if (!iregRead(IREG_SREG_CTRL, &sreg)) return false;
  if (!iregWrite(IREG_SREG_CTRL, (uint8_t)(sreg & ~0x01))) return false;

  // Enable DRDY as an INT1 source so INT1_STATUS0.drdy latches for polling
  // (the physical pin is unconnected on this board).
  if (!writeReg(REG_INT1_CONFIG0, 0x04)) return false;

  // Full scale + ODR: CONFIG0 layout is [6:4]/[7:4] FS_SEL, [3:0] ODR.
  if (!writeReg(REG_ACCEL_CONFIG0, (uint8_t)((afs << 4) | odr))) return false;
  if (!writeReg(REG_GYRO_CONFIG0, (uint8_t)((gfs << 4) | odr))) return false;

  // UI low-pass filters (indirect space), read-modify-write bits [2:0].
  uint8_t r = 0;
  if (!iregRead(IREG_SYS1_REG_172, &r)) return false;
  if (!iregWrite(IREG_SYS1_REG_172, (uint8_t)((r & ~0x07) | gyro_lpf)))
    return false;
  if (!iregRead(IREG_SYS2_REG_131, &r)) return false;
  if (!iregWrite(IREG_SYS2_REG_131, (uint8_t)((r & ~0x07) | accel_lpf)))
    return false;

  // Both sensors to low-noise mode; gyro start-up is 35 ms, accel 10 ms
  // (DS-000489 Tables 1/2) — wait it out here so the first polled samples
  // are live, then clear any latched status bits.
  if (!writeReg(REG_PWR_MGMT0, 0x0F)) return false;
  delay(40);
  uint8_t status = 0;
  readRegs(REG_INT1_STATUS0, &status, 1);

  // Cache scale factors: counts -> SI.
  static const float fs_g[] = { 32.0f, 16.0f, 8.0f, 4.0f, 2.0f };
  static const float fs_dps[] = { 4000.0f, 2000.0f, 1000.0f, 500.0f, 250.0f };
  accel_scale_ = fs_g[afs] * 9.80665f / 32768.0f;
  gyro_scale_ = fs_dps[gfs] * 0.0174532925199433f / 32768.0f;
  return true;
}

bool Icm45686::readSample(RawSample &out) {
  // Status first (1 byte, clear-on-read), then the data block only when a
  // fresh sample is latched. Never sweeps into FIFO_COUNT/FIFO_DATA — see
  // the header note: reading FIFO_DATA outside FIFO operation is undefined
  // and can stall the serial interface. A stale poll now costs one byte on
  // the bus instead of 26.
  uint8_t status = 0;
  if (!readRegs(REG_INT1_STATUS0, &status, 1)) return false;
  out.drdy = (status & 0x04) != 0;
  if (!out.drdy) return true;
  uint8_t b[DATA_LEN];
  if (!readRegs(REG_ACCEL_DATA_X1, b, DATA_LEN)) return false;
  // Little-endian (SREG_CTRL programmed in begin()): low byte first.
  out.ax = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
  out.ay = (int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8));
  out.az = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
  out.gx = (int16_t)((uint16_t)b[6] | ((uint16_t)b[7] << 8));
  out.gy = (int16_t)((uint16_t)b[8] | ((uint16_t)b[9] << 8));
  out.gz = (int16_t)((uint16_t)b[10] | ((uint16_t)b[11] << 8));
  out.temp = (int16_t)((uint16_t)b[12] | ((uint16_t)b[13] << 8));
  return true;
}
