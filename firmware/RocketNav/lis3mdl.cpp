// lis3mdl.cpp — see lis3mdl.h for the protocol facts (SUB(7) auto-increment,
// BDU) that shape every transaction here.

#include "lis3mdl.h"

constexpr uint8_t Lis3mdl::kCfg[5];

bool Lis3mdl::writeReg(uint8_t reg, uint8_t val) {
  wire_->beginTransmission(addr_);
  wire_->write(reg);
  wire_->write(val);
  if (wire_->endTransmission() != 0) {
    bus_errors_++;
    return false;
  }
  return true;
}

bool Lis3mdl::readRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  wire_->beginTransmission(addr_);
  wire_->write(n > 1 ? (uint8_t)(reg | kAutoInc) : reg);
  if (wire_->endTransmission(false) != 0) {  // repeated start
    bus_errors_++;
    return false;
  }
  if (wire_->requestFrom(addr_, n) != n) {
    bus_errors_++;
    return false;
  }
  for (uint8_t i = 0; i < n; ++i) buf[i] = (uint8_t)wire_->read();
  return true;
}

bool Lis3mdl::probe() {
  uint8_t v = 0;
  if (!readRegs(REG_WHO_AM_I, &v, 1)) return false;
  return v == WHO_AM_I_VALUE;
}

bool Lis3mdl::begin(TwoWire &wire, uint8_t addr) {
  wire_ = &wire;
  addr_ = addr;

  bool found = false;
  for (int i = 0; i < 2 && !found; ++i) found = probe();
  if (!found) return false;

  // SOFT_RST (CTRL2 bit 2): user registers to defaults, then settle.
  if (!writeReg(REG_CTRL2, 0x04)) return false;
  delay(10);

  for (uint8_t i = 0; i < 5; ++i) {
    if (!writeReg((uint8_t)(REG_CTRL1 + i), kCfg[i])) return false;
  }

  // Auto-increment burst readback: five DISTINCT values can only come back
  // right if the address pointer advanced, so this verifies the configuration
  // AND the multi-byte read mechanism the poll path depends on.
  uint8_t back[5] = { 0, 0, 0, 0, 0 };
  if (!readRegs(REG_CTRL1, back, 5)) return false;
  for (uint8_t i = 0; i < 5; ++i) {
    if (back[i] != kCfg[i]) return false;
  }
  return true;
}

bool Lis3mdl::readSample(RawSample &out) {
  // STATUS through OUT_Z_H in one transaction: freshness flag and data are
  // a single atomic snapshot (BDU guards the axis pairs on the sensor side).
  uint8_t b[7];
  if (!readRegs(REG_STATUS, b, 7)) return false;
  out.drdy = (b[0] & kZyxda) != 0;
  out.mx = (int16_t)((uint16_t)b[1] | ((uint16_t)b[2] << 8));
  out.my = (int16_t)((uint16_t)b[3] | ((uint16_t)b[4] << 8));
  out.mz = (int16_t)((uint16_t)b[5] | ((uint16_t)b[6] << 8));
  return true;
}

bool Lis3mdl::rawRead(TwoWire &wire, uint8_t addr, uint8_t sub, uint8_t *buf,
                      uint8_t n) {
  wire.beginTransmission(addr);
  wire.write(sub);
  if (wire.endTransmission(false) != 0) return false;
  if (wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; ++i) buf[i] = (uint8_t)wire.read();
  return true;
}

void Lis3mdl::diag(TwoWire &wire, uint8_t addr, MagDiag &out) {
  out = MagDiag();
  uint8_t b[6];

  out.who_ok = rawRead(wire, addr, REG_WHO_AM_I, &out.who, 1);
  if (!out.who_ok) out.bus_err++;

  out.ctrl_ok = true;
  for (uint8_t i = 0; i < 5; ++i) {
    if (!rawRead(wire, addr, (uint8_t)(REG_CTRL1 + i), &out.ctrl[i], 1)) {
      out.ctrl_ok = false;
      out.bus_err++;
    }
  }
  if (!rawRead(wire, addr, REG_STATUS, &out.status, 1)) out.bus_err++;

  // The three read strategies over the same six output registers. Their
  // agreement pattern localizes a fault: single-byte reads are the ground
  // truth (no increment needed), the inc burst is this driver's mechanism,
  // the noinc burst is the transaction shape the old library used on I2C.
  if (rawRead(wire, addr, REG_OUT_X_L, b, 6)) {
    for (int i = 0; i < 3; ++i) {
      out.noinc[i] =
          (int16_t)((uint16_t)b[2 * i] | ((uint16_t)b[2 * i + 1] << 8));
    }
  } else {
    out.bus_err++;
  }
  if (rawRead(wire, addr, (uint8_t)(REG_OUT_X_L | kAutoInc), b, 6)) {
    for (int i = 0; i < 3; ++i) {
      out.inc[i] =
          (int16_t)((uint16_t)b[2 * i] | ((uint16_t)b[2 * i + 1] << 8));
    }
  } else {
    out.bus_err++;
  }
  {
    uint8_t s[6];
    bool ok = true;
    for (uint8_t i = 0; i < 6; ++i) {
      if (!rawRead(wire, addr, (uint8_t)(REG_OUT_X_L + i), &s[i], 1)) {
        ok = false;
        out.bus_err++;
        break;
      }
    }
    if (ok) {
      for (int i = 0; i < 3; ++i) {
        out.single[i] =
            (int16_t)((uint16_t)s[2 * i] | ((uint16_t)s[2 * i + 1] << 8));
      }
    }
  }

  // Liveness: at the configured 80 Hz ODR, ~5 ms spacing should see ZYXDA
  // set on roughly a third of polls and the data change most reads. A dead
  // ODR shows zyxda_n == 0; frozen silicon shows change_n == 0 with ZYXDA
  // happily set. Blocking is fine: this runs only on an explicit ground
  // command, like the flash-save stall.
  int16_t prev[3] = { 0, 0, 0 };
  bool have_prev = false;
  for (uint8_t k = 0; k < MagDiag::kDiagPolls; ++k) {
    uint8_t st = 0;
    if (rawRead(wire, addr, REG_STATUS, &st, 1) && (st & kZyxda)) {
      out.zyxda_n++;
    }
    if (rawRead(wire, addr, (uint8_t)(REG_OUT_X_L | kAutoInc), b, 6)) {
      int16_t v[3];
      for (int i = 0; i < 3; ++i) {
        v[i] = (int16_t)((uint16_t)b[2 * i] | ((uint16_t)b[2 * i + 1] << 8));
      }
      if (have_prev &&
          (v[0] != prev[0] || v[1] != prev[1] || v[2] != prev[2])) {
        out.change_n++;
      }
      for (int i = 0; i < 3; ++i) prev[i] = v[i];
      have_prev = true;
    } else {
      out.bus_err++;
    }
    delay(5);
  }
}
