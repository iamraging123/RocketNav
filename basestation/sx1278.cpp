#include "sx1278.h"

// SX1276/7/8 register map (LoRa page)
#define R_FIFO 0x00
#define R_OPMODE 0x01
#define R_FRF_MSB 0x06
#define R_PA_CONFIG 0x09
#define R_LNA 0x0C
#define R_FIFO_ADDR_PTR 0x0D
#define R_FIFO_TX_BASE 0x0E
#define R_FIFO_RX_BASE 0x0F
#define R_FIFO_RX_CURRENT 0x10
#define R_IRQ_FLAGS 0x12
#define R_RX_NB_BYTES 0x13
#define R_PKT_SNR 0x19
#define R_PKT_RSSI 0x1A
#define R_MODEM_CFG1 0x1D
#define R_MODEM_CFG2 0x1E
#define R_PREAMBLE_MSB 0x20
#define R_PAYLOAD_LEN 0x22
#define R_MODEM_CFG3 0x26
#define R_DIO_MAPPING1 0x40
#define R_VERSION 0x42
#define R_PA_DAC 0x4D

#define M_SLEEP 0x00
#define M_STDBY 0x01
#define M_TX 0x03
#define M_RXCONT 0x05
#define LONG_RANGE 0x80

#define IRQ_TXDONE 0x08
#define IRQ_RXDONE 0x40
#define IRQ_CRCERR 0x20

static const SPISettings kSpi(8000000, MSBFIRST, SPI_MODE0);

uint8_t Sx1278::rd(uint8_t reg) {
  spi_->beginTransaction(kSpi);
  digitalWrite(cs_, LOW);
  spi_->transfer(reg & 0x7F);
  uint8_t v = spi_->transfer(0);
  digitalWrite(cs_, HIGH);
  spi_->endTransaction();
  return v;
}

void Sx1278::wr(uint8_t reg, uint8_t v) {
  spi_->beginTransaction(kSpi);
  digitalWrite(cs_, LOW);
  spi_->transfer(reg | 0x80);
  spi_->transfer(v);
  digitalWrite(cs_, HIGH);
  spi_->endTransaction();
}

void Sx1278::rdBurst(uint8_t reg, uint8_t *p, int n) {
  spi_->beginTransaction(kSpi);
  digitalWrite(cs_, LOW);
  spi_->transfer(reg & 0x7F);
  for (int i = 0; i < n; ++i) p[i] = spi_->transfer(0);
  digitalWrite(cs_, HIGH);
  spi_->endTransaction();
}

void Sx1278::wrBurst(uint8_t reg, const uint8_t *p, int n) {
  spi_->beginTransaction(kSpi);
  digitalWrite(cs_, LOW);
  spi_->transfer(reg | 0x80);
  for (int i = 0; i < n; ++i) spi_->transfer(p[i]);
  digitalWrite(cs_, HIGH);
  spi_->endTransaction();
}

void Sx1278::setMode(uint8_t m) { wr(R_OPMODE, LONG_RANGE | m); }

bool Sx1278::begin(SPIClass *spi, int8_t cs_pin, int8_t rst_pin,
                   int8_t dio0_pin) {
  spi_ = spi;
  cs_ = cs_pin;
  rst_ = rst_pin;
  dio0_ = dio0_pin;
  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);
  pinMode(dio0_, INPUT);
  pinMode(rst_, OUTPUT);
  digitalWrite(rst_, LOW);
  delay(2);
  digitalWrite(rst_, HIGH);
  delay(6);
  present_ = (rd(R_VERSION) == 0x12);
  tx_busy_ = false;
  return present_;
}

void Sx1278::configure(uint32_t freq_hz, uint8_t sf, uint32_t bw_hz,
                       uint8_t cr_denom, uint8_t sync, uint16_t preamble,
                       int8_t tx_dbm) {
  if (!present_) return;
  wr(R_OPMODE, 0x00);        // FSK sleep first: LongRangeMode writable
  delay(1);
  wr(R_OPMODE, LONG_RANGE | M_SLEEP);
  delay(1);

  // Frf = f / (Fxosc / 2^19), Fxosc = 32 MHz
  uint64_t frf = ((uint64_t)freq_hz << 19) / 32000000ull;
  wr(R_FRF_MSB, (uint8_t)(frf >> 16));
  wr(R_FRF_MSB + 1, (uint8_t)(frf >> 8));
  wr(R_FRF_MSB + 2, (uint8_t)(frf >> 0));

  uint8_t bw_code = 7;  // 125k
  if (bw_hz >= 500000ul) bw_code = 9;
  else if (bw_hz >= 250000ul) bw_code = 8;
  uint8_t cr_code = (uint8_t)(cr_denom - 4);  // 4/5 -> 1
  wr(R_MODEM_CFG1, (uint8_t)((bw_code << 4) | (cr_code << 1)));  // explicit hdr
  wr(R_MODEM_CFG2, (uint8_t)((sf << 4) | 0x04));  // CRC on
  // AGC on; LowDataRateOptimize only needed when Tsym > 16 ms (SF11+/125k) -
  // far from this profile.
  wr(R_MODEM_CFG3, 0x04);
  wr(R_PREAMBLE_MSB, (uint8_t)(preamble >> 8));
  wr(R_PREAMBLE_MSB + 1, (uint8_t)(preamble & 0xFF));
  wr(0x39, sync);  // RegSyncWord

  // Ra-02 routes only PA_BOOST. 2..17 dBm; PA_DAC stays in default (no
  // +20 dBm mode - that needs duty/SWR care this airframe doesn't control).
  int8_t p = tx_dbm;
  if (p < 2) p = 2;
  if (p > 17) p = 17;
  wr(R_PA_CONFIG, (uint8_t)(0x80 | (uint8_t)(p - 2)));
  wr(R_PA_DAC, 0x84);
  wr(R_LNA, (uint8_t)(rd(R_LNA) | 0x03));  // LNA boost (HF reg, harmless)

  wr(R_FIFO_TX_BASE, 0x00);
  wr(R_FIFO_RX_BASE, 0x00);
  startRx();
}

void Sx1278::startRx() {
  if (!present_) return;
  wr(R_IRQ_FLAGS, 0xFF);
  wr(R_DIO_MAPPING1, 0x00);  // DIO0 = RxDone
  setMode(M_RXCONT);
  tx_busy_ = false;
}

bool Sx1278::txStart(const uint8_t *buf, uint8_t len) {
  if (!present_ || tx_busy_) return false;
  setMode(M_STDBY);
  wr(R_IRQ_FLAGS, 0xFF);
  wr(R_FIFO_ADDR_PTR, 0x00);
  wrBurst(R_FIFO, buf, len);
  wr(R_PAYLOAD_LEN, len);
  wr(R_DIO_MAPPING1, 0x40);  // DIO0 = TxDone
  setMode(M_TX);
  tx_busy_ = true;
  return true;
}

// Flag-register read cadence when DIO0 says nothing is pending. A loose or
// floating DIO0 (base station bench, 2026-09-20: TX worked, RX counted
// nothing for minutes) would otherwise silence the receive path completely;
// with this, it costs at most kFlagPollMs of latency. One 2-byte SPI read
// per period is nothing on either end.
static const uint32_t kFlagPollMs = 20;

uint8_t Sx1278::poll() {
  if (!present_) return 0;
  // Zero-SPI fast path: nothing pending unless DIO0 is up, a TX is in
  // flight, or the timed fallback read is due.
  uint32_t now = millis();
  if (!tx_busy_ && dio0_ >= 0 && digitalRead(dio0_) == LOW &&
      (uint32_t)(now - last_flags_ms_) < kFlagPollMs) return 0;
  last_flags_ms_ = now;
  uint8_t f = rd(R_IRQ_FLAGS);
  uint8_t ev = 0;
  if (tx_busy_ && (f & IRQ_TXDONE)) {
    wr(R_IRQ_FLAGS, IRQ_TXDONE);
    startRx();  // hand the channel straight back to the uplink
    ev |= EV_TXDONE;
  }
  if (f & IRQ_RXDONE) {
    if (f & IRQ_CRCERR) {
      wr(R_IRQ_FLAGS, IRQ_RXDONE | IRQ_CRCERR);
      ev |= EV_CRCERR;
    } else {
      int8_t snr4 = (int8_t)rd(R_PKT_SNR);
      snr_db_ = snr4 * 0.25f;
      // LF port (433 MHz): RSSI = -164 + value, SNR-corrected when negative
      int16_t r = (int16_t)rd(R_PKT_RSSI) - 164;
      if (snr_db_ < 0) r += (int16_t)snr_db_;
      rssi_dbm_ = r;
      ev |= EV_RXDONE;  // flags cleared in rxRead
    }
  }
  return ev;
}

int Sx1278::rxRead(uint8_t *buf, int cap) {
  if (!present_) return 0;
  int n = rd(R_RX_NB_BYTES);
  if (n <= 0 || n > cap) {
    wr(R_IRQ_FLAGS, IRQ_RXDONE);
    return 0;
  }
  wr(R_FIFO_ADDR_PTR, rd(R_FIFO_RX_CURRENT));
  rdBurst(R_FIFO, buf, n);
  wr(R_IRQ_FLAGS, IRQ_RXDONE);
  return n;
}

void Sx1278::sleep() {
  if (present_) setMode(M_SLEEP);
}
