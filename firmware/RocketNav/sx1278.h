// sx1278 - minimal owned LoRa driver for the Ai-Thinker Ra-02 (SX1278).
// Plain Arduino SPI + GPIO, so the identical file compiles on the rocket
// (stm32duino) and the base station (ESP32 core).
//
// Design points for THIS hardware: only DIO0 is wired (DIO1-5 NC), so all
// state comes from DIO0 plus the IRQ-flags register; every call is a short
// bounded SPI burst and nothing blocks - TX completion is discovered by
// later poll() calls, which fits the rocket's cooperative scheduler.
// The Ra-02 TX path is PA_BOOST only (2..17 dBm).
#pragma once

#include <Arduino.h>
#include <SPI.h>

class Sx1278 {
 public:
  // Pulses RESET, probes RegVersion (0x12). false = chip absent.
  bool begin(SPIClass *spi, int8_t cs_pin, int8_t rst_pin, int8_t dio0_pin);

  // Full LoRa mode setup; leaves the radio in RX-continuous.
  void configure(uint32_t freq_hz, uint8_t sf, uint32_t bw_hz,
                 uint8_t cr_denom, uint8_t sync, uint16_t preamble,
                 int8_t tx_dbm);

  void startRx();  // RX-continuous, DIO0 = RxDone

  // Loads the FIFO and starts transmitting. false when a TX is already in
  // flight or the chip is absent. Completion arrives via poll().
  bool txStart(const uint8_t *buf, uint8_t len);

  // Event bits returned by poll():
  static const uint8_t EV_TXDONE = 1;   // TX finished (radio back in RX)
  static const uint8_t EV_RXDONE = 2;   // frame waiting - call rxRead()
  static const uint8_t EV_CRCERR = 4;   // frame arrived but failed CRC
  // Cheap: reads SPI only when DIO0 is high or a TX is in flight - plus a
  // timed fallback read (kFlagPollMs) so a loose DIO0 wire costs latency,
  // never the whole receive path.
  uint8_t poll();

  // Diagnostics: raw register read (does not clear IRQ flags) and DIO0 level.
  uint8_t reg(uint8_t r) { return present_ ? rd(r) : 0; }
  bool dio0High() const { return dio0_ >= 0 && digitalRead(dio0_) == HIGH; }

  // After EV_RXDONE: copies the packet, returns its length (0 on failure).
  int rxRead(uint8_t *buf, int cap);

  int16_t pktRssiDbm() const { return rssi_dbm_; }
  float pktSnrDb() const { return snr_db_; }
  bool present() const { return present_; }
  bool txBusy() const { return tx_busy_; }
  void sleep();

 private:
  uint8_t rd(uint8_t reg);
  void wr(uint8_t reg, uint8_t v);
  void rdBurst(uint8_t reg, uint8_t *p, int n);
  void wrBurst(uint8_t reg, const uint8_t *p, int n);
  void setMode(uint8_t m);

  SPIClass *spi_ = nullptr;
  int8_t cs_ = -1, rst_ = -1, dio0_ = -1;
  bool present_ = false;
  bool tx_busy_ = false;
  uint32_t last_flags_ms_ = 0;
  int16_t rssi_dbm_ = 0;
  float snr_db_ = 0;
};
