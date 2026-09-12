#include "leds.h"
#include <Arduino.h>

// WS2812B, cycle-counted bit-bang: DWT->CYCCNT paces every edge, data hits
// the pin through direct BSRR writes (stm32duino digitalWrite is far too
// slow at these widths). Timing per datasheet, tolerance +-150 ns:
//   bit period 1250 ns, T1H 800 ns, T0H 400 ns.
// One refresh masks IRQs for count * 30 us (4 px = 120 us). That is why
// ledsService() pushes a single strip per call and runs only in the 10 Hz
// service slot — the DRDY-gated IMU poll never collides with it.
//
// PC13 note: the pin lives in the RTC power domain and is rated for ~2 MHz
// output on the F7 — marginal for the 400 ns low bit, but inside what the
// WS2812B input threshold tolerates in practice. Keep that strip's wiring
// short.

namespace {

struct Strip {
  GPIO_TypeDef *port = nullptr;
  uint32_t set_mask = 0, clr_mask = 0;
  uint8_t count = 0;
  uint8_t grb[LEDS_MAX_PER_STRIP * 3] = {0};
  bool dirty = false;
};

Strip strips_[2];

void sendStrip(Strip &s) {
  const uint32_t cyc_us = SystemCoreClock / 1000000UL;
  const uint32_t t1h = cyc_us * 800 / 1000;
  const uint32_t t0h = cyc_us * 400 / 1000;
  const uint32_t bit = cyc_us * 1250 / 1000;
  const uint32_t nbits = (uint32_t)s.count * 24;
  noInterrupts();
  const uint32_t start = DWT->CYCCNT;
  for (uint32_t i = 0; i < nbits; ++i) {
    const bool one = s.grb[i >> 3] & (0x80u >> (i & 7));
    const uint32_t t0 = start + i * bit;
    while ((int32_t)(DWT->CYCCNT - t0) < 0) {}
    s.port->BSRR = s.set_mask;
    const uint32_t th = t0 + (one ? t1h : t0h);
    while ((int32_t)(DWT->CYCCNT - th) < 0) {}
    s.port->BSRR = s.clr_mask;
  }
  const uint32_t tend = start + nbits * bit;  // finish the last low time
  while ((int32_t)(DWT->CYCCNT - tend) < 0) {}
  interrupts();
}

}  // namespace

void ledsInit(uint32_t pin_a, uint8_t n_a, uint32_t pin_b, uint8_t n_b) {
  const uint32_t pins[2] = { pin_a, pin_b };
  const uint8_t counts[2] = { n_a, n_b };
  for (int i = 0; i < 2; ++i) {
    Strip &s = strips_[i];
    s.count = counts[i] > LEDS_MAX_PER_STRIP ? LEDS_MAX_PER_STRIP : counts[i];
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
    const PinName pn = digitalPinToPinName(pins[i]);
    s.port = get_GPIO_Port(STM_PORT(pn));
    s.set_mask = 1u << STM_PIN(pn);
    s.clr_mask = 1u << (STM_PIN(pn) + 16);
    s.dirty = true;              // push the all-off frame once at boot
  }
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void ledsSet(uint8_t strip, uint8_t r, uint8_t g, uint8_t b) {
  if (strip > 1) return;
  Strip &s = strips_[strip];
  for (uint8_t i = 0; i < s.count; ++i) {
    s.grb[i * 3 + 0] = g;        // WS2812B wire order is GRB
    s.grb[i * 3 + 1] = r;
    s.grb[i * 3 + 2] = b;
  }
  s.dirty = true;
}

void ledsService() {
  for (int i = 0; i < 2; ++i) {
    if (strips_[i].dirty) {
      sendStrip(strips_[i]);
      strips_[i].dirty = false;
      return;
    }
  }
}
