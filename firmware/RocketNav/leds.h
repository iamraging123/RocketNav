// leds.h — WS2812B status strips, hand bit-banged (no library, no DMA).
// Two strips on independent GPIOs; colors are ground-commanded over the
// telemetry serial ($led, documented in SCHEMA.md). A whole strip carries
// one color at a time — these are status lights, not a display.
#pragma once
#include <stdint.h>

#define LEDS_MAX_PER_STRIP 8

// pin_*: Arduino pin ids (e.g. PC10). Also enables the DWT cycle counter
// used for pulse timing, and queues an all-off push so strip state is
// known after boot.
void ledsInit(uint32_t pin_a, uint8_t n_a, uint32_t pin_b, uint8_t n_b);

// Set every pixel of one strip (0 = a, 1 = b) to r/g/b. Takes effect at the
// next ledsService() call.
void ledsSet(uint8_t strip, uint8_t r, uint8_t g, uint8_t b);

// Push at most ONE dirty strip per call (keeps the caller's time budget
// honest: a 4-pixel refresh masks IRQs for ~120 us). Call from the service
// slot; the slot cadence also guarantees the >50 us reset latch.
void ledsService();
