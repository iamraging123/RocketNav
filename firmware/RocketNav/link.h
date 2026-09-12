// link - the rocket side of the LoRa air protocol (MAC + queues).
// The rocket is TX-master: one state (or message) frame every
// LC_TX_PERIOD_US, RX-continuous the rest of the time; the base station
// talks only after it hears us, so the channel never collides with our own
// transmissions. Uplink '$' command lines are deduplicated by sequence
// number and handed to the same execCommand() the USB serial feeds.
#pragma once

#include <stdint.h>

#include "linkcodec.h"

class Sx1278;

namespace link {

// The .ino owns the state sampling: fill f with the current snapshot.
typedef void (*StateFillFn)(lc::StateFields *f);

void begin(Sx1278 *radio, StateFillFn fill);

// Call from the radio scheduler slot (a few hundred Hz). Cheap when idle.
void service(uint64_t now_us);

// Queue a msg-record text for downlink (truncated to 48 bytes; message
// frames ride every other TX slot, oldest dropped when the ring is full).
void queueMsg(const char *txt);

// Uplink command line, if one arrived ("$..."). True at most once per frame.
bool popCommand(char *buf, int cap);

void setMute(bool on);
bool muted();

// Telemetry / diagnostics
uint32_t txCount();
uint32_t rxCount();
uint32_t crcErrCount();
bool rssiValid();       // an uplink frame has been received
int16_t lastRssiDbm();  // of the last uplink frame
float lastSnrDb();

}  // namespace link
