// link - the rocket side of the LoRa air protocol (MAC + queues).
// The rocket is TX-master: one state (or message) frame every profile
// period, RX-continuous the rest of the time; the base station talks only
// after it hears us, so the channel never collides with our own
// transmissions. Uplink '$' command lines are deduplicated by sequence
// number and handed to the same execCommand() the USB serial feeds.
//
// Two air profiles (PLAN_LORA_RANGE.md, lc::kProfileFlight / kProfileRecovery):
// FLIGHT sends 54-byte 'T' state frames at 4 Hz; RECOVERY sends 27-byte 'B'
// position beacons every 10 s at +17 dB of sensitivity. The rocket moves to
// RECOVERY by itself on the landed EDGE (SAFE and still for 30 s, reported
// by the .ino) or when the base station's 'K' keepalives have stopped for
// 30 s after being heard - never while armed (ARMED / ACTIVE / BENCH: short
// frames dodge the spin fades), never while muted. Arming returns the link
// to FLIGHT unless the operator pinned RECOVERY. $lora rec / $lora flight
// force either way. Every switch is deferred until the queued acknowledgement
// has left on the OLD profile, so the base hears it.
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
// frames ride every other TX slot in FLIGHT, go out paced at 1.5 s in
// RECOVERY; oldest dropped when the ring is full).
void queueMsg(const char *txt);

// Uplink command line, if one arrived ("$..."). True at most once per frame.
bool popCommand(char *buf, int cap);

void setMute(bool on);
bool muted();

// Profile control. requestProfile schedules the switch: it happens once the
// message queue has drained (the acknowledgement travels on the old
// profile) or after 2 s. manual = operator command: pins RECOVERY against
// the arm-time return to FLIGHT, or clears that pin.
void requestProfile(uint8_t prof, uint64_t now_us, bool manual);
uint8_t profile();            // the profile the modem is on now
void noteLanded(bool landed); // .ino, 10 Hz: control SAFE and still for 30 s
void noteArmed(uint64_t now_us);   // .ino: control just entered ARMED
void setFlightLock(bool armed);    // .ino: ARMED / ACTIVE / BENCH
// One-shot: true once per applied switch, with the new profile.
bool takeProfileChange(uint8_t *to);

// Telemetry / diagnostics
uint32_t txCount();
uint32_t rxCount();          // uplink command frames accepted (both deliveries)
uint32_t keepaliveCount();   // base keepalives heard
uint32_t crcErrCount();
bool rssiValid();       // an uplink frame has been received
int16_t lastRssiDbm();  // of the last uplink frame
float lastSnrDb();
bool baseLost();        // keepalives were seen once and have stopped

}  // namespace link
