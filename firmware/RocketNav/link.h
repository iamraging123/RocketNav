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
// RECOVERY by itself when it has landed (SAFE and still for 30 s, reported by
// the .ino) or when the base station's 'K' keepalives stop for 20 s - never
// while ACTIVE (boost/coast: short frames dodge the spin fades), never while
// muted. $lora rec / $lora flight force either way.
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
// frames ride every other TX slot in FLIGHT, go out at once in RECOVERY;
// oldest dropped when the ring is full).
void queueMsg(const char *txt);

// Uplink command line, if one arrived ("$..."). True at most once per frame.
bool popCommand(char *buf, int cap);

void setMute(bool on);
bool muted();

// Profile control. setProfile re-programs the modem immediately.
void setProfile(uint8_t prof, uint64_t now_us);  // lc::kProfFlight / kProfRecovery
uint8_t profile();
void noteLanded(bool landed);        // .ino: control SAFE and still for 30 s
void setFlightLock(bool in_boost);   // .ino: control ACTIVE -> no auto-switch

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
