#include "link.h"

#include <string.h>

#include "sx1278.h"

namespace link {

static Sx1278 *radio_ = nullptr;
static StateFillFn fill_ = nullptr;
static uint64_t next_tx_us_ = 0;
static uint8_t tx_seq_ = 0;
static bool mute_ = false;

// profile state
static const uint8_t kNoPending = 0xFF;
static uint8_t prof_ = lc::kProfFlight;
static uint8_t pending_prof_ = kNoPending;
static uint64_t pending_deadline_us_ = 0;
static bool manual_rec_ = false;     // operator pinned RECOVERY
static bool landed_ = false, landed_seen_ = false;
static bool flight_lock_ = false;
static bool ka_seen_ = false;
static bool base_lost_ = false;
static uint64_t last_base_us_ = 0;
static bool change_flag_ = false;
static uint8_t change_to_ = lc::kProfFlight;
static uint64_t next_msg_ok_us_ = 0;  // RECOVERY message pacing
static const uint64_t kBaseLostUs = 30000000ull;     // 30 s without a keepalive
static const uint64_t kSwitchSettleUs = 1000000ull;  // first TX after a switch
static const uint64_t kPendingMaxUs = 2000000ull;    // ack must leave within 2 s
static const uint64_t kRecMsgGapUs = 1500000ull;     // RECOVERY: room for the
                                                     // base's cued 0.6 s uplink

// msg downlink ring
static const int kMsgQ = 4;
static char msgq_[kMsgQ][lc::kMaxText + 1];
static uint8_t mq_head_ = 0, mq_count_ = 0;
static bool last_was_state_ = false;

// uplink command hand-off (single slot: commands are rare and short)
static char cmd_[lc::kMaxText + 1];
static bool cmd_pending_ = false;
static uint8_t last_cmd_seq_ = 0;
static bool any_cmd_ = false;

static uint32_t n_tx_ = 0, n_rx_ = 0, n_ka_ = 0, n_crc_ = 0;
static bool rssi_valid_ = false;

static const lc::Profile &prof(uint8_t p) {
  return p == lc::kProfRecovery ? lc::kProfileRecovery : lc::kProfileFlight;
}

void begin(Sx1278 *radio, StateFillFn fill) {
  radio_ = radio;
  fill_ = fill;
  next_tx_us_ = 0;
  tx_seq_ = 0;
  mq_head_ = 0;
  mq_count_ = 0;
  cmd_pending_ = false;
  any_cmd_ = false;
  n_tx_ = n_rx_ = n_ka_ = n_crc_ = 0;
  rssi_valid_ = false;
  prof_ = lc::kProfFlight;  // configure() at boot programmed the flight modem
  pending_prof_ = kNoPending;
  manual_rec_ = false;
  landed_ = landed_seen_ = false;
  flight_lock_ = false;
  ka_seen_ = false;
  base_lost_ = false;
  last_base_us_ = 0;
  change_flag_ = false;
  next_msg_ok_us_ = 0;
}

void queueMsg(const char *txt) {
  if (radio_ == nullptr || !radio_->present()) return;
  if (mq_count_ == kMsgQ) {  // full: drop the oldest, keep the freshest
    mq_head_ = (uint8_t)((mq_head_ + 1) % kMsgQ);
    mq_count_--;
  }
  uint8_t slot = (uint8_t)((mq_head_ + mq_count_) % kMsgQ);
  strncpy(msgq_[slot], txt, lc::kMaxText);
  msgq_[slot][lc::kMaxText] = 0;
  mq_count_++;
}

bool popCommand(char *buf, int cap) {
  if (!cmd_pending_) return false;
  strncpy(buf, cmd_, (size_t)cap - 1);
  buf[cap - 1] = 0;
  cmd_pending_ = false;
  return true;
}

void setMute(bool on) { mute_ = on; }
bool muted() { return mute_; }

static void applyProfile(uint8_t p, uint64_t now_us) {
  const lc::Profile &pr = prof(p);
  if (radio_ != nullptr && radio_->present()) {
    radio_->setModem(pr.sf, pr.bw_hz, pr.cr_denom, pr.preamble);
  }
  prof_ = p;
  next_tx_us_ = now_us + kSwitchSettleUs;
  next_msg_ok_us_ = 0;
  last_was_state_ = false;
  last_base_us_ = now_us;  // the base needs its follow/scan time first
  base_lost_ = false;
  change_flag_ = true;
  change_to_ = p;
}

void requestProfile(uint8_t p, uint64_t now_us, bool manual) {
  if (p != lc::kProfFlight && p != lc::kProfRecovery) return;
  if (manual) manual_rec_ = (p == lc::kProfRecovery);
  if (p == prof_ && pending_prof_ == kNoPending) return;
  if (p == prof_) { pending_prof_ = kNoPending; return; }  // cancelled
  pending_prof_ = p;
  pending_deadline_us_ = now_us + kPendingMaxUs;
}

uint8_t profile() { return prof_; }

void noteLanded(bool landed) {
  landed_ = landed;
  if (!landed) landed_seen_ = false;  // re-arm the edge for the next landing
}

void noteArmed(uint64_t now_us) {
  // A re-arm means another flight: the display link comes back unless the
  // operator pinned RECOVERY.
  if (!manual_rec_ && prof_ != lc::kProfFlight) {
    requestProfile(lc::kProfFlight, now_us, false);
  }
  landed_seen_ = false;
}

void setFlightLock(bool armed) { flight_lock_ = armed; }
bool baseLost() { return base_lost_; }

bool takeProfileChange(uint8_t *to) {
  if (!change_flag_) return false;
  change_flag_ = false;
  *to = change_to_;
  return true;
}

static void noteBase(uint64_t now_us) {
  last_base_us_ = now_us;
  rssi_valid_ = true;
}

static void handleRx(uint64_t now_us) {
  uint8_t buf[lc::kMaxFrame];
  int n = radio_->rxRead(buf, sizeof(buf));
  if (n <= 0) return;
  uint8_t seq = 0, type = 0;
  if (lc::parseKeepalive(buf, n, &seq)) {
    n_ka_++;
    ka_seen_ = true;
    noteBase(now_us);
    return;
  }
  char txt[lc::kMaxText + 1];
  if (!lc::parseText(buf, n, txt, sizeof(txt), &seq, &type)) {
    n_crc_++;  // wrong shape or bad frame CRC-16 (radio CRC already passed)
    return;
  }
  if (type != lc::kTypeCmd) return;  // rocket only consumes commands
  n_rx_++;
  noteBase(now_us);
  if (any_cmd_ && seq == last_cmd_seq_) return;  // duplicate delivery
  last_cmd_seq_ = seq;
  any_cmd_ = true;
  strncpy(cmd_, txt, lc::kMaxText);
  cmd_[lc::kMaxText] = 0;
  cmd_pending_ = true;
}

static int packMsgFrame(uint8_t *frame) {
  int len = lc::packText(lc::kTypeMsg, msgq_[mq_head_], tx_seq_++, frame);
  mq_head_ = (uint8_t)((mq_head_ + 1) % kMsgQ);
  mq_count_--;
  return len;
}

static int packDownlink(uint8_t *frame) {
  lc::StateFields f;
  memset(&f, 0, sizeof(f));
  if (fill_ != nullptr) fill_(&f);
  if (prof_ == lc::kProfRecovery) {
    lc::BeaconFields b;
    b.ms = f.ms;
    b.fst = f.fst;
    b.fix = f.fix;
    b.cmode = f.cmode;
    b.lat_deg = f.lat_deg;
    b.lon_deg = f.lon_deg;
    b.alt_msl_m = f.alt_msl_m;
    b.ral_m = f.ral_m;
    for (int i = 0; i < 3; ++i) b.eul_deg[i] = f.eul_deg[i];
    b.sats = f.sats;
    b.health = f.health;
    return lc::packBeacon(b, tx_seq_++, frame);
  }
  return lc::packState(f, tx_seq_++, frame);
}

static void startTx(const uint8_t *frame, int len, uint64_t now_us) {
  if (len > 0 && radio_->txStart(frame, (uint8_t)len)) {
    n_tx_++;
    if (prof_ == lc::kProfRecovery) next_msg_ok_us_ = now_us + kRecMsgGapUs;
  }
}

void service(uint64_t now_us) {
  if (radio_ == nullptr || !radio_->present()) return;

  uint8_t ev = radio_->poll();
  if (ev & Sx1278::EV_RXDONE) handleRx(now_us);
  if (ev & Sx1278::EV_CRCERR) n_crc_++;

  // Self-directed move to the recovery profile, on the landed EDGE or once
  // the base has gone quiet after being heard. Never while armed, muted, or
  // already leaving.
  base_lost_ = ka_seen_ && (now_us - last_base_us_) > kBaseLostUs;
  if (!mute_ && !flight_lock_ && prof_ == lc::kProfFlight &&
      pending_prof_ == kNoPending) {
    bool landed_edge = landed_ && !landed_seen_;
    if (landed_edge || base_lost_) {
      landed_seen_ = landed_;
      queueMsg(landed_edge ? "lora: -> RECOVERY (landed)"
                           : "lora: -> RECOVERY (base lost)");
      requestProfile(lc::kProfRecovery, now_us, false);
    }
  }

  // Apply a pending switch once the acknowledgement has left on the old
  // profile (queue drained and the last frame off the air), or on timeout.
  if (pending_prof_ != kNoPending && !radio_->txBusy() &&
      (mq_count_ == 0 || now_us >= pending_deadline_us_)) {
    uint8_t p = pending_prof_;
    pending_prof_ = kNoPending;
    applyProfile(p, now_us);
    return;
  }

  if (mute_ || radio_->txBusy()) return;

  const uint64_t period = prof(prof_).period_us;
  if (next_tx_us_ == 0) next_tx_us_ = now_us + period;
  const bool slot_due = now_us >= next_tx_us_;

  // RECOVERY: replies must not wait for a 10 s slot, but they are paced so
  // the base's cued uplink (30 ms after our frame, ~0.6 s on SF11) always
  // finds us listening, and the beacon slot always wins.
  if (prof_ == lc::kProfRecovery && !slot_due && mq_count_ > 0 &&
      now_us >= next_msg_ok_us_) {
    uint8_t frame[lc::kMaxFrame];
    int len = packMsgFrame(frame);
    startTx(frame, len, now_us);
    return;
  }
  if (!slot_due) return;
  next_tx_us_ += period;
  if (now_us > next_tx_us_ + 4ull * period) {
    next_tx_us_ = now_us + period;  // fell far behind: resync
  }

  uint8_t frame[lc::kMaxFrame];
  int len = 0;
  // FLIGHT: messages ride every other slot at most - state frames keep cadence.
  if (prof_ == lc::kProfFlight && mq_count_ > 0 && last_was_state_) {
    len = packMsgFrame(frame);
    last_was_state_ = false;
  } else {
    len = packDownlink(frame);
    last_was_state_ = true;
  }
  startTx(frame, len, now_us);
}

uint32_t txCount() { return n_tx_; }
uint32_t rxCount() { return n_rx_; }
uint32_t keepaliveCount() { return n_ka_; }
uint32_t crcErrCount() { return n_crc_; }
bool rssiValid() { return rssi_valid_; }
int16_t lastRssiDbm() { return radio_ ? radio_->pktRssiDbm() : 0; }
float lastSnrDb() { return radio_ ? radio_->pktSnrDb() : 0; }

}  // namespace link
