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
static uint8_t prof_ = lc::kProfFlight;
static bool landed_ = false;
static bool flight_lock_ = false;
static bool ka_seen_ = false;
static uint64_t last_base_us_ = 0;
static const uint64_t kBaseLostUs = 20000000ull;   // 20 s without a keepalive
static const uint64_t kSwitchSettleUs = 1000000ull; // first TX after a switch

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
  landed_ = false;
  flight_lock_ = false;
  ka_seen_ = false;
  last_base_us_ = 0;
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

void setProfile(uint8_t p, uint64_t now_us) {
  if (p != lc::kProfFlight && p != lc::kProfRecovery) return;
  if (p == prof_) return;
  const lc::Profile &pr = prof(p);
  if (radio_ != nullptr && radio_->present()) {
    radio_->setModem(pr.sf, pr.bw_hz, pr.cr_denom, pr.preamble);
  }
  prof_ = p;
  next_tx_us_ = now_us + kSwitchSettleUs;
  last_was_state_ = false;
  last_base_us_ = now_us;  // give the base its scan time before "lost" again
}

static bool base_lost_ = false;
uint8_t profile() { return prof_; }
void noteLanded(bool landed) { landed_ = landed; }
void setFlightLock(bool in_boost) { flight_lock_ = in_boost; }
bool baseLost() { return base_lost_; }

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

void service(uint64_t now_us) {
  if (radio_ == nullptr || !radio_->present()) return;

  uint8_t ev = radio_->poll();
  if (ev & Sx1278::EV_RXDONE) handleRx(now_us);
  if (ev & Sx1278::EV_CRCERR) n_crc_++;

  // Self-directed move to the recovery profile: landed, or the base has
  // gone quiet after once being heard. Never during boost/coast, never
  // while muted (bench), never twice.
  base_lost_ = ka_seen_ && (now_us - last_base_us_) > kBaseLostUs;
  if (!mute_ && !flight_lock_ && prof_ == lc::kProfFlight &&
      (landed_ || base_lost_)) {
    setProfile(lc::kProfRecovery, now_us);
  }

  if (mute_ || radio_->txBusy()) return;

  // RECOVERY: replies must not wait for a 10 s slot - a queued message goes
  // out at once (the base has just transmitted and is back in RX).
  if (prof_ == lc::kProfRecovery && mq_count_ > 0) {
    uint8_t frame[lc::kMaxFrame];
    int len = packMsgFrame(frame);
    if (len > 0 && radio_->txStart(frame, (uint8_t)len)) n_tx_++;
    return;
  }

  const uint64_t period = prof(prof_).period_us;
  if (next_tx_us_ == 0) next_tx_us_ = now_us + period;
  if (now_us < next_tx_us_) return;
  next_tx_us_ += period;
  if (now_us > next_tx_us_ + 4ull * period) {
    next_tx_us_ = now_us + period;  // fell far behind: resync
  }

  uint8_t frame[lc::kMaxFrame];
  int len = 0;
  // FLIGHT: messages ride every other slot at most - state frames keep cadence.
  if (mq_count_ > 0 && last_was_state_) {
    len = packMsgFrame(frame);
    last_was_state_ = false;
  } else {
    len = packDownlink(frame);
    last_was_state_ = true;
  }
  if (len > 0 && radio_->txStart(frame, (uint8_t)len)) n_tx_++;
}

uint32_t txCount() { return n_tx_; }
uint32_t rxCount() { return n_rx_; }
uint32_t keepaliveCount() { return n_ka_; }
uint32_t crcErrCount() { return n_crc_; }
bool rssiValid() { return rssi_valid_; }
int16_t lastRssiDbm() { return radio_ ? radio_->pktRssiDbm() : 0; }
float lastSnrDb() { return radio_ ? radio_->pktSnrDb() : 0; }

}  // namespace link
