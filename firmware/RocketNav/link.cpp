#include "link.h"

#include <string.h>

#include "sx1278.h"

namespace link {

static Sx1278 *radio_ = nullptr;
static StateFillFn fill_ = nullptr;
static uint64_t next_tx_us_ = 0;
static uint8_t tx_seq_ = 0;
static bool mute_ = false;

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

static uint32_t n_tx_ = 0, n_rx_ = 0, n_crc_ = 0;
static bool rssi_valid_ = false;

void begin(Sx1278 *radio, StateFillFn fill) {
  radio_ = radio;
  fill_ = fill;
  next_tx_us_ = 0;
  tx_seq_ = 0;
  mq_head_ = 0;
  mq_count_ = 0;
  cmd_pending_ = false;
  any_cmd_ = false;
  n_tx_ = n_rx_ = n_crc_ = 0;
  rssi_valid_ = false;
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

static void handleRx() {
  uint8_t buf[lc::kMaxFrame];
  int n = radio_->rxRead(buf, sizeof(buf));
  if (n <= 0) return;
  char txt[lc::kMaxText + 1];
  uint8_t seq = 0, type = 0;
  if (!lc::parseText(buf, n, txt, sizeof(txt), &seq, &type)) {
    n_crc_++;  // wrong shape or bad frame CRC-16 (radio CRC already passed)
    return;
  }
  if (type != lc::kTypeCmd) return;  // rocket only consumes commands
  n_rx_++;
  rssi_valid_ = true;
  if (any_cmd_ && seq == last_cmd_seq_) return;  // duplicate delivery
  last_cmd_seq_ = seq;
  any_cmd_ = true;
  strncpy(cmd_, txt, lc::kMaxText);
  cmd_[lc::kMaxText] = 0;
  cmd_pending_ = true;
}

void service(uint64_t now_us) {
  if (radio_ == nullptr || !radio_->present()) return;

  uint8_t ev = radio_->poll();
  if (ev & Sx1278::EV_RXDONE) handleRx();
  if (ev & Sx1278::EV_CRCERR) n_crc_++;

  if (mute_ || radio_->txBusy()) return;
  if (next_tx_us_ == 0) next_tx_us_ = now_us + LC_TX_PERIOD_US;
  if (now_us < next_tx_us_) return;
  next_tx_us_ += LC_TX_PERIOD_US;
  if (now_us > next_tx_us_ + 4ull * LC_TX_PERIOD_US) {
    next_tx_us_ = now_us + LC_TX_PERIOD_US;  // fell far behind: resync
  }

  uint8_t frame[lc::kMaxFrame];
  int len = 0;
  // Messages ride every other slot at most - state frames keep cadence.
  if (mq_count_ > 0 && last_was_state_) {
    len = lc::packText(lc::kTypeMsg, msgq_[mq_head_], tx_seq_++, frame);
    mq_head_ = (uint8_t)((mq_head_ + 1) % kMsgQ);
    mq_count_--;
    last_was_state_ = false;
  } else if (fill_ != nullptr) {
    lc::StateFields f;
    memset(&f, 0, sizeof(f));
    fill_(&f);
    len = lc::packState(f, tx_seq_++, frame);
    last_was_state_ = true;
  }
  if (len > 0 && radio_->txStart(frame, (uint8_t)len)) n_tx_++;
}

uint32_t txCount() { return n_tx_; }
uint32_t rxCount() { return n_rx_; }
uint32_t crcErrCount() { return n_crc_; }
bool rssiValid() { return rssi_valid_; }
int16_t lastRssiDbm() { return radio_ ? radio_->pktRssiDbm() : 0; }
float lastSnrDb() { return radio_ ? radio_->pktSnrDb() : 0; }

}  // namespace link
