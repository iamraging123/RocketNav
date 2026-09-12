#include "cfgstore.h"

#include <string.h>

#include "cfgcodec.h"
#include "stm32f7xx_hal.h"

namespace cfgstore {

// Sector 7 of the F722's 512 KB single-bank flash: 4x16K + 64K + 3x128K,
// so the last sector starts at 0x08060000.
static const uint32_t kSector = FLASH_SECTOR_7;
static const uint32_t kAddr = 0x08060000UL;

static cfgcodec::Record cache_;
static bool loaded_ = false;   // cache_ initialized (from flash or zeroed)
static bool mag_ok_ = false;   // a stored mag record exists
static bool lnk_ok_ = false;   // at least one stored fin is calibrated

static void dcacheInvalidate() {
  // Flash content changed (or is about to be re-read for verification)
  // behind the cache's back. Only touch the cache when it is actually on.
#if (__DCACHE_PRESENT == 1U)
  if (SCB->CCR & SCB_CCR_DC_Msk) {
    // kAddr is 128 KB aligned, so the 32-byte-alignment requirement holds.
    SCB_InvalidateDCache_by_Addr((uint32_t *)kAddr,
                                 (int32_t)(sizeof(cfgcodec::Record) + 32));
  }
#endif
}

static void ensureLoaded() {
  if (loaded_) return;
  loaded_ = true;
  dcacheInvalidate();
  uint8_t img[sizeof(cfgcodec::Record)];
  memcpy(img, (const void *)kAddr, sizeof(img));
  cfgcodec::parse(img, &cache_, &mag_ok_, &lnk_ok_);
}

static bool eraseSector() {
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                         FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR |
                         FLASH_FLAG_ERSERR);
  FLASH_EraseInitTypeDef e;
  e.TypeErase = FLASH_TYPEERASE_SECTORS;
  e.Sector = kSector;
  e.NbSectors = 1;
  e.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // 3.3 V rail: x32 parallelism
  uint32_t bad = 0;
  HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&e, &bad);
  HAL_FLASH_Lock();
  dcacheInvalidate();
  return st == HAL_OK;
}

// Erase + rewrite the whole cached record. ~1-2 s CPU stall.
static bool commit() {
  cfgcodec::seal(&cache_);
  if (!eraseSector()) return false;
  HAL_FLASH_Unlock();
  const uint32_t *w = (const uint32_t *)&cache_;
  bool ok = true;
  for (uint32_t i = 0; i < sizeof(cache_) / 4; ++i) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, kAddr + 4 * i, w[i]) !=
        HAL_OK) {
      ok = false;
      break;
    }
  }
  HAL_FLASH_Lock();
  dcacheInvalidate();
  if (!ok) return false;
  return memcmp((const void *)kAddr, &cache_, sizeof(cache_)) == 0;
}

bool loadMagHard(float hard_ut[3]) {
  ensureLoaded();
  if (!mag_ok_) return false;
  memcpy(hard_ut, cache_.hard, sizeof(cache_.hard));
  return true;
}

bool saveMagHard(const float hard_ut[3]) {
  ensureLoaded();
  memcpy(cache_.hard, hard_ut, sizeof(cache_.hard));
  bool ok = commit();
  if (ok) mag_ok_ = true;
  return ok;
}

bool loadLinkage(LinkageCal lc[4]) {
  ensureLoaded();
  cfgcodec::toLinkage(cache_, lc);
  return lnk_ok_;
}

bool saveLinkage(const LinkageCal lc[4]) {
  ensureLoaded();
  cfgcodec::fromLinkage(&cache_, lc);
  lnk_ok_ = false;
  for (int i = 0; i < 4; ++i) {
    if (cache_.fin[i].n >= 2) lnk_ok_ = true;
  }
  return commit();
}

bool inspectRaw(FlashInfo *out) {
  // Non-destructive: re-read the physical sector (past the cache) and report
  // what is ACTUALLY stored - the definitive "did the last save persist?"
  // check on hardware, independent of the RAM cache.
  dcacheInvalidate();
  uint8_t img[sizeof(cfgcodec::Record)];
  memcpy(img, (const void *)kAddr, sizeof(img));
  memcpy(&out->magic, img, sizeof(out->magic));
  cfgcodec::Record r;
  bool m = false, l = false;
  int ver = cfgcodec::parse(img, &r, &m, &l);
  out->version = ver;
  out->crc_ok = (ver != 0);
  out->mag_ok = m;
  out->lnk_ok = l;
  for (int i = 0; i < 4; ++i) out->fin_n[i] = r.fin[i].n;
  memcpy(out->hard, r.hard, sizeof(out->hard));
  return ver != 0;
}

}  // namespace cfgstore
