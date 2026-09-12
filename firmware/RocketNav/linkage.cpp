#include "linkage.h"

namespace linkage {

float deflToUs(const LinkageCal &lc, float d_deg) {
  int n = lc.n;
  if (n < 2) return 0.0f;
  if (d_deg <= lc.deg[0]) return lc.us[0];
  if (d_deg >= lc.deg[n - 1]) return lc.us[n - 1];
  for (int i = 1; i < n; ++i) {
    if (d_deg <= lc.deg[i]) {
      float f = (d_deg - lc.deg[i - 1]) / (lc.deg[i] - lc.deg[i - 1]);
      return lc.us[i - 1] + f * (lc.us[i] - lc.us[i - 1]);
    }
  }
  return lc.us[n - 1];
}

void synthDefault(LinkageCal &lc, float center_us, float us_per_deg,
                  float min_us, float max_us) {
  float d0 = (min_us - center_us) / us_per_deg;
  float d1 = (max_us - center_us) / us_per_deg;
  lc.n = 2;
  if (d0 < d1) {
    lc.deg[0] = d0; lc.us[0] = min_us;
    lc.deg[1] = d1; lc.us[1] = max_us;
  } else {
    lc.deg[0] = d1; lc.us[0] = max_us;
    lc.deg[1] = d0; lc.us[1] = min_us;
  }
}

void synthOffset(LinkageCal &lc, float d0_deg, float u0_us, float us_per_deg,
                 float min_us, float max_us) {
  // The line u = u0 + (d - d0) * us_per_deg, sampled at the pulse limits so
  // deflToUs(d0) == u0 exactly and the ends are the mechanical limits.
  float d_at_min = d0_deg + (min_us - u0_us) / us_per_deg;
  float d_at_max = d0_deg + (max_us - u0_us) / us_per_deg;
  lc.n = 2;
  if (d_at_min < d_at_max) {
    lc.deg[0] = d_at_min; lc.us[0] = min_us;
    lc.deg[1] = d_at_max; lc.us[1] = max_us;
  } else {
    lc.deg[0] = d_at_max; lc.us[0] = max_us;
    lc.deg[1] = d_at_min; lc.us[1] = min_us;
  }
}

bool finalize(LinkageCal &lc) {
  if (lc.n < 2 || lc.n > 5) return false;
  // Range bounds MIRROR cfgcodec's boot-time finSane(): anything finalize
  // accepts must survive a power cycle. Without this a |deg| >= 90 typo
  // saved fine, then finSane zeroed the whole fin at reboot - a silent
  // revert to the default table.
  for (int i = 0; i < lc.n; ++i) {
    if (!(lc.deg[i] > -90.0f && lc.deg[i] < 90.0f)) return false;
    if (!(lc.us[i] > 500.0f && lc.us[i] < 2500.0f)) return false;
  }
  // insertion sort by canard angle (n <= 5)
  for (int i = 1; i < lc.n; ++i) {
    for (int j = i; j > 0 && lc.deg[j] < lc.deg[j - 1]; --j) {
      float td = lc.deg[j]; lc.deg[j] = lc.deg[j - 1]; lc.deg[j - 1] = td;
      float tu = lc.us[j]; lc.us[j] = lc.us[j - 1]; lc.us[j - 1] = tu;
    }
  }
  for (int i = 1; i < lc.n; ++i) {
    if (lc.deg[i] - lc.deg[i - 1] < 0.5f) return false;  // duplicate angle
  }
  bool up = lc.us[1] > lc.us[0];
  for (int i = 1; i < lc.n; ++i) {
    float d = lc.us[i] - lc.us[i - 1];
    // A monotonic linkage cannot reverse; < 1 us per step is a mis-entry.
    if ((up && d < 1.0f) || (!up && d > -1.0f)) return false;
  }
  return true;
}

float endAuthorityDeg(const LinkageCal &lc) {
  if (lc.n < 2) return 0.0f;
  float a = lc.deg[0] < 0 ? -lc.deg[0] : lc.deg[0];
  float b = lc.deg[lc.n - 1] < 0 ? -lc.deg[lc.n - 1] : lc.deg[lc.n - 1];
  return a < b ? a : b;
}

}  // namespace linkage
