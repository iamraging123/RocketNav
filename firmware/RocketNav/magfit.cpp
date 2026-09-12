#include "magfit.h"

#include <math.h>
#include <string.h>

namespace magfit {

void SphereFit::reset() {
  memset(ata_, 0, sizeof(ata_));
  memset(aty_, 0, sizeof(aty_));
  for (int i = 0; i < 3; ++i) {
    m0_[i] = 0;
    mn_[i] = 1e9f;
    mx_[i] = -1e9f;
  }
  n_ = 0;
  have_m0_ = false;
}

void SphereFit::add(const float m_ut[3]) {
  float n2 = m_ut[0] * m_ut[0] + m_ut[1] * m_ut[1] + m_ut[2] * m_ut[2];
  if (!(n2 >= 25.0f && n2 <= 40000.0f)) return;  // also rejects NaN
  if (!have_m0_) {
    for (int i = 0; i < 3; ++i) m0_[i] = m_ut[i];
    have_m0_ = true;
  }
  double a[4];
  double y = 0.0;
  for (int i = 0; i < 3; ++i) {
    if (m_ut[i] < mn_[i]) mn_[i] = m_ut[i];
    if (m_ut[i] > mx_[i]) mx_[i] = m_ut[i];
    double s = (double)m_ut[i] - (double)m0_[i];
    a[i] = 2.0 * s;
    y += s * s;
  }
  a[3] = 1.0;
  for (int i = 0; i < 4; ++i) {
    aty_[i] += a[i] * y;
    for (int j = i; j < 4; ++j) ata_[i][j] += a[i] * a[j];
  }
  n_++;
}

void SphereFit::spreads(float spread_ut[3]) const {
  for (int i = 0; i < 3; ++i) {
    spread_ut[i] = (n_ > 0) ? (mx_[i] - mn_[i]) : 0.0f;
  }
}

bool SphereFit::solve(float c_ut[3], float spread_ut[3]) const {
  spreads(spread_ut);
  if (n_ < 200) return false;  // < 2.5 s of an 80 Hz sweep: not a sweep

  double A[4][4], b[4];
  double dmax = 0.0;
  for (int i = 0; i < 4; ++i) {
    b[i] = aty_[i];
    for (int j = 0; j < 4; ++j) A[i][j] = (j >= i) ? ata_[i][j] : ata_[j][i];
    if (A[i][i] > dmax) dmax = A[i][i];
  }

  // Gaussian elimination with partial pivoting. A degenerate direction
  // (e.g. a perfectly flat spin) drives a pivot toward zero; only genuine
  // collapse fails - thin-but-nonzero coverage is the caller's business,
  // reported through spread_ut.
  int piv[4] = { 0, 1, 2, 3 };
  for (int k = 0; k < 4; ++k) {
    int best = k;
    for (int r = k + 1; r < 4; ++r) {
      if (fabs(A[piv[r]][k]) > fabs(A[piv[best]][k])) best = r;
    }
    int t = piv[k]; piv[k] = piv[best]; piv[best] = t;
    double p = A[piv[k]][k];
    if (fabs(p) < 1e-9 * (dmax > 0.0 ? dmax : 1.0)) return false;
    for (int r = k + 1; r < 4; ++r) {
      double f = A[piv[r]][k] / p;
      for (int cc = k; cc < 4; ++cc) A[piv[r]][cc] -= f * A[piv[k]][cc];
      b[piv[r]] -= f * b[piv[k]];
    }
  }
  double x[4];
  for (int k = 3; k >= 0; --k) {
    double s = b[piv[k]];
    for (int cc = k + 1; cc < 4; ++cc) s -= A[piv[k]][cc] * x[cc];
    x[k] = s / A[piv[k]][k];
  }
  for (int i = 0; i < 3; ++i) {
    double ci = x[i] + (double)m0_[i];  // unshift
    if (!isfinite(ci)) return false;
    c_ut[i] = (float)ci;
  }
  return true;
}

}  // namespace magfit
