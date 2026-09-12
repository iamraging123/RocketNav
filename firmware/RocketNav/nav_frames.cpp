// nav_frames.cpp — implementation of quaternion utilities and WGS-84 <-> NED.
// See nav_frames.h for conventions. Platform-free.

#include "nav_frames.h"

namespace nav {

float wrap_pi(float a) {
  while (a > PI_F)  a -= 2.0f * PI_F;
  while (a <= -PI_F) a += 2.0f * PI_F;
  return a;
}

void quat_identity(float q[4]) {
  q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
}

void quat_mul(const float a[4], const float b[4], float out[4]) {
  // Hamilton product, scalar-first. out may not alias a or b.
  out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

void quat_normalize(float q[4]) {
  float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n < 1e-12f) { quat_identity(q); return; }
  float inv = 1.0f / n;
  q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
}

void quat_from_rotvec(const float rv[3], float q[4]) {
  float ang = v3_norm(rv);
  if (ang < 1e-8f) {
    // Small-angle: q = [1, rv/2], renormalized (second-order exact enough).
    q[0] = 1.0f;
    q[1] = 0.5f * rv[0];
    q[2] = 0.5f * rv[1];
    q[3] = 0.5f * rv[2];
    quat_normalize(q);
    return;
  }
  float half = 0.5f * ang;
  float s = sinf(half) / ang;
  q[0] = cosf(half);
  q[1] = rv[0] * s;
  q[2] = rv[1] * s;
  q[3] = rv[2] * s;
}

void quat_to_dcm(const float q[4], float R[9]) {
  float w = q[0], x = q[1], y = q[2], z = q[3];
  R[0] = 1.0f - 2.0f * (y * y + z * z);
  R[1] = 2.0f * (x * y - w * z);
  R[2] = 2.0f * (x * z + w * y);
  R[3] = 2.0f * (x * y + w * z);
  R[4] = 1.0f - 2.0f * (x * x + z * z);
  R[5] = 2.0f * (y * z - w * x);
  R[6] = 2.0f * (x * z - w * y);
  R[7] = 2.0f * (y * z + w * x);
  R[8] = 1.0f - 2.0f * (x * x + y * y);
}

void dcm_mul_vec(const float R[9], const float v[3], float out[3]) {
  out[0] = R[0] * v[0] + R[1] * v[1] + R[2] * v[2];
  out[1] = R[3] * v[0] + R[4] * v[1] + R[5] * v[2];
  out[2] = R[6] * v[0] + R[7] * v[1] + R[8] * v[2];
}

void dcm_t_mul_vec(const float R[9], const float v[3], float out[3]) {
  out[0] = R[0] * v[0] + R[3] * v[1] + R[6] * v[2];
  out[1] = R[1] * v[0] + R[4] * v[1] + R[7] * v[2];
  out[2] = R[2] * v[0] + R[5] * v[1] + R[8] * v[2];
}

void quat_to_euler(const float q[4], float eul[3]) {
  float R[9];
  quat_to_dcm(q, R);
  // ZYX extraction from body->NED DCM.
  float s = -R[6];                       // sin(pitch) = -R[2][0]
  if (s > 1.0f) s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  eul[0] = atan2f(R[7], R[8]);           // roll
  eul[1] = asinf(s);                     // pitch
  eul[2] = atan2f(R[3], R[0]);           // yaw
}

void quat_from_euler(float roll, float pitch, float yaw, float q[4]) {
  float cr = cosf(0.5f * roll),  sr = sinf(0.5f * roll);
  float cp = cosf(0.5f * pitch), sp = sinf(0.5f * pitch);
  float cy = cosf(0.5f * yaw),   sy = sinf(0.5f * yaw);
  // q = qz(yaw) (x) qy(pitch) (x) qx(roll)
  q[0] = cy * cp * cr + sy * sp * sr;
  q[1] = cy * cp * sr - sy * sp * cr;
  q[2] = cy * sp * cr + sy * cp * sr;
  q[3] = sy * cp * cr - cy * sp * sr;
  quat_normalize(q);
}

// ---------- geodesy ----------

static const double WGS84_A  = 6378137.0;            // semi-major axis, m
static const double WGS84_E2 = 6.69437999014e-3;     // first eccentricity^2

float gravity_wgs84(double lat_rad, float h_m) {
  // Somigliana normal gravity (WGS-84 constants) + free-air correction.
  double s2 = sin(lat_rad); s2 *= s2;
  double g0 = 9.7803253359 * (1.0 + 1.93185265241e-3 * s2) /
              sqrt(1.0 - 6.69437999013e-3 * s2);
  return (float)(g0 - 3.086e-6 * (double)h_m);
}

void geo_origin_init(GeoOrigin &o, int32_t lat1e7, int32_t lon1e7,
                     int32_t hae_mm, int32_t hmsl_mm) {
  o.lat1e7 = lat1e7;
  o.lon1e7 = lon1e7;
  o.hae_mm = hae_mm;
  o.undulation_m = (float)(hae_mm - hmsl_mm) * 1e-3f;
  o.lat_rad = (double)lat1e7 * 1e-7 * (double)DEG2RAD;
  double s2 = sin(o.lat_rad); s2 *= s2;
  double den = 1.0 - WGS84_E2 * s2;
  o.rn_m = (float)(WGS84_A * (1.0 - WGS84_E2) / (den * sqrt(den)));
  o.re_cos_m = (float)((WGS84_A / sqrt(den)) * cos(o.lat_rad));
  o.gravity = gravity_wgs84(o.lat_rad, (float)hae_mm * 1e-3f);
  o.valid = true;
}

void geo_to_ned(const GeoOrigin &o, int32_t lat1e7, int32_t lon1e7,
                int32_t hae_mm, float ned[3]) {
  // Integer deltas first (exact), then scale to radians/meters in float.
  float dlat = (float)(lat1e7 - o.lat1e7) * 1e-7f * DEG2RAD;
  float dlon = (float)(lon1e7 - o.lon1e7) * 1e-7f * DEG2RAD;
  ned[0] = dlat * o.rn_m;
  ned[1] = dlon * o.re_cos_m;
  ned[2] = -(float)(hae_mm - o.hae_mm) * 1e-3f;
}

void ned_to_geo(const GeoOrigin &o, const float ned[3],
                double *lat_deg, double *lon_deg, double *alt_msl_m) {
  // Double only for print formatting (see header note on float lat/lon).
  *lat_deg = (double)o.lat1e7 * 1e-7 +
             (double)(ned[0] / o.rn_m) * (double)RAD2DEG;
  *lon_deg = (double)o.lon1e7 * 1e-7 +
             (double)(ned[1] / o.re_cos_m) * (double)RAD2DEG;
  *alt_msl_m = (double)o.hae_mm * 1e-3 - (double)ned[2] -
               (double)o.undulation_m;
}

}  // namespace nav
