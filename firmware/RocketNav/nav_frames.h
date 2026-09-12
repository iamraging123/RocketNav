// nav_frames.h — quaternion utilities and WGS-84 <-> local NED conversions.
// Platform-free (no Arduino dependencies) so the ESKF and the host-side test
// can share it. float32 throughout except geodetic output formatting, where
// float32 cannot represent 1e-7 deg at mid-latitudes (~0.5 m quantization),
// so lat/lon travel as int32 1e-7 deg and only printing uses double.
//
// Conventions (fixed for this project):
//   * Quaternion: scalar-first [w,x,y,z], Hamilton product, unit norm.
//     q represents the rotation BODY -> NED: v_ned = R(q) * v_body.
//   * NED: X north, Y east, Z down. Gravity is +Z (down).
//   * "Upright" vehicle = identity attitude = body Z pointing down
//     (per the config-block bench check: stationary upright accel reads
//     -9.81 m/s^2 on body Z). The vehicle flies near identity attitude, so
//     the ZYX Euler singularity (pitch +/-90 deg) is far from the flight
//     envelope.
//   * Euler ZYX aerospace: yaw about Z(down), pitch about Y, roll about X;
//     R = Rz(yaw)*Ry(pitch)*Rx(roll).

#pragma once
#include <stdint.h>
#include <math.h>

namespace nav {

constexpr float PI_F     = 3.14159265358979f;
constexpr float DEG2RAD  = 0.0174532925199433f;
constexpr float RAD2DEG  = 57.2957795130823f;

// ---------- small vector helpers (float[3]) ----------
inline float v3_dot(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline void v3_cross(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}
inline float v3_norm(const float a[3]) { return sqrtf(v3_dot(a, a)); }
inline void v3_scale(const float a[3], float s, float out[3]) {
  out[0] = a[0] * s; out[1] = a[1] * s; out[2] = a[2] * s;
}

// Wrap an angle to (-pi, pi].
float wrap_pi(float a);

// ---------- quaternion ----------
void quat_identity(float q[4]);
void quat_mul(const float a[4], const float b[4], float out[4]);  // out = a (x) b
void quat_normalize(float q[4]);
// Exact exponential map: rotation vector (rad, axis*angle) -> unit quaternion.
void quat_from_rotvec(const float rv[3], float q[4]);
// Direction cosine matrix, row-major R[9], body->NED: v_n = R * v_b.
void quat_to_dcm(const float q[4], float R[9]);
void dcm_mul_vec(const float R[9], const float v[3], float out[3]);   // R * v
void dcm_t_mul_vec(const float R[9], const float v[3], float out[3]); // R' * v
void quat_to_euler(const float q[4], float eul[3]);  // [roll, pitch, yaw] rad
void quat_from_euler(float roll, float pitch, float yaw, float q[4]);

// ---------- WGS-84 geodesy ----------
// Local-level NED tangent frame anchored at the first valid GNSS fix.
// Flat-earth linearization about the anchor using the meridian and prime
// vertical radii of curvature; adequate for HPR ranges (curvature error
// ~d^2/2R: <1 m within 3.5 km of the pad) and exactly self-inverse, which is
// what the filter needs for consistency.
struct GeoOrigin {
  bool    valid = false;
  int32_t lat1e7 = 0;        // anchor latitude, 1e-7 deg
  int32_t lon1e7 = 0;        // anchor longitude, 1e-7 deg
  int32_t hae_mm = 0;        // anchor height above ellipsoid, mm
  float   undulation_m = 0;  // (hae - hMSL) at anchor, for MSL output
  double  lat_rad = 0;       // anchor latitude, rad (double: one-time trig)
  float   rn_m = 0;          // meridian radius of curvature at anchor
  float   re_cos_m = 0;      // prime-vertical radius * cos(lat) at anchor
  float   gravity = 9.80665f; // local gravity magnitude (Somigliana), m/s^2
};

void geo_origin_init(GeoOrigin &o, int32_t lat1e7, int32_t lon1e7,
                     int32_t hae_mm, int32_t hmsl_mm);
// Geodetic fix -> NED meters relative to origin. Integer deltas keep full
// 1e-7 deg resolution before the float conversion.
void geo_to_ned(const GeoOrigin &o, int32_t lat1e7, int32_t lon1e7,
                int32_t hae_mm, float ned[3]);
// NED meters -> geodetic (double outputs are for telemetry printing only).
void ned_to_geo(const GeoOrigin &o, const float ned[3],
                double *lat_deg, double *lon_deg, double *alt_msl_m);

// WGS-84 normal gravity (Somigliana) with free-air height correction.
float gravity_wgs84(double lat_rad, float h_m);

}  // namespace nav
