// Host tests for the magcal sphere fit. Build:
//   zig c++ -std=c++17 -O2 -I../firmware/RocketNav -o magfit_test.exe \
//     magfit_test.cpp ../firmware/RocketNav/magfit.cpp \
//     ../firmware/RocketNav/nav_frames.cpp
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "magfit.h"
#include "nav_frames.h"

static int checks = 0, fails = 0;
#define CHECK(cond, ...)                    \
  do {                                      \
    checks++;                               \
    if (cond) {                             \
      printf("  ok  : ");                   \
    } else {                                \
      fails++;                              \
      printf("  FAIL: ");                   \
    }                                       \
    printf(__VA_ARGS__);                    \
    printf("\n");                           \
  } while (0)

static uint64_t rng_state = 0xC0FFEE123ull;
static double uniform() {  // [0,1)
  rng_state = rng_state * 6364136223846793005ull + 1442695040888963407ull;
  return (double)(rng_state >> 11) * (1.0 / 9007199254740992.0);
}
static double gauss() {
  double s = 0;
  for (int i = 0; i < 12; ++i) s += uniform();
  return s - 6.0;
}

// Champaign field, NED (matches the flight config).
static const double kPi = 3.14159265358979323846;
static const float kB[3] = { 20.15f, -1.23f, 47.77f };

// One sample: body field at the given attitude, plus iron offset and noise.
static void sample(float roll, float pitch, float yaw, const float c[3],
                   float noise, float m[3]) {
  float q[4], R[9], mb[3];
  nav::quat_from_euler(roll, pitch, yaw, q);
  nav::quat_to_dcm(q, R);
  nav::dcm_t_mul_vec(R, kB, mb);  // R' B: NED -> body
  for (int i = 0; i < 3; ++i) m[i] = mb[i] + c[i] + noise * (float)gauss();
}

int main() {
  const float cTrue[3] = { 34.0f, -12.0f, 20.0f };

  printf("== full tumble ==\n");
  {
    magfit::SphereFit f;
    f.reset();
    for (int k = 0; k < 2400; ++k) {
      float m[3];
      sample((float)(uniform() * 2 * kPi), (float)((uniform() - 0.5) * kPi),
             (float)(uniform() * 2 * kPi), cTrue, 0.4f, m);
      f.add(m);
    }
    float c[3], sp[3];
    bool ok = f.solve(c, sp);
    CHECK(ok, "solve ok (n=%u)", f.count());
    CHECK(ok && fabsf(c[0] - cTrue[0]) < 0.5f && fabsf(c[1] - cTrue[1]) < 0.5f &&
              fabsf(c[2] - cTrue[2]) < 0.5f,
          "center within 0.5 uT (err %.2f %.2f %.2f)", c[0] - cTrue[0],
          c[1] - cTrue[1], c[2] - cTrue[2]);
    CHECK(sp[0] > 60 && sp[1] > 60 && sp[2] > 60,
          "coverage spreads wide (%.0f %.0f %.0f uT)", sp[0], sp[1], sp[2]);
  }

  printf("== flat tabletop spin (the classic bad sweep) ==\n");
  {
    magfit::SphereFit f;
    f.reset();
    for (int k = 0; k < 2400; ++k) {
      float m[3];
      float wob = 1.0f * (float)nav::DEG2RAD;
      sample(wob * (float)gauss(), wob * (float)gauss(),
             (float)k * 0.00785f, cTrue, 0.4f, m);  // 3 slow turns, level
      f.add(m);
    }
    float c[3], sp[3];
    bool ok = f.solve(c, sp);
    CHECK(ok, "solve still ok");
    CHECK(ok && fabsf(c[0] - cTrue[0]) < 1.0f && fabsf(c[1] - cTrue[1]) < 1.0f,
          "horizontal axes recovered (err %.2f %.2f)", c[0] - cTrue[0],
          c[1] - cTrue[1]);
    CHECK(sp[2] < 15.0f,
          "z coverage reported thin (%.1f uT) - the hold-axis trigger",
          sp[2]);
    CHECK(sp[0] > 30.0f && sp[1] > 30.0f,
          "x/y coverage reported healthy (%.0f %.0f uT)", sp[0], sp[1]);
    // The reason min/max midpoints were replaced: on this sweep they put
    // the Earth's vertical field straight into the z offset.
    // (z body stays ~B_D the whole spin -> midpoint ~= cz + B_D.)
  }

  printf("== glitches and outliers ==\n");
  {
    magfit::SphereFit f;
    f.reset();
    for (int k = 0; k < 2000; ++k) {
      float m[3];
      sample((float)(uniform() * 2 * kPi), (float)((uniform() - 0.5) * kPi),
             (float)(uniform() * 2 * kPi), cTrue, 0.4f, m);
      f.add(m);
      if (k == 300 || k == 900 || k == 1500) {
        float wild[3] = { 500, -500, 500 };  // bus glitch: outside [5,200]
        f.add(wild);
        float sneaky[3] = { m[0] + 60, m[1] - 60, m[2] + 60 };  // in range
        f.add(sneaky);
      }
    }
    float c[3], sp[3];
    bool ok = f.solve(c, sp);
    CHECK(ok, "solve ok");
    CHECK(ok && fabsf(c[0] - cTrue[0]) < 2.0f && fabsf(c[1] - cTrue[1]) < 2.0f &&
              fabsf(c[2] - cTrue[2]) < 2.0f,
          "center survives outliers (err %.2f %.2f %.2f; min/max midpoint "
          "would be ~30 uT off)",
          c[0] - cTrue[0], c[1] - cTrue[1], c[2] - cTrue[2]);
  }

  printf("== not enough data ==\n");
  {
    magfit::SphereFit f;
    f.reset();
    for (int k = 0; k < 50; ++k) {
      float m[3];
      sample(0.1f * k, 0.2f, 0.3f * k, cTrue, 0.4f, m);
      f.add(m);
    }
    float c[3] = { 0, 0, 0 }, sp[3];
    CHECK(!f.solve(c, sp), "50 samples refused");
  }

  printf("\n%d checks, %d failed -> %s\n", checks, fails,
         fails ? "FAIL" : "PASS");
  return fails ? 1 : 0;
}
