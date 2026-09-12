# RocketNav

Flight navigation firmware for a high-power rocket: a 16-state error-state
Kalman filter (GNSS + IMU + magnetometer + barometer) on an STM32F722, with
LoRa telemetry, active canard roll control, and a single-file Web Serial
ground station.

- **[Project page](https://iamraging123.github.io/RocketNav/site/)** —
  architecture, filter design, verification status
- **[Ground station](https://iamraging123.github.io/RocketNav/viewer/)** —
  connect over Web Serial (Chrome, desktop) or press *Demo* to explore it
  without hardware
- `firmware/RocketNav/` — the sketch: hand-written drivers (ICM-45686,
  LIS3MDL, PCA9685, SX1278), ESKF, roll controller, telemetry
- `basestation/` — ESP32-C3 LoRa-to-USB bridge sketch
- `test/` — host-side test suites (build with `zig c++`, see each file's header)
- `DESIGN.md` / `SCHEMA.md` — design notes and the NDJSON telemetry contract

Build: `arduino-cli compile --fqbn
STMicroelectronics:stm32:GenF7:pnum=GENERIC_F722RETX .` from
`firmware/RocketNav/`.
