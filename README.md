# Flight Computer Simulation

A small C++17 project simulating a basic flight computer with sensor processing,
attitude estimation, and autopilot control. No external dependencies.

## What it does

Runs a 120-second flight simulation with:
- **IMU sensor model** — simulated accelerometer + gyroscope with noise (uses a deterministic LCG, no heap)
- **Complementary filter** — fuses gyro (high-freq) and accel (low-freq) to estimate pitch/roll
- **PID controller** — altitude hold via pitch command with anti-windup
- **Flight mode state machine** — PREFLIGHT → TAKEOFF → CLIMB → CRUISE → DESCENT → LANDED (+ FAILSAFE)

## Build

```
make          # build sim
make run      # build and run
make test     # build and run tests (17 tests)
make clean
```

## Files

```
flight_computer.h    — all types and class declarations
flight_computer.cpp  — implementations
main.cpp             — simulation loop
tests.cpp            — unit tests
```
