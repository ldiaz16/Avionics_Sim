#include "flight_computer.h"
#include <algorithm>

// ── IMU ────────────────────────────────────────────────────

IMU::IMU(float noise, uint32_t seed)
    : noise_(noise), rng_(seed) {}

uint32_t IMU::lcg() {
    rng_ = rng_ * 1664525u + 1013904223u;
    return rng_;
}

float IMU::rand_gauss() {
    float u1 = (lcg() + 1u) / 4294967296.0f;
    float u2 = lcg() / 4294967296.0f;
    return std::sqrt(-2.0f * std::log(u1)) * std::cos(2.0f * PI * u2);
}

float IMU::noisy(float val) {
    return val + noise_ * rand_gauss();
}

IMUReading IMU::sample(const AircraftState& truth) {
    IMUReading r{};
    r.valid = healthy_;
    if (!healthy_) return r;

    float p = truth.pitch_deg * DEG_TO_RAD;
    float rl = truth.roll_deg * DEG_TO_RAD;

    // Gravity components projected through body frame
    r.ax = noisy(-GRAVITY * std::sin(p));
    r.ay = noisy( GRAVITY * std::sin(rl) * std::cos(p));
    r.az = noisy(-GRAVITY * std::cos(rl) * std::cos(p));

    r.gx = noisy(0.0f);
    r.gy = noisy(0.0f);
    r.gz = noisy(0.0f);
    return r;
}

// ── Complementary filter ───────────────────────────────────

AttitudeFilter::AttitudeFilter(float alpha) : alpha_(alpha) {}

void AttitudeFilter::update(const IMUReading& imu, float dt) {
    if (!imu.valid) return;

    // Pitch & roll from accelerometer
    float ap = std::atan2(-imu.ax, std::sqrt(imu.ay*imu.ay + imu.az*imu.az)) * RAD_TO_DEG;
    float ar = std::atan2(imu.ay, -imu.az) * RAD_TO_DEG;

    if (!init_) {
        pitch_deg_ = ap;
        roll_deg_ = ar;
        init_ = true;
    } else {
        // High-pass gyro + low-pass accel
        pitch_deg_ = alpha_ * (pitch_deg_ + imu.gy * dt * RAD_TO_DEG)
                   + (1.0f - alpha_) * ap;
        roll_deg_  = alpha_ * (roll_deg_ + imu.gx * dt * RAD_TO_DEG)
                   + (1.0f - alpha_) * ar;
    }
}

void AttitudeFilter::reset() {
    pitch_deg_ = 0.0f;
    roll_deg_ = 0.0f;
    init_ = false;
}

// ── PID ────────────────────────────────────────────────────

PID::PID(float kp, float ki, float kd, float min_out, float max_out)
    : kp_(kp), ki_(ki), kd_(kd), min_out_(min_out), max_out_(max_out) {}

float PID::update(float setpoint, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float err = setpoint - measured;
    integral_ += err * dt;
    integral_ = std::clamp(integral_, min_out_ / (ki_ + 0.001f),
                                       max_out_ / (ki_ + 0.001f));

    float deriv = 0.0f;
    if (!first_) {
        deriv = (err - prev_error_) / dt;
    }
    first_ = false;
    prev_error_ = err;

    float out = kp_ * err + ki_ * integral_ + kd_ * deriv;
    return std::clamp(out, min_out_, max_out_);
}

void PID::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    first_ = true;
}

// ── Flight mode FSM ───────────────────────────────────────

const char* mode_str(FlightMode m) {
    switch (m) {
        case FlightMode::PREFLIGHT: return "PREFLIGHT";
        case FlightMode::TAKEOFF:   return "TAKEOFF";
        case FlightMode::CLIMB:     return "CLIMB";
        case FlightMode::CRUISE:    return "CRUISE";
        case FlightMode::DESCENT:   return "DESCENT";
        case FlightMode::LANDED:    return "LANDED";
        case FlightMode::FAILSAFE:  return "FAILSAFE";
    }
    return "UNKNOWN";
}

FlightMode FlightModeFSM::update(const Inputs& in) {
    prev_ = mode_;

    if (in.fault) {
        mode_ = FlightMode::FAILSAFE;
        return mode_;
    }

    switch (mode_) {
    case FlightMode::PREFLIGHT:
        if (in.engine_on && in.airspeed_kt > 60.0f)
            mode_ = FlightMode::TAKEOFF;
        break;

    case FlightMode::TAKEOFF:
        if (!in.on_ground && in.altitude_ft > 50.0f)
            mode_ = FlightMode::CLIMB;
        break;

    case FlightMode::CLIMB:
        if (in.altitude_ft >= in.target_alt * 0.95f && in.vs_fpm < 200.0f)
            mode_ = FlightMode::CRUISE;
        break;

    case FlightMode::CRUISE:
        if (in.vs_fpm < -300.0f)
            mode_ = FlightMode::DESCENT;
        break;

    case FlightMode::DESCENT:
        if (in.on_ground && in.airspeed_kt < 50.0f)
            mode_ = FlightMode::LANDED;
        break;

    case FlightMode::LANDED:
        break;

    case FlightMode::FAILSAFE:
        if (!in.fault && in.on_ground)
            mode_ = FlightMode::LANDED;
        break;
    }

    return mode_;
}
