#ifndef FLIGHT_COMPUTER_H
#define FLIGHT_COMPUTER_H

#include <cstdint>
#include <cmath>

// ── Constants ──────────────────────────────────────────────

constexpr float GRAVITY     = 9.80665f;
constexpr float DEG_TO_RAD  = 0.01745329f;
constexpr float RAD_TO_DEG  = 57.29577951f;
constexpr float PI          = 3.14159265f;

// ── Aircraft state ─────────────────────────────────────────

struct AircraftState {
    float altitude_ft  = 0.0f;
    float airspeed_kt  = 0.0f;
    float vs_fpm       = 0.0f;   // vertical speed, feet per minute
    float pitch_deg    = 0.0f;
    float roll_deg     = 0.0f;
    float heading_deg  = 0.0f;
    bool  on_ground    = true;
};

// ── IMU sensor simulation ──────────────────────────────────

struct IMUReading {
    float ax, ay, az;   // accelerometer (m/s^2)
    float gx, gy, gz;   // gyroscope (rad/s)
    bool  valid;
};

class IMU {
public:
    IMU(float noise = 0.02f, uint32_t seed = 42);
    IMUReading sample(const AircraftState& truth);
    void setHealthy(bool h) { healthy_ = h; }

private:
    float noisy(float val);
    float rand_gauss();
    uint32_t lcg();

    float noise_;
    uint32_t rng_;
    bool healthy_ = true;
};

// ── Complementary filter (sensor fusion) ───────────────────

class AttitudeFilter {
public:
    AttitudeFilter(float alpha = 0.98f);

    // Returns filtered pitch/roll in degrees
    void update(const IMUReading& imu, float dt);
    float pitch() const { return pitch_deg_; }
    float roll() const  { return roll_deg_; }
    void reset();

private:
    float alpha_;
    float pitch_deg_ = 0.0f;
    float roll_deg_  = 0.0f;
    bool  init_      = false;
};

// ── PID controller ─────────────────────────────────────────

class PID {
public:
    PID(float kp, float ki, float kd, float min_out, float max_out);

    float update(float setpoint, float measured, float dt);
    void reset();

private:
    float kp_, ki_, kd_;
    float min_out_, max_out_;
    float integral_   = 0.0f;
    float prev_error_ = 0.0f;
    bool  first_      = true;
};

// ── Flight mode state machine ──────────────────────────────

enum class FlightMode {
    PREFLIGHT,
    TAKEOFF,
    CLIMB,
    CRUISE,
    DESCENT,
    LANDED,
    FAILSAFE
};

const char* mode_str(FlightMode m);

class FlightModeFSM {
public:
    struct Inputs {
        float altitude_ft  = 0.0f;
        float airspeed_kt  = 0.0f;
        float vs_fpm       = 0.0f;
        bool  on_ground    = true;
        bool  engine_on    = false;
        bool  fault        = false;
        float target_alt   = 10000.0f;
    };

    FlightMode update(const Inputs& in);
    FlightMode mode() const { return mode_; }
    bool just_transitioned() const { return mode_ != prev_; }

private:
    FlightMode mode_ = FlightMode::PREFLIGHT;
    FlightMode prev_ = FlightMode::PREFLIGHT;
};

#endif
