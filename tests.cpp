#include "flight_computer.h"
#include <cstdio>
#include <cmath>

static int run = 0, fail = 0;

#define CHECK(expr) do { run++; \
    if (!(expr)) { fail++; std::printf("  FAIL: %s (%s:%d)\n", #expr, __FILE__, __LINE__); } \
} while(0)

#define CHECK_NEAR(a, b, tol) do { run++; \
    if (std::fabs((a)-(b)) > (tol)) { \
        fail++; std::printf("  FAIL: %s ~= %s (got %f vs %f) (%s:%d)\n", \
                            #a, #b, (double)(a), (double)(b), __FILE__, __LINE__); \
    } } while(0)

#define TEST(name) static void name(); \
    static struct name##_reg { name##_reg() { \
        std::printf("  %-45s", #name); int bf = fail; name(); \
        std::printf("%s\n", fail == bf ? "OK" : ""); \
    }} name##_instance; \
    static void name()

// ── IMU tests ──────────────────────────────────────────────

TEST(imu_returns_valid_reading) {
    IMU imu;
    AircraftState s;
    auto r = imu.sample(s);
    CHECK(r.valid);
    // Level flight: az should be close to -g
    CHECK_NEAR(r.az, -GRAVITY, 0.5f);
}

TEST(imu_invalid_when_unhealthy) {
    IMU imu;
    imu.setHealthy(false);
    auto r = imu.sample(AircraftState{});
    CHECK(!r.valid);
}

TEST(imu_pitch_affects_accel) {
    IMU imu(0.0f);  // no noise
    AircraftState s;
    s.pitch_deg = 30.0f;
    auto r = imu.sample(s);
    // With 30° pitch, ax should be roughly -g*sin(30°) = -4.9
    CHECK_NEAR(r.ax, -GRAVITY * std::sin(30.0f * DEG_TO_RAD), 0.1f);
}

// ── Complementary filter tests ─────────────────────────────

TEST(filter_converges_to_true_pitch) {
    IMU imu(0.01f);
    AttitudeFilter f(0.98f);
    AircraftState s;
    s.pitch_deg = 15.0f;

    for (int i = 0; i < 300; i++) {
        f.update(imu.sample(s), 0.01f);
    }
    CHECK_NEAR(f.pitch(), 15.0f, 2.0f);
}

TEST(filter_reset_zeroes_state) {
    AttitudeFilter f;
    IMU imu(0.0f);
    AircraftState s;
    s.pitch_deg = 10.0f;

    f.update(imu.sample(s), 0.01f);
    f.reset();
    CHECK_NEAR(f.pitch(), 0.0f, 0.001f);
}

// ── PID tests ──────────────────────────────────────────────

TEST(pid_proportional_response) {
    PID pid(1.0f, 0.0f, 0.0f, -100.0f, 100.0f);
    float out = pid.update(10.0f, 0.0f, 0.1f);
    CHECK_NEAR(out, 10.0f, 0.01f);
}

TEST(pid_output_is_clamped) {
    PID pid(10.0f, 0.0f, 0.0f, -5.0f, 5.0f);
    float out = pid.update(100.0f, 0.0f, 0.1f);
    CHECK_NEAR(out, 5.0f, 0.01f);
}

TEST(pid_integral_accumulates) {
    PID pid(0.0f, 1.0f, 0.0f, -100.0f, 100.0f);
    pid.update(10.0f, 0.0f, 1.0f);   // integral = 10
    float out = pid.update(10.0f, 0.0f, 1.0f);  // integral = 20
    CHECK_NEAR(out, 20.0f, 0.1f);
}

TEST(pid_reset_clears_state) {
    PID pid(1.0f, 1.0f, 0.0f, -100.0f, 100.0f);
    pid.update(10.0f, 0.0f, 1.0f);
    pid.reset();
    // After reset, same input should give pure proportional response
    float out = pid.update(5.0f, 0.0f, 1.0f);
    CHECK_NEAR(out, 5.0f + 5.0f * 1.0f, 0.1f);  // kp*err + ki*err*dt
}

// ── FSM tests ──────────────────────────────────────────────

TEST(fsm_starts_in_preflight) {
    FlightModeFSM fsm;
    CHECK(fsm.mode() == FlightMode::PREFLIGHT);
}

TEST(fsm_takeoff_on_speed) {
    FlightModeFSM fsm;
    FlightModeFSM::Inputs in;
    in.engine_on = true;
    in.airspeed_kt = 70.0f;
    in.on_ground = true;
    fsm.update(in);
    CHECK(fsm.mode() == FlightMode::TAKEOFF);
}

TEST(fsm_climb_after_liftoff) {
    FlightModeFSM fsm;
    FlightModeFSM::Inputs in;
    in.engine_on = true;

    in.airspeed_kt = 70.0f;
    fsm.update(in);  // -> TAKEOFF

    in.on_ground = false;
    in.altitude_ft = 100.0f;
    fsm.update(in);  // -> CLIMB
    CHECK(fsm.mode() == FlightMode::CLIMB);
}

TEST(fsm_cruise_at_altitude) {
    FlightModeFSM fsm;
    FlightModeFSM::Inputs in;
    in.engine_on = true;
    in.target_alt = 10000.0f;

    in.airspeed_kt = 70.0f;
    fsm.update(in);  // TAKEOFF
    in.on_ground = false;
    in.altitude_ft = 100.0f;
    fsm.update(in);  // CLIMB

    in.altitude_ft = 9600.0f;
    in.vs_fpm = 100.0f;
    fsm.update(in);  // CRUISE
    CHECK(fsm.mode() == FlightMode::CRUISE);
}

TEST(fsm_failsafe_on_fault) {
    FlightModeFSM fsm;
    FlightModeFSM::Inputs in;
    in.fault = true;
    fsm.update(in);
    CHECK(fsm.mode() == FlightMode::FAILSAFE);
}

TEST(fsm_descent_from_cruise) {
    FlightModeFSM fsm;
    FlightModeFSM::Inputs in;
    in.engine_on = true;
    in.target_alt = 1000.0f;

    // Fast-forward to CRUISE
    in.airspeed_kt = 70.0f; fsm.update(in);
    in.on_ground = false; in.altitude_ft = 100.0f; fsm.update(in);
    in.altitude_ft = 960.0f; in.vs_fpm = 50.0f; fsm.update(in);
    CHECK(fsm.mode() == FlightMode::CRUISE);

    in.vs_fpm = -500.0f;
    fsm.update(in);
    CHECK(fsm.mode() == FlightMode::DESCENT);
}

// ── Runner ─────────────────────────────────────────────────

int main() {
    std::printf("=== Flight Computer Tests ===\n\n");
    // Tests auto-register and run via static init above
    std::printf("\n%d/%d passed\n", run - fail, run);
    return fail > 0 ? 1 : 0;
}
