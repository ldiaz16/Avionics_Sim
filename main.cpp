#include "flight_computer.h"
#include <cstdio>

// Bare-bones aircraft dynamics for the sim loop.
// Not meant to be realistic — just enough to exercise the flight computer.
struct SimAircraft {
    AircraftState state;
    bool engine_on = false;

    void step(float pitch_cmd, float dt) {
        if (!engine_on) return;

        // Pitch follows command with lag
        state.pitch_deg += (pitch_cmd - state.pitch_deg) * 0.5f * dt * 10.0f;

        // VS from pitch
        float target_vs = state.pitch_deg * 200.0f;
        state.vs_fpm += (target_vs - state.vs_fpm) * 0.1f;

        // Integrate altitude
        state.altitude_ft += state.vs_fpm * (dt / 60.0f);
        if (state.altitude_ft < 0.0f) state.altitude_ft = 0.0f;

        state.on_ground = (state.altitude_ft < 1.0f);
    }
};

int main() {
    std::printf("=== Flight Computer Sim ===\n\n");

    SimAircraft ac;
    ac.engine_on = true;
    ac.state.heading_deg = 270.0f;

    IMU imu(0.03f);
    AttitudeFilter filter(0.98f);
    FlightModeFSM fsm;

    // PID: altitude error -> pitch command (degrees)
    PID alt_pid(0.005f, 0.0005f, 0.002f, -10.0f, 15.0f);
    float target_alt = 5000.0f;

    constexpr float dt = 0.1f;
    constexpr float duration = 120.0f;
    float t = 0.0f;

    // Scripted airspeed ramp (not part of the flight computer — just drives the sim)
    auto airspeed = [](float t) -> float {
        if (t < 8.0f) return t * 12.0f;       // accelerate on runway
        if (t < 40.0f) return 100.0f + t;      // climb speed
        return 140.0f;                          // cruise
    };

    while (t < duration) {
        ac.state.airspeed_kt = airspeed(t);

        // --- Flight computer starts here ---

        // 1. Read sensors
        IMUReading reading = imu.sample(ac.state);
        filter.update(reading, dt);

        // 2. Update flight mode
        FlightModeFSM::Inputs fi;
        fi.altitude_ft = ac.state.altitude_ft;
        fi.airspeed_kt = ac.state.airspeed_kt;
        fi.vs_fpm      = ac.state.vs_fpm;
        fi.on_ground   = ac.state.on_ground;
        fi.engine_on   = ac.engine_on;
        fi.fault       = false;
        fi.target_alt  = target_alt;
        fsm.update(fi);

        // 3. Compute pitch command
        float pitch_cmd = 0.0f;
        FlightMode mode = fsm.mode();

        if (mode == FlightMode::TAKEOFF) {
            pitch_cmd = 8.0f;   // fixed rotation pitch
        } else if (mode == FlightMode::CLIMB || mode == FlightMode::CRUISE) {
            pitch_cmd = alt_pid.update(target_alt, ac.state.altitude_ft, dt);
        }

        // --- Flight computer ends here ---

        // Apply to sim
        ac.step(pitch_cmd, dt);

        // Print on transitions
        if (fsm.just_transitioned()) {
            std::printf("[%5.1fs] Mode -> %s\n", t, mode_str(mode));
        }

        // Status every 10 seconds
        if (static_cast<int>(t * 10) % 100 == 0 && t > 0.0f) {
            std::printf("[%5.1fs] %-10s  alt=%7.0f ft  vs=%+6.0f fpm  "
                        "pitch=%+5.1f°  filter_pitch=%+5.1f°\n",
                        t, mode_str(mode),
                        ac.state.altitude_ft, ac.state.vs_fpm,
                        ac.state.pitch_deg, filter.pitch());
        }

        t += dt;
    }

    std::printf("\n[Done] Final alt: %.0f ft  Mode: %s\n",
                ac.state.altitude_ft, mode_str(fsm.mode()));
    return 0;
}
