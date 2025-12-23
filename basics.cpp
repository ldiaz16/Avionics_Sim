#include <iostream> 
#include <chrono>
#include <thread>

class StartButton {
public:
    int turnOn() {
        return 1; // Simulate button being pressed
    }
};

//custom data type (struct) for IMU data
struct IMUData {
    float pitch;
    float roll;
    float yaw;
    bool valid;
};

//custom data type (struct) for avionics status
struct avionicsStatus {
    int altitude;
    float speed;
    bool systemHealthy;
};

void initializeIMU(struct IMUData &imu);
void initializeAvionics(struct avionicsStatus &dashboard);
void printIMUData(const struct IMUData &imu);
void printDashboardStatus(const struct avionicsStatus &dashboard);
void updateIMU(struct IMUData &imu, float t_sec);
void updateAvionics(struct avionicsStatus &dashboard, float t_sec);

int main() {
    char pilotInput;
    StartButton startButton;
    IMUData imu;  
    avionicsStatus dashboard;

    std::cout << "Turn on Plane? (y/n)" << std::endl;
    std::cin >> pilotInput;

    if (pilotInput == 'y') {
        if (startButton.turnOn() == 1) {
            std::cout << "Plane is now ON" << std::endl;
            initializeIMU(imu);
            initializeAvionics(dashboard);
            constexpr int TICKS = 30;      // run 30 updates total
            constexpr int MS = 100;        // 100 ms per tick = 10 Hz
            float t = 0.0f;   
            
            for (int i = 0; i < TICKS; i++) {
                std::cout << "Tick " << i << std::endl;
                updateIMU(imu, t);
                updateAvionics(dashboard, t);

                printIMUData(imu);
                printDashboardStatus(dashboard);

                //now that we've updated and printed, wait for next tick
                std::this_thread::sleep_for(std::chrono::milliseconds(MS));
                t += 0.1f; // increment time by 0.1 seconds
            }
        } else {
            std::cout << "Failed to start the plane." << std::endl;
            return 1;
    }
    } else {
        std::cout << "Plane remains OFF" << std::endl;
    }

    printIMUData(imu);
    printDashboardStatus(dashboard);

    return 0;
}


void initializeIMU(IMUData &imu) {
    imu.pitch = 0.0f;
    imu.roll = 0.0f;
    imu.yaw = 0.0f;
    imu.valid = true;
}

void initializeAvionics(avionicsStatus &dashboard) {
    dashboard.altitude = 0;
    dashboard.speed = 0.0f;
    dashboard.systemHealthy = true;
}

void updateIMU(IMUData &imu, float t_sec) {
    imu.pitch = t_sec;
    imu.roll = 0.0f;
    imu.yaw = 0.0f;
    imu.valid = true;
}

void updateAvionics(avionicsStatus &dashboard, float t_sec) {
    dashboard.altitude += 50;   // add 50 feet each tick
    dashboard.speed += 5.0f;    // increase speed by 5 knots each tick
    dashboard.systemHealthy = true;
}

void printIMUData(const IMUData &imu) {
    std::cout << "IMU Data -> Pitch: " << imu.pitch 
              << ", Roll: " << imu.roll 
              << ", Yaw: " << imu.yaw 
              << ", Valid: " << (imu.valid ? "Yes" : "No") 
              << std::endl;
}

void printDashboardStatus(const avionicsStatus &dashboard) {
    std::cout << "Avionics Status -> Altitude: " << dashboard.altitude 
              << ", Speed: " << dashboard.speed 
              << ", System Healthy: " << (dashboard.systemHealthy ? "Yes" : "No") 
              << std::endl;
}