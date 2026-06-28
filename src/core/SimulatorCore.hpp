#pragma once

#include <glm/glm.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace uam {

struct Route {
    std::vector<glm::vec3> waypoints;
    std::size_t current = 0;
};

struct Drone {
    std::string name;
    glm::vec3 position{0.0f, 50.0f, 0.0f};
    glm::vec3 velocity{0.0f};
    glm::vec3 color{0.15f, 0.18f, 0.2f};
    float speed = 8.0f;
    float yawDeg = 0.0f;
    bool manual = false;
    Route route;
};

struct SensorConfig {
    int lidarBeamsH = 36;
    int lidarBeamsV = 15;
    float lidarFovH = 360.0f;
    float lidarFovV = 45.0f;
    float lidarMaxRange = 200.0f;
    float lidarFps = 10.0f;
    std::string lidarOutputDir = "lidar_output";
    int radarBeamsH = 60;
    int radarBeamsV = 8;
    float radarFovH = 90.0f;
    float radarFovV = 20.0f;
    float radarMaxRange = 200.0f;
    float radarMinRange = 2.0f;
    float radarSnr0 = 40.0f;
    float radarSnrMin = 8.0f;
    float radarFps = 10.0f;
    std::string radarOutputDir = "radar_output";
    unsigned cameraWidth = 640;
    unsigned cameraHeight = 360;
    float cameraFov = 60.0f;
    float cameraFps = 20.0f;
    float cameraExposure = 0.75f;
    float cameraContrast = 1.0f;
    float cameraSaturation = 1.0f;
    std::string cameraOutputDir = "camera_output";
};

class SimulatorCore {
public:
    bool load(const std::string& configPath, const std::string& sensorsPath);
    void update(float dt);
    void setDroneManual(std::size_t index, bool manual);
    bool droneManual(std::size_t index) const;
    void manualControlDrone(std::size_t index, float forward, float right, float up, float yawRateDeg, float dt);

    const std::vector<Drone>& drones() const { return drones_; }
    const SensorConfig& sensors() const { return sensors_; }

private:
    std::vector<Drone> drones_;
    SensorConfig sensors_;
};

} // namespace uam
