#include "core/SimulatorCore.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace uam {
namespace {

glm::vec3 readVec3(const YAML::Node& n, const glm::vec3& fallback) {
    if (!n || !n.IsSequence() || n.size() < 3) return fallback;
    return glm::vec3(n[0].as<float>(), n[1].as<float>(), n[2].as<float>());
}

std::vector<Drone> defaultDrones() {
    Drone a;
    a.name = "drone1";
    a.position = {-80.0f, 85.0f, -80.0f};
    a.color = {0.12f, 0.16f, 0.18f};
    a.speed = 10.0f;
    a.route.waypoints = {{-80.0f, 85.0f, -80.0f}, {80.0f, 85.0f, 80.0f}};

    Drone b;
    b.name = "drone2";
    b.position = {80.0f, 95.0f, -80.0f};
    b.color = {0.05f, 0.35f, 0.55f};
    b.speed = 8.0f;
    b.route.waypoints = {{80.0f, 95.0f, -80.0f}, {-80.0f, 95.0f, 80.0f}};

    return {a, b};
}

} // namespace

bool SimulatorCore::load(const std::string& configPath, const std::string& sensorsPath) {
    drones_.clear();

    try {
        YAML::Node root = YAML::LoadFile(configPath);
        if (root["drones"]) {
            int index = 1;
            for (const auto& node : root["drones"]) {
                Drone d;
                d.name = node["name"] ? node["name"].as<std::string>() : "drone" + std::to_string(index);
                d.position = readVec3(node["position"], d.position);
                d.color = readVec3(node["color"], d.color);
                if (node["speed"]) d.speed = node["speed"].as<float>();
                if (node["route"]) {
                    for (const auto& wp : node["route"]) {
                        d.route.waypoints.push_back(readVec3(wp, d.position));
                    }
                    if (!d.route.waypoints.empty() && !node["position"]) {
                        d.position = d.route.waypoints.front();
                    }
                }
                drones_.push_back(std::move(d));
                index++;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Config load failed: " << e.what() << "\n";
    }

    if (drones_.empty()) drones_ = defaultDrones();

    try {
        YAML::Node root = YAML::LoadFile(sensorsPath);
        if (root["lidar"]) {
            const auto n = root["lidar"];
            if (n["beamsH"]) sensors_.lidarBeamsH = n["beamsH"].as<int>();
            if (n["beamsV"]) sensors_.lidarBeamsV = n["beamsV"].as<int>();
            if (n["fovH_deg"]) sensors_.lidarFovH = n["fovH_deg"].as<float>();
            if (n["fovV_deg"]) sensors_.lidarFovV = n["fovV_deg"].as<float>();
            if (n["maxRange"]) sensors_.lidarMaxRange = n["maxRange"].as<float>();
            if (n["fps"]) sensors_.lidarFps = n["fps"].as<float>();
            if (n["outputDir"]) sensors_.lidarOutputDir = n["outputDir"].as<std::string>();
        }
        if (root["radar"]) {
            const auto n = root["radar"];
            if (n["beamsH"]) sensors_.radarBeamsH = n["beamsH"].as<int>();
            if (n["beamsV"]) sensors_.radarBeamsV = n["beamsV"].as<int>();
            if (n["fovH_deg"]) sensors_.radarFovH = n["fovH_deg"].as<float>();
            if (n["fovV_deg"]) sensors_.radarFovV = n["fovV_deg"].as<float>();
            if (n["maxRange"]) sensors_.radarMaxRange = n["maxRange"].as<float>();
            if (n["minRange"]) sensors_.radarMinRange = n["minRange"].as<float>();
            if (n["snr0"]) sensors_.radarSnr0 = n["snr0"].as<float>();
            if (n["snrMin"]) sensors_.radarSnrMin = n["snrMin"].as<float>();
            if (n["fps"]) sensors_.radarFps = n["fps"].as<float>();
            if (n["outputDir"]) sensors_.radarOutputDir = n["outputDir"].as<std::string>();
        }
        if (root["camera"]) {
            const auto n = root["camera"];
            if (n["width"]) sensors_.cameraWidth = n["width"].as<unsigned>();
            if (n["height"]) sensors_.cameraHeight = n["height"].as<unsigned>();
            if (n["fov_deg"]) sensors_.cameraFov = n["fov_deg"].as<float>();
            if (n["fps"]) sensors_.cameraFps = n["fps"].as<float>();
            if (n["exposure"]) sensors_.cameraExposure = n["exposure"].as<float>();
            if (n["contrast"]) sensors_.cameraContrast = n["contrast"].as<float>();
            if (n["saturation"]) sensors_.cameraSaturation = n["saturation"].as<float>();
            if (n["outputDir"]) sensors_.cameraOutputDir = n["outputDir"].as<std::string>();
        }
    } catch (const std::exception& e) {
        std::cerr << "Sensor config load failed: " << e.what() << "\n";
    }

    std::cout << "Modern simulator loaded " << drones_.size() << " drones\n";
    return true;
}

void SimulatorCore::update(float dt) {
    for (auto& drone : drones_) {
        if (drone.manual) {
            continue;
        }

        if (drone.route.waypoints.empty()) {
            drone.velocity = glm::vec3(0.0f);
            continue;
        }

        if (drone.route.current >= drone.route.waypoints.size()) drone.route.current = 0;
        glm::vec3 target = drone.route.waypoints[drone.route.current];
        glm::vec3 delta = target - drone.position;
        float dist = glm::length(delta);
        if (dist < 0.5f) {
            drone.route.current = (drone.route.current + 1) % drone.route.waypoints.size();
            target = drone.route.waypoints[drone.route.current];
            delta = target - drone.position;
            dist = glm::length(delta);
        }

        if (dist < 1e-4f) {
            drone.velocity = glm::vec3(0.0f);
            continue;
        }

        glm::vec3 dir = delta / dist;
        float step = std::min(drone.speed * dt, dist);
        drone.position += dir * step;
        drone.velocity = dir * (step / std::max(dt, 1e-4f));
        drone.yawDeg = glm::degrees(std::atan2(dir.z, dir.x));
    }
}


void SimulatorCore::setDroneManual(std::size_t index, bool manual) {
    if (index >= drones_.size()) return;
    drones_[index].manual = manual;
    if (manual) drones_[index].velocity = glm::vec3(0.0f);
}

bool SimulatorCore::droneManual(std::size_t index) const {
    return index < drones_.size() && drones_[index].manual;
}

void SimulatorCore::manualControlDrone(std::size_t index, float forward, float right, float up, float yawRateDeg, float dt) {
    if (index >= drones_.size() || dt <= 0.0f) return;

    Drone& drone = drones_[index];
    if (!drone.manual) return;

    drone.yawDeg += yawRateDeg * dt;
    if (drone.yawDeg > 180.0f) drone.yawDeg -= 360.0f;
    if (drone.yawDeg < -180.0f) drone.yawDeg += 360.0f;

    float yaw = glm::radians(drone.yawDeg);
    glm::vec3 fwd(std::cos(yaw), 0.0f, std::sin(yaw));
    glm::vec3 rgt(-std::sin(yaw), 0.0f, std::cos(yaw));
    glm::vec3 move = fwd * forward + rgt * right + glm::vec3(0.0f, up, 0.0f);

    if (glm::length(move) > 1e-5f) {
        move = glm::normalize(move);
        float speed = std::max(drone.speed * 2.0f, 12.0f);
        drone.position += move * speed * dt;
        drone.velocity = move * speed;
    } else {
        drone.velocity = glm::vec3(0.0f);
    }
}

} // namespace uam
