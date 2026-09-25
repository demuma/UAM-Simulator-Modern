#include "core/SensorGeometry.hpp"
#include "core/SimulatorCore.hpp"

#include <cmath>
#include <future>
#include <iostream>
#include <stdexcept>

static void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

int main() {
    try {
        uam::SceneMesh mesh;
        mesh.vertices = {{{0, 0, 0}}, {{10, 0, 0}}, {{0, 0, 10}}};
        uam::SensorGeometry geometry;
        geometry.addMesh(mesh, 42);
        geometry.build();
        auto checkRays = [&geometry] {
            for (int i = 0; i < 1000; ++i) {
                auto hit = geometry.raycast({2, 5, 2}, {0, -1, 0}, 10);
                require(hit.hit && hit.objectId == 42 && std::abs(hit.t - 5) < 1e-5f,
                        "Ray must hit a zero-thickness ground BVH bound");
                require(!geometry.raycast({12, 5, 2}, {0, -1, 0}, 10).hit,
                        "Parallel ray outside slab must miss");
                require(!geometry.raycast({2, 5, 2}, {0, -1, 0}, 4).hit,
                        "Range limit must be respected");
                require(geometry.raycast({0, 5, 0}, {0, -1, 0}, 10).hit,
                        "Ray at triangle vertex must not be culled by bounds");
            }
        };
        auto worker = std::async(std::launch::async, checkRays);
        checkRays();
        worker.get();

        uam::SimulatorCore sim;
        require(sim.load("config.yaml", "sensors.yaml") && !sim.drones().empty(), "Load fixture config");
        sim.setDroneManual(0, true);
        auto start = sim.drones()[0].position;
        sim.update(0.1f);
        require(glm::length(sim.drones()[0].position - start) == 0, "Manual drone must pause route");
        sim.manualControlDrone(0, 1, 0, 0, 0, 0.1f);
        require(glm::length(sim.drones()[0].position - start) > 0, "Manual movement must work");
        sim.manualControlDrone(0, 0, 0, 0, 0, 0.1f);
        require(glm::length(sim.drones()[0].velocity) == 0, "Releasing controls must clear velocity");
        sim.setDroneManual(0, false);
        start = sim.drones()[0].position;
        sim.update(0.1f);
        require(glm::length(sim.drones()[0].position - start) > 0, "Route must resume");
        std::cout << "Core regression checks passed\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
