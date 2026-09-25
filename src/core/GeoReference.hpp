#pragma once

#include <glm/glm.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <string>
#include <vector>

namespace uam {
struct GeoReference {
    bool valid = false;
    glm::dvec3 origin{0.0}; // Easting, northing, source height.

    bool load(const std::string& path) {
        valid = false;
        try {
            auto node = YAML::LoadFile(path);
            if (node["horizontal_crs"].as<std::string>() != "EPSG:25832") return false;
            const auto axes = node["runtime_axes"].as<std::vector<std::string>>();
            if (axes != std::vector<std::string>{"west", "up", "north"}) return false;
            origin = {node["origin_easting"].as<double>(), node["origin_northing"].as<double>(),
                      node["origin_height"].as<double>()};
            valid = std::isfinite(origin.x) && std::isfinite(origin.y) && std::isfinite(origin.z);
        } catch (const YAML::Exception&) {}
        return valid;
    }

    glm::dvec3 toProjected(const glm::dvec3& local) const {
        return {origin.x - local.x, origin.y + local.z, origin.z + local.y};
    }
    glm::dvec3 toLocal(const glm::dvec3& projected) const {
        return {origin.x - projected.x, projected.z - origin.z, projected.y - origin.y};
    }
    bool matches(const GeoReference& other) const {
        return valid && other.valid && glm::length(origin - other.origin) < 1e-6;
    }
};
}
