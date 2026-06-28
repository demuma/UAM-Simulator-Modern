#pragma once

#include "core/SceneMesh.hpp"

#include <glm/glm.hpp>

#include <vector>

namespace uam {

struct LidarHit {
    bool hit = false;
    float range = 0.0f;
    glm::vec3 point{0.0f};
    glm::vec3 dir{1.0f, 0.0f, 0.0f};
    int objectId = -1;
};

class SensorGeometry {
public:
    struct RayHit {
        bool hit = false;
        float t = 0.0f;
        int objectId = -1;
    };

    void clear();
    void addMesh(const SceneMesh& mesh, int objectId);
    void build();

    bool empty() const { return triangles_.empty(); }
    RayHit raycast(const glm::vec3& origin, const glm::vec3& dir, float maxRange) const;
    std::vector<LidarHit> simulateLidar(const glm::vec3& origin,
                                        float yawDeg,
                                        float pitchDeg,
                                        int beamsH,
                                        int beamsV,
                                        float fovHDeg,
                                        float fovVDeg,
                                        float maxRange) const;

public:
    struct Triangle {
        glm::vec3 a{0.0f};
        glm::vec3 b{0.0f};
        glm::vec3 c{0.0f};
        int objectId = -1;
    };

    struct BVHNode {
        glm::vec3 mn{0.0f};
        glm::vec3 mx{0.0f};
        int left = -1;
        int right = -1;
        std::size_t start = 0;
        std::size_t count = 0;
        bool leaf() const { return left < 0 && right < 0; }
    };

private:
    std::vector<Triangle> triangles_;
    std::vector<int> indices_;
    std::vector<BVHNode> bvh_;
    glm::vec3 boundsMin_{0.0f};
    glm::vec3 boundsMax_{0.0f};
    bool hasBounds_ = false;

    int buildRecursive(std::size_t start, std::size_t end);
};

} // namespace uam
