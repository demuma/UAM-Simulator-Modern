#pragma once

#include <glm/glm.hpp>

#include <string>
#include <vector>

namespace uam {

struct SceneVertex {
    glm::vec3 position{0.0f};
    glm::vec3 color{0.7f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    glm::vec2 texCoord{0.0f};
};

struct SceneMaterialRange {
    size_t firstVertex = 0;
    size_t vertexCount = 0;
    glm::vec3 diffuse{1.0f};
    std::string diffuseTexturePath;
    bool hasTexture = false;
};

struct SceneMesh {
    std::vector<SceneVertex> vertices;
    std::vector<SceneMaterialRange> materialRanges;
    glm::vec3 boundsMin{0.0f};
    glm::vec3 boundsMax{0.0f};
    bool hasBounds = false;
};

SceneMesh loadObjSceneMesh(const std::string& path, glm::vec3 baseColor, bool mirrorX, float scale = 1.0f);

} // namespace uam
