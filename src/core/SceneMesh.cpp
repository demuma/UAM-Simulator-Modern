#include "core/SceneMesh.hpp"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <unordered_map>

namespace uam {
namespace {

glm::vec3 readPosition(const tinyobj::attrib_t& attrib, int vertexIndex, bool mirrorX, float scale) {
    const size_t i = static_cast<size_t>(vertexIndex) * 3;
    glm::vec3 p(attrib.vertices[i + 0], attrib.vertices[i + 1], attrib.vertices[i + 2]);
    if (mirrorX) p.x = -p.x;
    return p * scale;
}

glm::vec3 readNormal(const tinyobj::attrib_t& attrib, int normalIndex, bool mirrorX) {
    if (normalIndex < 0) return glm::vec3(0.0f, 1.0f, 0.0f);
    const size_t i = static_cast<size_t>(normalIndex) * 3;
    glm::vec3 n(attrib.normals[i + 0], attrib.normals[i + 1], attrib.normals[i + 2]);
    if (mirrorX) n.x = -n.x;
    float len = glm::length(n);
    return len > 1e-5f ? n / len : glm::vec3(0.0f, 1.0f, 0.0f);
}

glm::vec2 readTexCoord(const tinyobj::attrib_t& attrib, int texcoordIndex) {
    if (texcoordIndex < 0) return glm::vec2(0.0f);
    const size_t i = static_cast<size_t>(texcoordIndex) * 2;
    return glm::vec2(attrib.texcoords[i + 0], attrib.texcoords[i + 1]);
}

struct MaterialFallback {
    std::string name;
    glm::vec3 diffuse{1.0f};
    std::string diffuseTex;
};

bool loadMtlFallback(const std::string& mtlPath,
                     std::vector<MaterialFallback>& materials,
                     std::unordered_map<std::string, int>& nameToId) {
    std::ifstream in(mtlPath);
    if (!in.is_open()) return false;

    MaterialFallback current;
    bool hasCurrent = false;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        std::string tag;
        iss >> tag;
        if (tag == "newmtl") {
            if (hasCurrent) {
                nameToId[current.name] = static_cast<int>(materials.size());
                materials.push_back(current);
            }
            current = MaterialFallback{};
            iss >> current.name;
            hasCurrent = true;
        } else if (tag == "Kd") {
            iss >> current.diffuse.r >> current.diffuse.g >> current.diffuse.b;
        } else if (tag == "map_Kd") {
            iss >> current.diffuseTex;
        }
    }
    if (hasCurrent) {
        nameToId[current.name] = static_cast<int>(materials.size());
        materials.push_back(current);
    }
    return !materials.empty();
}

std::string readMtllibPath(const std::string& objPath, const std::string& baseDir) {
    std::ifstream in(objPath);
    if (!in.is_open()) return {};

    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("mtllib ", 0) == 0) {
            return baseDir + line.substr(7);
        }
    }
    return {};
}

std::vector<int> parseObjFaceMaterialIds(const std::string& objPath,
                                         const std::unordered_map<std::string, int>& nameToId) {
    std::ifstream in(objPath);
    if (!in.is_open()) return {};

    std::vector<int> ids;
    std::string line;
    int currentId = -1;
    while (std::getline(in, line)) {
        if (line.rfind("usemtl ", 0) == 0) {
            std::string name = line.substr(7);
            auto it = nameToId.find(name);
            currentId = it != nameToId.end() ? it->second : -1;
        } else if (line.rfind("f ", 0) == 0) {
            ids.push_back(currentId);
        }
    }
    return ids;
}

} // namespace

SceneMesh loadObjSceneMesh(const std::string& path, glm::vec3 baseColor, bool mirrorX, float scale) {
    SceneMesh mesh;
    if (!std::filesystem::exists(path)) {
        std::cerr << "Scene mesh missing: " << path << "\n";
        return mesh;
    }

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;

    std::filesystem::path objPath(path);
    std::string baseDir = objPath.parent_path().string();
    if (!baseDir.empty()) baseDir += "/";

    bool ok = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                               path.c_str(), baseDir.c_str(), true);
    if (!warn.empty()) std::cerr << "OBJ warn: " << warn << "\n";
    if (!err.empty()) std::cerr << "OBJ err: " << err << "\n";
    if (!ok) {
        std::cerr << "OBJ load failed: " << path << "\n";
        return mesh;
    }

    std::vector<MaterialFallback> materialsFallback;
    std::unordered_map<std::string, int> materialNameToId;
    std::vector<int> faceMaterialIds;
    std::string mtlPath = readMtllibPath(path, baseDir);
    if (!mtlPath.empty() && loadMtlFallback(mtlPath, materialsFallback, materialNameToId)) {
        faceMaterialIds = parseObjFaceMaterialIds(path, materialNameToId);
        std::cout << "Scene materials: " << materialsFallback.size() << " from " << mtlPath << "\n";
    }

    glm::vec3 mn(std::numeric_limits<float>::max());
    glm::vec3 mx(std::numeric_limits<float>::lowest());
    std::map<int, std::vector<SceneVertex>> verticesByMaterial;

    size_t globalFace = 0;
    for (const auto& shape : shapes) {
        size_t indexOffset = 0;
        for (size_t face = 0; face < shape.mesh.num_face_vertices.size(); ++face) {
            int fv = shape.mesh.num_face_vertices[face];
            int matId = -1;
            if (!faceMaterialIds.empty() && globalFace < faceMaterialIds.size()) {
                matId = faceMaterialIds[globalFace];
            }
            if (fv != 3) {
                indexOffset += static_cast<size_t>(fv);
                ++globalFace;
                continue;
            }

            glm::vec3 materialColor = baseColor;
            if (matId >= 0 && static_cast<size_t>(matId) < materialsFallback.size()) {
                materialColor = materialsFallback[static_cast<size_t>(matId)].diffuse;
            }
            auto& vertices = verticesByMaterial[matId];
            for (int v = 0; v < 3; ++v) {
                const tinyobj::index_t idx = shape.mesh.indices[indexOffset + static_cast<size_t>(v)];
                if (idx.vertex_index < 0) continue;

                glm::vec3 p = readPosition(attrib, idx.vertex_index, mirrorX, scale);
                glm::vec3 n = readNormal(attrib, idx.normal_index, mirrorX);
                glm::vec2 uv = readTexCoord(attrib, idx.texcoord_index);
                glm::vec3 color = glm::clamp(materialColor, glm::vec3(0.0f), glm::vec3(1.0f));

                vertices.push_back({p, color, n, uv});
                mn = glm::min(mn, p);
                mx = glm::max(mx, p);
            }
            indexOffset += 3;
            ++globalFace;
        }
    }

    for (const auto& [matId, vertices] : verticesByMaterial) {
        if (vertices.empty()) continue;
        SceneMaterialRange range;
        range.firstVertex = mesh.vertices.size();
        range.vertexCount = vertices.size();
        if (matId >= 0 && static_cast<size_t>(matId) < materialsFallback.size()) {
            const auto& mat = materialsFallback[static_cast<size_t>(matId)];
            range.diffuse = mat.diffuse;
            if (!mat.diffuseTex.empty()) {
                std::filesystem::path texPath = std::filesystem::path(baseDir) / mat.diffuseTex;
                range.diffuseTexturePath = texPath.string();
                range.hasTexture = true;
            }
        }
        mesh.vertices.insert(mesh.vertices.end(), vertices.begin(), vertices.end());
        mesh.materialRanges.push_back(std::move(range));
    }

    if (!mesh.vertices.empty()) {
        mesh.boundsMin = mn;
        mesh.boundsMax = mx;
        mesh.hasBounds = true;
    }

    std::cout << "Loaded scene mesh " << path << ": " << mesh.vertices.size() / 3 << " triangles\n";
    if (mesh.hasBounds) {
        std::cout << "Mesh bounds min(" << mesh.boundsMin.x << ", " << mesh.boundsMin.y << ", " << mesh.boundsMin.z
                  << ") max(" << mesh.boundsMax.x << ", " << mesh.boundsMax.y << ", " << mesh.boundsMax.z << ")\n";
    }
    return mesh;
}

} // namespace uam
