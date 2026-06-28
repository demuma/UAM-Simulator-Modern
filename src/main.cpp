// UAM Simulator — modularized main with comments and clearer structure

#include <GL/glew.h>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ===================== Mesh / OBJ =====================
struct Vertex {
    glm::vec3 pos{};
    glm::vec3 normal{0.f, 1.f, 0.f};
    glm::vec2 uv{0.f, 0.f};
};

struct Mesh {
    GLuint vao = 0;
    GLuint vbo = 0;
    size_t vertexCount = 0;
    std::vector<Vertex> cpuVertices;
    glm::vec3 boundsMin{0.f};
    glm::vec3 boundsMax{0.f};
    bool hasBounds = false;

    bool valid() const { return vao != 0 && vbo != 0 && vertexCount > 0; }

    void destroy() {
        if (vao) glDeleteVertexArrays(1, &vao);
        if (vbo) glDeleteBuffers(1, &vbo);
        vao = 0; vbo = 0; vertexCount = 0;
        cpuVertices.clear();
    }
};

struct ColoredMesh {
    Mesh mesh;
    glm::vec3 color{0.7f, 0.7f, 0.7f};
    GLuint textureId = 0;
    bool hasTexture = false;
};

struct CityModel {
    std::vector<ColoredMesh> parts;
    glm::vec3 boundsMin{0.f};
    glm::vec3 boundsMax{0.f};
    bool hasBounds = false;
    std::optional<glm::dvec2> utmCenter;
    float sourceHalfExtent = 0.0f;

    bool valid() const { return !parts.empty(); }

    void destroy() {
        for (auto& p : parts) p.mesh.destroy();
        for (auto& p : parts) {
            if (p.textureId) glDeleteTextures(1, &p.textureId);
        }
        parts.clear();
        hasBounds = false;
        utmCenter.reset();
        sourceHalfExtent = 0.0f;
    }
};

struct GeoReference {
    bool valid = false;
    glm::dvec2 utmCenter{0.0, 0.0};
    std::string crs = "EPSG:25832";
    glm::mat4 mapLocalToSim{1.0f};
    glm::mat4 simToMapLocal{1.0f};

    glm::dvec2 simToWorld(const glm::vec3& p) const {
        glm::vec3 local = glm::vec3(simToMapLocal * glm::vec4(p, 1.0f));
        return utmCenter + glm::dvec2(static_cast<double>(local.x), static_cast<double>(local.z));
    }

    glm::vec3 worldToSim(double easting, double northing, float altitudeMeters) const {
        glm::vec3 mapLocal(static_cast<float>(easting - utmCenter.x),
                           altitudeMeters,
                           static_cast<float>(northing - utmCenter.y));
        return glm::vec3(mapLocalToSim * glm::vec4(mapLocal, 1.0f));
    }
};

static std::optional<GeoReference> readGeoReferenceFromObj(const std::string& objPath) {
    std::ifstream in(objPath);
    if (!in.is_open()) return std::nullopt;

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (line[0] != '#') break;
        auto centerEPos = line.find("centerE ");
        auto centerNPos = line.find("centerN ");
        if (centerEPos == std::string::npos || centerNPos == std::string::npos) continue;

        std::istringstream iss(line.substr(1));
        std::string token;
        GeoReference ref;
        ref.valid = true;
        while (iss >> token) {
            if (token == "centerE") iss >> ref.utmCenter.x;
            else if (token == "centerN") iss >> ref.utmCenter.y;
            else {
                double ignored = 0.0;
                if (token == "half") iss >> ignored;
            }
        }
        return ref;
    }
    return std::nullopt;
}

struct MaterialFallback {
    std::string name;
    glm::vec3 diffuse{1.f, 1.f, 1.f};
    std::string diffuseTex;
};

static bool loadMtlFallback(const std::string& mtlPath,
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
            std::string tex;
            iss >> tex;
            current.diffuseTex = tex;
        }
    }
    if (hasCurrent) {
        nameToId[current.name] = static_cast<int>(materials.size());
        materials.push_back(current);
    }
    return !materials.empty();
}

static std::vector<int> parseObjFaceMaterialIds(const std::string& objPath,
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
            currentId = (it != nameToId.end()) ? it->second : -1;
        } else if (line.rfind("f ", 0) == 0) {
            ids.push_back(currentId);
        }
    }
    return ids;
}

static Mesh loadObjMesh(const std::string& path, float scale) {
    Mesh mesh;

    if (!std::filesystem::exists(path)) {
        std::cerr << "OBJ missing: " << path << "\n";
        return mesh;
    }

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    std::filesystem::path p(path);
    std::string baseDir = p.parent_path().string();
    if (!baseDir.empty()) baseDir += "/";

    bool ok = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                               path.c_str(), baseDir.c_str(), true);
    if (!warn.empty()) std::cerr << "OBJ warn: " << warn << "\n";
    if (!err.empty())  std::cerr << "OBJ err: " << err << "\n";
    if (!ok) {
        std::cerr << "OBJ load failed: " << path << "\n";
        return mesh;
    }

    std::vector<Vertex> vertices;
    size_t totalIdx = 0;
    for (const auto& s : shapes) totalIdx += s.mesh.indices.size();
    vertices.reserve(totalIdx);

    for (const auto& shape : shapes) {
        for (const auto& idx : shape.mesh.indices) {
            Vertex v{};
            if (idx.vertex_index >= 0) {
                const size_t vi = static_cast<size_t>(idx.vertex_index) * 3;
                v.pos = glm::vec3(attrib.vertices[vi + 0],
                                  attrib.vertices[vi + 1],
                                  attrib.vertices[vi + 2]) * scale;
            }
            if (idx.normal_index >= 0) {
                const size_t ni = static_cast<size_t>(idx.normal_index) * 3;
                v.normal = glm::normalize(glm::vec3(attrib.normals[ni + 0],
                                                    attrib.normals[ni + 1],
                                                    attrib.normals[ni + 2]));
            }
            if (idx.texcoord_index >= 0) {
                const size_t ti = static_cast<size_t>(idx.texcoord_index) * 2;
                v.uv = glm::vec2(attrib.texcoords[ti + 0], attrib.texcoords[ti + 1]);
            }
            vertices.push_back(v);
        }
    }

    if (vertices.empty()) {
        std::cerr << "OBJ has no triangles: " << path << "\n";
        return mesh;
    }

    glm::vec3 mn(std::numeric_limits<float>::max());
    glm::vec3 mx(std::numeric_limits<float>::lowest());
    for (const auto& v : vertices) {
        mn = glm::min(mn, v.pos);
        mx = glm::max(mx, v.pos);
    }
    mesh.boundsMin = mn;
    mesh.boundsMax = mx;
    mesh.hasBounds = true;

    glGenVertexArrays(1, &mesh.vao);
    glBindVertexArray(mesh.vao);

    glGenBuffers(1, &mesh.vbo);
    glBindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, uv));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    mesh.vertexCount = vertices.size();
    mesh.cpuVertices = std::move(vertices);
    return mesh;
}

static CityModel loadCityModel(const std::string& path, float scale) {
    CityModel cm;

    if (!std::filesystem::exists(path)) {
        std::cerr << "OBJ missing: " << path << "\n";
        return cm;
    }

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    std::filesystem::path p(path);
    std::string baseDir = p.parent_path().string();
    if (!baseDir.empty()) baseDir += "/";
    if (auto ref = readGeoReferenceFromObj(path)) {
        cm.utmCenter = ref->utmCenter;
    }

    bool ok = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                               path.c_str(), baseDir.c_str(), true);
    if (!warn.empty()) std::cerr << "OBJ warn: " << warn << "\n";
    if (!err.empty())  std::cerr << "OBJ err: " << err << "\n";
    if (!ok) {
        std::cerr << "OBJ load failed: " << path << "\n";
        return cm;
    }

    std::vector<MaterialFallback> mtlFallback;
    std::unordered_map<std::string, int> mtlNameToId;
    std::vector<int> faceMatIds;

    if (materials.empty()) {
        std::string mtlPath;
        std::ifstream objIn(path);
        std::string line;
        while (std::getline(objIn, line)) {
            if (line.rfind("mtllib ", 0) == 0) {
                mtlPath = baseDir + line.substr(7);
                break;
            }
        }

        if (!mtlPath.empty() && loadMtlFallback(mtlPath, mtlFallback, mtlNameToId)) {
            materials.resize(mtlFallback.size());
            for (size_t i = 0; i < mtlFallback.size(); ++i) {
                materials[i].name = mtlFallback[i].name;
                materials[i].diffuse[0] = mtlFallback[i].diffuse.r;
                materials[i].diffuse[1] = mtlFallback[i].diffuse.g;
                materials[i].diffuse[2] = mtlFallback[i].diffuse.b;
                materials[i].diffuse_texname = mtlFallback[i].diffuseTex;
            }
            faceMatIds = parseObjFaceMaterialIds(path, mtlNameToId);
        }
    }

    std::cout << "City materials: " << materials.size() << "\n";
    if (!materials.empty()) {
        size_t sampleCount = std::min<size_t>(3, materials.size());
        for (size_t i = 0; i < sampleCount; ++i) {
            std::cout << "City mat[" << i << "] map_Kd: " << materials[i].diffuse_texname << "\n";
        }
    }

    std::unordered_map<int, std::vector<Vertex>> vertsByMat;
    const std::vector<glm::vec3> palette = {
        {0.75f, 0.75f, 0.75f},
        {0.85f, 0.80f, 0.70f},
        {0.70f, 0.80f, 0.85f},
        {0.80f, 0.70f, 0.85f},
        {0.85f, 0.85f, 0.70f},
        {0.70f, 0.85f, 0.80f},
        {0.80f, 0.75f, 0.70f},
        {0.75f, 0.80f, 0.70f}
    };
    glm::vec3 mn(std::numeric_limits<float>::max());
    glm::vec3 mx(std::numeric_limits<float>::lowest());

    size_t globalFace = 0;
    for (const auto& shape : shapes) {
        size_t indexOffset = 0;
        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); ++f) {
            int fv = shape.mesh.num_face_vertices[f];
            int matId = -1;
            if (!shape.mesh.material_ids.empty()) {
                matId = shape.mesh.material_ids[f];
            } else if (!faceMatIds.empty() && globalFace < faceMatIds.size()) {
                matId = faceMatIds[globalFace];
            }
            if (fv != 3) { indexOffset += fv; continue; }

            std::array<Vertex, 3> tri{};
            for (int v = 0; v < 3; ++v) {
                tinyobj::index_t idx = shape.mesh.indices[indexOffset + v];
                if (idx.vertex_index >= 0) {
                    const size_t vi = static_cast<size_t>(idx.vertex_index) * 3;
                    tri[v].pos = glm::vec3(attrib.vertices[vi + 0],
                                           attrib.vertices[vi + 1],
                                           attrib.vertices[vi + 2]) * scale;
                    mn = glm::min(mn, tri[v].pos);
                    mx = glm::max(mx, tri[v].pos);
                }
                if (idx.normal_index >= 0) {
                    const size_t ni = static_cast<size_t>(idx.normal_index) * 3;
                    tri[v].normal = glm::normalize(glm::vec3(attrib.normals[ni + 0],
                                                             attrib.normals[ni + 1],
                                                             attrib.normals[ni + 2]));
                }
                if (idx.texcoord_index >= 0) {
                    const size_t ti = static_cast<size_t>(idx.texcoord_index) * 2;
                    tri[v].uv = glm::vec2(attrib.texcoords[ti + 0], attrib.texcoords[ti + 1]);
                }
            }

            if (materials.empty() || matId < 0 || static_cast<size_t>(matId) >= materials.size()) {
                glm::vec3 c = (tri[0].pos + tri[1].pos + tri[2].pos) / 3.0f;
                int bucket = static_cast<int>(std::abs(c.x * 0.1f + c.z * 0.1f));
                matId = 1000 + (bucket % static_cast<int>(palette.size()));
            }

            auto& verts = vertsByMat[matId];
            verts.push_back(tri[0]);
            verts.push_back(tri[1]);
            verts.push_back(tri[2]);

            indexOffset += fv;
            globalFace++;
        }
    }

    if (!vertsByMat.empty()) {
        cm.boundsMin = mn;
        cm.boundsMax = mx;
        cm.hasBounds = true;
    }

    int texturesTried = 0;
    int texturesLoaded = 0;
    for (const auto& [matId, verts] : vertsByMat) {
        if (verts.empty()) continue;
        ColoredMesh part;

        glGenVertexArrays(1, &part.mesh.vao);
        glBindVertexArray(part.mesh.vao);

        glGenBuffers(1, &part.mesh.vbo);
        glBindBuffer(GL_ARRAY_BUFFER, part.mesh.vbo);
        glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(Vertex), verts.data(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, uv));
        glEnableVertexAttribArray(2);

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        part.mesh.vertexCount = verts.size();
        part.mesh.cpuVertices = verts;

        if (matId >= 0 && static_cast<size_t>(matId) < materials.size()) {
            const auto& m = materials[matId];
            part.color = glm::vec3(m.diffuse[0], m.diffuse[1], m.diffuse[2]);
            if (!m.diffuse_texname.empty()) {
                texturesTried++;
                std::filesystem::path texPath = std::filesystem::path(baseDir) / m.diffuse_texname;
                if (std::filesystem::exists(texPath)) {
                    sf::Image img;
                    if (img.loadFromFile(texPath.string())) {
                        glGenTextures(1, &part.textureId);
                        glBindTexture(GL_TEXTURE_2D, part.textureId);
                        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, img.getSize().x, img.getSize().y, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.getPixelsPtr());
                        glGenerateMipmap(GL_TEXTURE_2D);
                        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
                        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
                        glBindTexture(GL_TEXTURE_2D, 0);
                        part.hasTexture = true;
                        texturesLoaded++;
                    } else {
                        std::cerr << "City texture load failed: " << texPath << "\n";
                    }
                } else {
                    std::cerr << "City texture missing: " << texPath << "\n";
                }
            }
        } else if (matId >= 1000) {
            int idx = (matId - 1000) % static_cast<int>(palette.size());
            part.color = palette[idx];
        }
        cm.parts.push_back(std::move(part));
    }

    if (texturesTried > 0) {
        std::cout << "City textures loaded: " << texturesLoaded << " / " << texturesTried << "\n";
    }

    return cm;
}

static void drawMesh(const Mesh& mesh, GLuint shaderProgram, const glm::mat4& model, const glm::vec3& color,
                     GLuint textureId = 0, bool useTexture = false) {
    if (!mesh.valid()) return;

    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(model));
    GLint colorLoc = glGetUniformLocation(shaderProgram, "uObjectColor");
    if (colorLoc != -1) glUniform3fv(colorLoc, 1, glm::value_ptr(color));

    GLint useTexLoc = glGetUniformLocation(shaderProgram, "uUseTexture");
    if (useTexLoc != -1) glUniform1i(useTexLoc, useTexture ? 1 : 0);
    GLint texLoc = glGetUniformLocation(shaderProgram, "uTexture");
    if (texLoc != -1) glUniform1i(texLoc, 1);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, (useTexture && textureId) ? textureId : 0);

    glBindVertexArray(mesh.vao);
    glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(mesh.vertexCount));
    glBindVertexArray(0);

    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
}

struct DroneModel {
    Mesh body;
    Mesh propFL;
    Mesh propFR;
    Mesh propRL;
    Mesh propRR;
    bool loaded = false;

    void destroy() {
        body.destroy();
        propFL.destroy();
        propFR.destroy();
        propRL.destroy();
        propRR.destroy();
        loaded = false;
    }
};

static DroneModel loadDroneModel(const std::string& dir, float scale) {
    DroneModel dm;
    dm.body  = loadObjMesh(dir + "/body.obj", scale);
    dm.propFL = loadObjMesh(dir + "/prop_FL.obj", scale);
    dm.propFR = loadObjMesh(dir + "/prop_FR.obj", scale);
    dm.propRL = loadObjMesh(dir + "/prop_RL.obj", scale);
    dm.propRR = loadObjMesh(dir + "/prop_RR.obj", scale);
    dm.loaded = dm.body.valid();
    if (!dm.body.valid()) std::cerr << "Drone body mesh invalid.\n";
    if (!dm.propFL.valid()) std::cerr << "Drone prop FL mesh invalid.\n";
    if (!dm.propFR.valid()) std::cerr << "Drone prop FR mesh invalid.\n";
    if (!dm.propRL.valid()) std::cerr << "Drone prop RL mesh invalid.\n";
    if (!dm.propRR.valid()) std::cerr << "Drone prop RR mesh invalid.\n";
    if (!dm.loaded) std::cerr << "Drone model incomplete. Falling back to cube.\n";
    return dm;
}

// ===================== Math / Pose =====================
struct Pose {
    glm::vec3 pos{0.f, 100.0f, 0.f};
    float yaw = 0.f;   // degrees (CCW around +Y when seen from above)
    float pitch = 0.f; // degrees
};

// Forward from yaw/pitch: yaw=0 -> +X, yaw=90 -> +Z
static inline glm::vec3 forwardFrom(const Pose& p) {
    float y = glm::radians(p.yaw), pit = glm::radians(p.pitch);
    return glm::normalize(glm::vec3(std::cos(y) * std::cos(pit),
                                    std::sin(pit),
                                    std::sin(y) * std::cos(pit)));
}

// ===================== Shader utils =====================
namespace glutil {

// Load text file into string (used for shader source)
static std::string loadText(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "ERROR: Could not open file: " << filepath << std::endl;
        return "";
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// Compile + link shader program from vertex/fragment paths
static GLuint createShaderProgram(const std::string& vertPath, const std::string& fragPath) {
    std::string vertSource = loadText(vertPath);
    std::string fragSource = loadText(fragPath);
    if (vertSource.empty() || fragSource.empty()) return 0;

    const char* vsrc = vertSource.c_str();
    const char* fsrc = fragSource.c_str();

    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &vsrc, nullptr);
    glCompileShader(vs);
    GLint ok = GL_FALSE;
    glGetShaderiv(vs, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024]; glGetShaderInfoLog(vs, 1024, nullptr, log);
        std::cerr << "Vertex shader compile error:\n" << log << std::endl;
    }

    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &fsrc, nullptr);
    glCompileShader(fs);
    glGetShaderiv(fs, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024]; glGetShaderInfoLog(fs, 1024, nullptr, log);
        std::cerr << "Fragment shader compile error:\n" << log << std::endl;
    }

    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024]; glGetProgramInfoLog(prog, 1024, nullptr, log);
        std::cerr << "Program link error:\n" << log << std::endl;
    }

    glDeleteShader(vs);
    glDeleteShader(fs);

    if (ok) std::cout << "Shader program loaded: " << vertPath << " | " << fragPath << std::endl;
    return prog;
}

// Compile + link shader program from in-memory source
static GLuint createShaderProgramFromSource(const std::string& vertSource, const std::string& fragSource) {
    if (vertSource.empty() || fragSource.empty()) return 0;

    const char* vsrc = vertSource.c_str();
    const char* fsrc = fragSource.c_str();

    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &vsrc, nullptr);
    glCompileShader(vs);
    GLint ok = GL_FALSE;
    glGetShaderiv(vs, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024]; glGetShaderInfoLog(vs, 1024, nullptr, log);
        std::cerr << "Vertex shader compile error:\n" << log << std::endl;
    }

    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &fsrc, nullptr);
    glCompileShader(fs);
    glGetShaderiv(fs, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024]; glGetShaderInfoLog(fs, 1024, nullptr, log);
        std::cerr << "Fragment shader compile error:\n" << log << std::endl;
    }

    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);
    glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024]; glGetProgramInfoLog(prog, 1024, nullptr, log);
        std::cerr << "Program link error:\n" << log << std::endl;
    }

    glDeleteShader(vs);
    glDeleteShader(fs);

    return ok ? prog : 0;
}

} // namespace glutil

// ===================== Drone =====================
struct Drone {
    std::string name = "drone";
    Pose p;
    float speed = 6.f;    // m/s
    float yawRate = 90.f; // deg/s
    float climb = 3.f;    // m/s
    glm::vec3 bodyScale{0.5f, 0.15f, 0.5f};
    glm::vec3 color{0.15f, 0.15f, 0.15f};
    float collisionRadius = 0.45f;
    bool routeEnabled = false;
    std::vector<glm::vec3> route;
    size_t routeIndex = 0;
};

static inline bool hasManualDroneInput() {
    return sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W) ||
           sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S) ||
           sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A) ||
           sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D) ||
           sf::Keyboard::isKeyPressed(sf::Keyboard::Key::C) ||
           sf::Keyboard::isKeyPressed(sf::Keyboard::Key::V) ||
           sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Q) ||
           sf::Keyboard::isKeyPressed(sf::Keyboard::Key::E);
}

// Read keyboard and update drone physics.
static inline void updateDroneManual(Drone& d, float dt) {
    glm::vec3 fwd = forwardFrom(d.p);
    glm::vec3 right = glm::normalize(glm::cross(fwd, {0, 1, 0}));

    glm::vec3 move(0);
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) move += fwd;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)) move -= fwd;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) move -= right;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) move += right;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::C)) d.p.pos.y += d.climb * dt;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::V)) d.p.pos.y -= d.climb * dt;

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Q)) d.p.yaw -= d.yawRate * dt;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::E)) d.p.yaw += d.yawRate * dt;

    if (glm::length(move) > 0.f) d.p.pos += glm::normalize(move) * (d.speed * dt);
}

static inline void updateDroneRoute(Drone& d, float dt) {
    if (!d.routeEnabled || d.route.empty()) return;
    if (d.routeIndex >= d.route.size()) d.routeIndex = 0;

    glm::vec3 target = d.route[d.routeIndex];
    glm::vec3 delta = target - d.p.pos;
    float dist = glm::length(delta);
    if (dist < 0.5f) {
        d.routeIndex = (d.routeIndex + 1) % d.route.size();
        target = d.route[d.routeIndex];
        delta = target - d.p.pos;
        dist = glm::length(delta);
    }
    if (dist < 1e-4f) return;

    glm::vec3 dir = delta / dist;
    float step = std::min(d.speed * dt, dist);
    d.p.pos += dir * step;
    d.p.yaw = glm::degrees(std::atan2(dir.z, dir.x));
    d.p.pitch = glm::degrees(std::asin(glm::clamp(dir.y, -1.0f, 1.0f)));
}

// Draw drone as a scaled cube using object shader
static inline void drawDroneBox(const Pose& p, const glm::vec3& scale, GLuint shaderProgram, GLuint vao,
                                const glm::vec3& color = glm::vec3(0.15f, 0.15f, 0.15f)) {
    glm::mat4 model(1.f);
    model = glm::translate(model, p.pos);
    model = glm::rotate(model, glm::radians(-p.yaw), glm::vec3(0, 1, 0));
    model = glm::rotate(model, glm::radians(p.pitch), glm::vec3(1, 0, 0));
    model = glm::scale(model, scale);

    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(model));
    glUniform3fv(glGetUniformLocation(shaderProgram, "uObjectColor"), 1, glm::value_ptr(color));
    GLint useTexLoc = glGetUniformLocation(shaderProgram, "uUseTexture");
    if (useTexLoc != -1) glUniform1i(useTexLoc, 0);

    glBindVertexArray(vao);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);
}

// Draw drone from OBJ meshes (body + props)
static inline void drawDroneModel(const Drone& d, const DroneModel& model, float propAngle, GLuint shaderProgram) {
    if (!model.body.valid()) return;

    glm::mat4 base(1.f);
    base = glm::translate(base, d.p.pos);
    base = glm::rotate(base, glm::radians(-d.p.yaw), glm::vec3(0, 1, 0));
    base = glm::rotate(base, glm::radians(d.p.pitch), glm::vec3(1, 0, 0));

    // Model orientation fix: flip forward, then yaw +90.
    const float modelFlipDeg = 180.0f;
    const float modelYawFixDeg = 90.0f;
    glm::mat4 modelFix = glm::rotate(glm::mat4(1.f), glm::radians(modelFlipDeg), glm::vec3(0, 1, 0));
    modelFix = glm::rotate(modelFix, glm::radians(modelYawFixDeg), glm::vec3(0, 1, 0));

    // Body-only fixes
    const float bodyRollFixDeg = 270.0f;
    const float bodyYawFixDeg = 270.0f;
    glm::mat4 bodyFix = glm::rotate(glm::mat4(1.f), glm::radians(bodyYawFixDeg), glm::vec3(0, 1, 0));
    bodyFix = glm::rotate(bodyFix, glm::radians(bodyRollFixDeg), glm::vec3(1, 0, 0));

    // Prop offsets (diagonal distance 600 mm -> half side = 0.212132 m)
    const float propHalf = 0.212132f;
    const float propLift = 0.06f;
    const glm::vec3 offFL{ +propHalf, propLift, -propHalf };
    const glm::vec3 offFR{ +propHalf, propLift, +propHalf };
    const glm::vec3 offRL{ -propHalf, propLift, -propHalf };
    const glm::vec3 offRR{ -propHalf, propLift, +propHalf };

    const glm::vec3 bodyColor = d.color;
    const glm::vec3 propColor(0.1f, 0.1f, 0.1f);

    drawMesh(model.body, shaderProgram, base * modelFix * bodyFix, bodyColor);

    glm::mat4 rotCW  = glm::rotate(glm::mat4(1.f), propAngle, glm::vec3(0, 1, 0));
    glm::mat4 rotCCW = glm::rotate(glm::mat4(1.f), -propAngle, glm::vec3(0, 1, 0));

    if (model.propFL.valid()) drawMesh(model.propFL, shaderProgram, base * modelFix * glm::translate(glm::mat4(1.f), offFL) * rotCW, propColor);
    if (model.propRR.valid()) drawMesh(model.propRR, shaderProgram, base * modelFix * glm::translate(glm::mat4(1.f), offRR) * rotCW, propColor);
    if (model.propFR.valid()) drawMesh(model.propFR, shaderProgram, base * modelFix * glm::translate(glm::mat4(1.f), offFR) * rotCCW, propColor);
    if (model.propRL.valid()) drawMesh(model.propRL, shaderProgram, base * modelFix * glm::translate(glm::mat4(1.f), offRL) * rotCCW, propColor);
}

void drawDroneProjectedShadow(const Drone& drone, const glm::vec3& lightPos, GLuint shadowShader, GLuint cubeVAO, float groundY = 0.0f) {
    glUseProgram(shadowShader);
    
    // Lichtposition (L)
    float Lx = lightPos.x;
    float Ly = lightPos.y;
    float Lz = lightPos.z;

    // Ebene (Y=groundY)
    float A = 0.0f;
    float B = 1.0f;
    float C = 0.0f;
    float D = -groundY - 0.001f; // Bias

    // Dot product: L.P (Homogener Punkt L mit Ebene P) = Lx*A + Ly*B + Lz*C + 1*D
    float dot = Lx * A + Ly * B + Lz * C + 1.0f * D;

    // Planar Projection Matrix M
    glm::mat4 M(1.0f);
    
    // M_ij = (Dot) * delta_ij - L_i * P_j

    // Spalte 0 (x)
    M[0][0] = dot - Lx * A;
    M[1][0] = 0.0f - Lx * B;
    M[2][0] = 0.0f - Lx * C;
    M[3][0] = 0.0f - Lx * D;

    // Spalte 1 (y)
    M[0][1] = 0.0f - Ly * A;
    M[1][1] = dot - Ly * B;
    M[2][1] = 0.0f - Ly * C;
    M[3][1] = 0.0f - Ly * D;

    // Spalte 2 (z)
    M[0][2] = 0.0f - Lz * A;
    M[1][2] = 0.0f - Lz * B;
    M[2][2] = dot - Lz * C;
    M[3][2] = 0.0f - Lz * D;

    // Spalte 3 (w)
    M[0][3] = 0.0f - 1.0f * A;
    M[1][3] = 0.0f - 1.0f * B;
    M[2][3] = 0.0f - 1.0f * C;
    M[3][3] = dot - 1.0f * D; 

    // Drohnen-Modell-Matrix
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, drone.p.pos);
    glm::vec3 shadowScale(drone.bodyScale.x * 2.0f, drone.bodyScale.y, drone.bodyScale.z * 2.0f);
    model = glm::scale(model, shadowScale);
    
    // Schattenmodell-Matrix = Projektionsmatrix * Drohnen-Modell-Matrix
    glm::mat4 shadowModel = M * model;

    GLint modelLoc = glGetUniformLocation(shadowShader, "model");
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(shadowModel));

    glBindVertexArray(cubeVAO);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);
    glUseProgram(0);
}

// ===================== Scene Objects (Buildings) =====================
class Object3D {
public:
    std::string name;
    glm::vec3 position;
    glm::vec3 dimensions;
    glm::vec3 color;
    glm::vec3 scale{1.0f, 1.0f, 1.0f};

    // Construct from YAML node
    explicit Object3D(const YAML::Node& config) {
        name = config["name"].as<std::string>();
        position = glm::vec3(config["position"][0].as<float>(),
                             config["position"][1].as<float>(),
                             config["position"][2].as<float>());
        dimensions = glm::vec3(config["dimensions"][0].as<float>(),
                               config["dimensions"][1].as<float>(),
                               config["dimensions"][2].as<float>());
        color = glm::vec3(config["color"][0].as<float>(),
                          config["color"][1].as<float>(),
                          config["color"][2].as<float>());
    }

    glm::mat4 getModelMatrix() const {
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, position);
        model = glm::scale(model, scale);
        return model;
    }

    // Draw a cube for this object using provided shader/VAO
    void draw(GLuint shaderProgram, GLuint vao) const {
        glm::mat4 model(1.f);
        model = glm::translate(model, position);
        model = glm::scale(model, dimensions);

        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(model));
        if (glGetUniformLocation(shaderProgram, "uObjectColor") != -1) {
            glUniform3fv(glGetUniformLocation(shaderProgram, "uObjectColor"), 1, glm::value_ptr(color));
        }
        GLint useTexLoc = glGetUniformLocation(shaderProgram, "uUseTexture");
        if (useTexLoc != -1) glUniform1i(useTexLoc, 0);

        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glBindVertexArray(0);
    }

    static void drawProjectedShadow(const Object3D& object, const glm::vec3& lightPos, GLuint shadowShader, GLuint cubeVAO, float groundY = 0.0f) {
        glUseProgram(shadowShader);
        
        // Lichtposition (L)
        float Lx = lightPos.x;
        float Ly = lightPos.y;
        float Lz = lightPos.z;

        // Ebene (Y=groundY)
        // Normal N=(0, 1, 0) -> A=0, B=1, C=0
        // D (Abstand zum Ursprung) ist -groundY
        float A = 0.0f;
        float B = 1.0f;
        float C = 0.0f;
        // Bias: Verschiebt die Ebene leicht nach oben (Z-Fighting vermeiden)
        float D = -groundY - 0.001f; 

        // Dot product: L.P (Homogener Punkt L mit Ebene P) = Lx*A + Ly*B + Lz*C + 1*D
        float dot = Lx * A + Ly * B + Lz * C + 1.0f * D;

        // Planar Projection Matrix M (Standardformel)
        glm::mat4 M(1.0f);

        // M_ij = (Dot) * delta_ij - L_i * P_j
        
        // Spalte 0 (x)
        M[0][0] = dot - Lx * A;
        M[1][0] = 0.0f - Lx * B;
        M[2][0] = 0.0f - Lx * C;
        M[3][0] = 0.0f - Lx * D;

        // Spalte 1 (y)
        M[0][1] = 0.0f - Ly * A;
        M[1][1] = dot - Ly * B;
        M[2][1] = 0.0f - Ly * C;
        M[3][1] = 0.0f - Ly * D;

        // Spalte 2 (z)
        M[0][2] = 0.0f - Lz * A;
        M[1][2] = 0.0f - Lz * B;
        M[2][2] = dot - Lz * C;
        M[3][2] = 0.0f - Lz * D;

        // Spalte 3 (w)
        M[0][3] = 0.0f - 1.0f * A;
        M[1][3] = 0.0f - 1.0f * B;
        M[2][3] = 0.0f - 1.0f * C;
        M[3][3] = dot - 1.0f * D; 

        // Schattenmodell-Matrix = Projektionsmatrix * Objekt-Modell-Matrix
        glm::mat4 shadowModel = M * object.getModelMatrix();

        GLint modelLoc = glGetUniformLocation(shadowShader, "model");
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(shadowModel));

        glBindVertexArray(cubeVAO);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glBindVertexArray(0);
        glUseProgram(0);
    }
};

// ===================== Grid helpers =====================
// Create colored grid vertices around origin (XZ plane)
static std::vector<float> generateGridVertices(int halfSize = 5) {
    std::vector<float> vertices;
    float lineColor[3] = {0.25f, 0.25f, 0.25f};
    float centerColor[3] = {0.3f, 0.3f, 0.3f};

    float yOffset = 0.001f; // Slightly above ground to avoid Z-fighting

    for (int x = -halfSize; x <= halfSize; x++) {
        bool isCenter = (x == 0);
        auto [r, g, b] = std::tuple{isCenter ? centerColor[0] : lineColor[0],
                                    isCenter ? centerColor[1] : lineColor[1],
                                    isCenter ? centerColor[2] : lineColor[2]};
        vertices.insert(vertices.end(), {
            static_cast<float>(x), yOffset, static_cast<float>(-halfSize), r, g, b,
            static_cast<float>(x), yOffset, static_cast<float>(halfSize),  r, g, b
        });
    }
    for (int z = -halfSize; z <= halfSize; z++) {
        bool isCenter = (z == 0);
        auto [r, g, b] = std::tuple{isCenter ? centerColor[0] : lineColor[0],
                                    isCenter ? centerColor[1] : lineColor[1],
                                    isCenter ? centerColor[2] : lineColor[2]};
        vertices.insert(vertices.end(), {
            static_cast<float>(-halfSize), yOffset, static_cast<float>(z), r, g, b,
            static_cast<float>(halfSize),  yOffset, static_cast<float>(z), r, g, b
        });
    }
    return vertices;
}

// Create VAO/VBO for grid lines
static void createGridVAO(GLuint& vao, GLuint& vbo, int& vertexCount, int halfSize = 100) {
    std::vector<float> gridVertices = generateGridVertices(halfSize);
    vertexCount = static_cast<int>(gridVertices.size() / 6);

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, gridVertices.size() * sizeof(float), gridVertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

// Create VAO/VBO for ground plane
static void createGroundVAO(GLuint& vao, GLuint& vbo, float halfSize = 100.0f) {
        float groundVertices[] = {
        // PosX, PosY, PosZ, NormalX, NormalY, NormalZ (6 floats pro Vertex)
        -halfSize, 0.0f, -halfSize, 0.0f, 1.0f, 0.0f, // Unten links
        halfSize, 0.0f, -halfSize, 0.0f, 1.0f, 0.0f, // Unten rechts
        -halfSize, 0.0f,  halfSize, 0.0f, 1.0f, 0.0f, // Oben links

        halfSize, 0.0f, -halfSize, 0.0f, 1.0f, 0.0f, // Unten rechts
        halfSize, 0.0f,  halfSize, 0.0f, 1.0f, 0.0f, // Oben rechts
        -halfSize, 0.0f,  halfSize, 0.0f, 1.0f, 0.0f  // Oben links
    };

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(groundVertices), groundVertices, GL_STATIC_DRAW);

    // Position Attribute (Location 0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Normale Attribute (Location 1) - Wichtig für den Schatten-Bias!
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

// ===================== Cube mesh (for buildings/drone) =====================
static void createCubeVAO(GLuint& vao, GLuint& vbo) {
    // Interleaved (pos, normal). 36 vertices.
    float cubeVertices[] = {
        // Rückseite (-Z)
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
        0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
        0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
        0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
        -0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,

        // Vorderseite (+Z)
        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
        0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
        0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
        0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
        -0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,

        // Links (-X)
        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,

        // Rechts (+X)
        0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
        0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
        0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
        0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
        0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
        0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,

        // Unten (-Y)
        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
        0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
        0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
        0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,

        // Oben (+Y)
        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
        0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
        0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
        0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
        -0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f
    };

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);

    GLsizei stride = 6 * sizeof(float);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

static void createSensorVAO(GLuint& vao, GLuint& vbo) {

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    // VBO mit dynamischem Speicher reservieren (3 Floats pro Punkt/Vertex)
    glBufferData(GL_ARRAY_BUFFER, 10000 * 3 * sizeof(float), NULL, GL_DYNAMIC_DRAW); 
    // Position (location 0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

static void createScreenQuadVAO(GLuint& vao, GLuint& vbo) {
    float quadVertices[] = {
        // positions   // texcoords
        -1.0f, -1.0f,   0.0f, 0.0f,
         1.0f, -1.0f,   1.0f, 0.0f,
         1.0f,  1.0f,   1.0f, 1.0f,
        -1.0f,  1.0f,   0.0f, 1.0f
    };

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// ===================== Simple light/geometry helpers =====================
static inline bool projectPointToGround(const glm::vec3& L, const glm::vec3& P, float yPlane, glm::vec3& out) {
    glm::vec3 dir = P - L;
    if (std::abs(dir.y) < 1e-6f) return false;
    float t = (yPlane - L.y) / dir.y;
    if (t <= 0.f) return false;
    out = L + t * dir;
    return true;
}
static std::vector<glm::vec3> convexHullXZ(std::vector<glm::vec3> pts); // forward

// ===================== Lidar / Radar (simulation + debug draw) =====================
struct AABB { glm::vec3 mn, mx; int id; };
struct Triangle { glm::vec3 a, b, c; int id; };
struct RayHit { bool ok = false; float t = 0.0f; glm::vec3 point{0.f}; int id = -1; };
struct BVHNode {
    glm::vec3 mn{0.f};
    glm::vec3 mx{0.f};
    int left = -1;
    int right = -1;
    size_t start = 0;
    size_t count = 0;

    bool leaf() const { return left < 0 && right < 0; }
};
struct WorldGeometry {
    std::vector<AABB> boxes;
    std::vector<Triangle> triangles;
    std::vector<int> triangleIndices;
    std::vector<BVHNode> triangleBvh;
    glm::vec3 boundsMin{0.f};
    glm::vec3 boundsMax{0.f};
    bool hasBounds = false;
};

static inline AABB makeAABB(const Object3D& o) {
    glm::vec3 h = 0.5f * o.dimensions;
    return {o.position - h, o.position + h, 0};
}

static inline AABB makeAABBFromMesh(const Mesh& mesh, const glm::mat4& model, int id) {
    glm::vec3 mn = mesh.boundsMin;
    glm::vec3 mx = mesh.boundsMax;
    glm::vec3 corners[8] = {
        {mn.x, mn.y, mn.z}, {mx.x, mn.y, mn.z}, {mn.x, mx.y, mn.z}, {mx.x, mx.y, mn.z},
        {mn.x, mn.y, mx.z}, {mx.x, mn.y, mx.z}, {mn.x, mx.y, mx.z}, {mx.x, mx.y, mx.z}
    };
    glm::vec3 outMin(std::numeric_limits<float>::max());
    glm::vec3 outMax(std::numeric_limits<float>::lowest());
    for (const auto& c : corners) {
        glm::vec4 w = model * glm::vec4(c, 1.0f);
        glm::vec3 p(w.x, w.y, w.z);
        outMin = glm::min(outMin, p);
        outMax = glm::max(outMax, p);
    }
    return {outMin, outMax, id};
}

static inline AABB makeAABBFromBounds(const glm::vec3& mn, const glm::vec3& mx, const glm::mat4& model, int id) {
    glm::vec3 corners[8] = {
        {mn.x, mn.y, mn.z}, {mx.x, mn.y, mn.z}, {mn.x, mx.y, mn.z}, {mx.x, mx.y, mn.z},
        {mn.x, mn.y, mx.z}, {mx.x, mn.y, mx.z}, {mn.x, mx.y, mx.z}, {mx.x, mx.y, mx.z}
    };
    glm::vec3 outMin(std::numeric_limits<float>::max());
    glm::vec3 outMax(std::numeric_limits<float>::lowest());
    for (const auto& c : corners) {
        glm::vec4 w = model * glm::vec4(c, 1.0f);
        glm::vec3 p(w.x, w.y, w.z);
        outMin = glm::min(outMin, p);
        outMax = glm::max(outMax, p);
    }
    return {outMin, outMax, id};
}

static inline bool rayAABB(const glm::vec3& ro, const glm::vec3& rd, const AABB& b, float tMax, float& tHit) {
    float tmin = 0.001f, tmax = tMax;
    for (int i=0;i<3;++i){
        float d = rd[i]; if (std::abs(d) < 1e-6f) d = (d<0 ? -1e-6f : 1e-6f);
        float invD = 1.f / d;
        float t0 = (b.mn[i] - ro[i]) * invD;
        float t1 = (b.mx[i] - ro[i]) * invD;
        if (invD < 0.f) std::swap(t0, t1);
        tmin = t0 > tmin ? t0 : tmin;
        tmax = t1 < tmax ? t1 : tmax;
        if (tmax <= tmin) return false;
    }
    tHit = tmin;
    return true;
}

static inline bool rayBounds(const glm::vec3& ro, const glm::vec3& rd,
                             const glm::vec3& mn, const glm::vec3& mx,
                             float tMax, float& tNear) {
    float tmin = 0.001f, tmax = tMax;
    for (int i = 0; i < 3; ++i) {
        float d = rd[i];
        if (std::abs(d) < 1e-6f) d = (d < 0 ? -1e-6f : 1e-6f);
        float invD = 1.0f / d;
        float t0 = (mn[i] - ro[i]) * invD;
        float t1 = (mx[i] - ro[i]) * invD;
        if (invD < 0.0f) std::swap(t0, t1);
        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);
        if (tmax <= tmin) return false;
    }
    tNear = tmin;
    return true;
}

static inline bool rayTriangle(const glm::vec3& ro, const glm::vec3& rd, const Triangle& tri, float tMax, float& tHit) {
    const float eps = 1e-6f;
    glm::vec3 edge1 = tri.b - tri.a;
    glm::vec3 edge2 = tri.c - tri.a;
    glm::vec3 h = glm::cross(rd, edge2);
    float det = glm::dot(edge1, h);
    if (std::abs(det) < eps) return false;

    float invDet = 1.0f / det;
    glm::vec3 s = ro - tri.a;
    float u = invDet * glm::dot(s, h);
    if (u < 0.0f || u > 1.0f) return false;

    glm::vec3 q = glm::cross(s, edge1);
    float v = invDet * glm::dot(rd, q);
    if (v < 0.0f || u + v > 1.0f) return false;

    float t = invDet * glm::dot(edge2, q);
    if (t <= 0.001f || t > tMax) return false;

    tHit = t;
    return true;
}

static inline glm::vec3 triangleCentroid(const Triangle& tri) {
    return (tri.a + tri.b + tri.c) / 3.0f;
}

static inline void expandBounds(glm::vec3& mn, glm::vec3& mx, const glm::vec3& p) {
    mn = glm::min(mn, p);
    mx = glm::max(mx, p);
}

static inline void triangleBounds(const Triangle& tri, glm::vec3& mn, glm::vec3& mx) {
    mn = glm::min(tri.a, glm::min(tri.b, tri.c));
    mx = glm::max(tri.a, glm::max(tri.b, tri.c));
}

static int buildTriangleBVHRecursive(WorldGeometry& world, size_t start, size_t end) {
    BVHNode node;
    node.start = start;
    node.count = end - start;

    glm::vec3 mn(std::numeric_limits<float>::max());
    glm::vec3 mx(std::numeric_limits<float>::lowest());
    glm::vec3 centroidMn(std::numeric_limits<float>::max());
    glm::vec3 centroidMx(std::numeric_limits<float>::lowest());

    for (size_t i = start; i < end; ++i) {
        const Triangle& tri = world.triangles[static_cast<size_t>(world.triangleIndices[i])];
        glm::vec3 triMn, triMx;
        triangleBounds(tri, triMn, triMx);
        expandBounds(mn, mx, triMn);
        expandBounds(mn, mx, triMx);
        expandBounds(centroidMn, centroidMx, triangleCentroid(tri));
    }

    node.mn = mn;
    node.mx = mx;
    int nodeIndex = static_cast<int>(world.triangleBvh.size());
    world.triangleBvh.push_back(node);

    constexpr size_t leafSize = 8;
    if (node.count <= leafSize) return nodeIndex;

    glm::vec3 extent = centroidMx - centroidMn;
    int axis = 0;
    if (extent.y > extent.x && extent.y >= extent.z) axis = 1;
    else if (extent.z > extent.x && extent.z >= extent.y) axis = 2;

    size_t mid = start + node.count / 2;
    if (extent[axis] > 1e-5f) {
        std::nth_element(world.triangleIndices.begin() + static_cast<std::ptrdiff_t>(start),
                         world.triangleIndices.begin() + static_cast<std::ptrdiff_t>(mid),
                         world.triangleIndices.begin() + static_cast<std::ptrdiff_t>(end),
                         [&](int lhs, int rhs) {
                             return triangleCentroid(world.triangles[static_cast<size_t>(lhs)])[axis] <
                                    triangleCentroid(world.triangles[static_cast<size_t>(rhs)])[axis];
                         });
    }

    if (mid == start || mid == end) return nodeIndex;

    int left = buildTriangleBVHRecursive(world, start, mid);
    int right = buildTriangleBVHRecursive(world, mid, end);
    world.triangleBvh[static_cast<size_t>(nodeIndex)].left = left;
    world.triangleBvh[static_cast<size_t>(nodeIndex)].right = right;
    world.triangleBvh[static_cast<size_t>(nodeIndex)].count = 0;
    return nodeIndex;
}

static void buildTriangleBVH(WorldGeometry& world) {
    world.triangleIndices.clear();
    world.triangleBvh.clear();
    if (world.triangles.empty()) return;

    world.triangleIndices.reserve(world.triangles.size());
    for (size_t i = 0; i < world.triangles.size(); ++i) {
        world.triangleIndices.push_back(static_cast<int>(i));
    }

    world.triangleBvh.reserve(world.triangles.size() / 4 + 1);
    buildTriangleBVHRecursive(world, 0, world.triangleIndices.size());
}

static inline void rayTriangleBVH(const glm::vec3& ro, const glm::vec3& rd, const WorldGeometry& world, RayHit& best) {
    if (world.triangleBvh.empty()) return;

    std::array<int, 128> stack{};
    int stackSize = 0;
    stack[stackSize++] = 0;

    while (stackSize > 0) {
        int nodeIndex = stack[--stackSize];
        const BVHNode& node = world.triangleBvh[static_cast<size_t>(nodeIndex)];
        float tNear = 0.0f;
        if (!rayBounds(ro, rd, node.mn, node.mx, best.t, tNear)) continue;

        if (node.leaf()) {
            for (size_t i = node.start; i < node.start + node.count; ++i) {
                const Triangle& tri = world.triangles[static_cast<size_t>(world.triangleIndices[i])];
                float th = 0.0f;
                if (rayTriangle(ro, rd, tri, best.t, th) && th < best.t) {
                    best = {true, th, ro + rd * th, tri.id};
                }
            }
        } else {
            if (node.left >= 0 && stackSize < static_cast<int>(stack.size())) stack[stackSize++] = node.left;
            if (node.right >= 0 && stackSize < static_cast<int>(stack.size())) stack[stackSize++] = node.right;
        }
    }
}

static inline RayHit rayWorld(const glm::vec3& ro, const glm::vec3& rd, const WorldGeometry& world, float tMax) {
    RayHit best;
    best.t = tMax;

    for (const auto& box : world.boxes) {
        float th = 0.0f;
        if (rayAABB(ro, rd, box, tMax, th) && th < best.t) {
            best = {true, th, ro + rd * th, box.id};
        }
    }

    if (!world.triangleBvh.empty()) {
        rayTriangleBVH(ro, rd, world, best);
    } else {
        for (const auto& tri : world.triangles) {
            float th = 0.0f;
            if (rayTriangle(ro, rd, tri, best.t, th) && th < best.t) {
                best = {true, th, ro + rd * th, tri.id};
            }
        }
    }

    return best;
}

static inline float sceneRayLimit(const glm::vec3& origin, const WorldGeometry& world, float configuredMax) {
    if (configuredMax > 0.0f) return configuredMax;
    if (!world.hasBounds) return 100000.0f;
    glm::vec3 farCorner = glm::max(glm::abs(world.boundsMin - origin), glm::abs(world.boundsMax - origin));
    return glm::length(farCorner) + 100.0f;
}

struct Hit3D { bool ok; float range; glm::vec3 point; int objId; glm::vec3 dir; };
static inline void writeLidarFrameYaml(const std::string& outputDir,
                                       int frameId,
                                       const std::vector<Hit3D>& hits,
                                       int beamsH, int beamsV,
                                       float fovH_deg, float fovV_deg) {
    if (hits.empty()) return;

    std::error_code ec;
    std::filesystem::create_directories(outputDir, ec);
    if (ec) {
        std::cerr << "LiDAR export: could not create directory '" << outputDir
                  << "' (" << ec.message() << ")\n";
        return;
    }

    std::ostringstream filename;
    filename << outputDir << "/lidar_frame_" << std::setw(6) << std::setfill('0') << frameId << ".yaml";
    std::ofstream out(filename.str());
    if (!out.is_open()) {
        std::cerr << "LiDAR export: could not open file " << filename.str() << "\n";
        return;
    }

    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &now_c);
#else
    localtime_r(&now_c, &tm);
#endif
    std::ostringstream ts;
     ts << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S")
         << '.' << std::setw(3) << std::setfill('0') << ms.count();

    const auto hitCount = std::count_if(hits.begin(), hits.end(), [](const Hit3D& h){ return h.ok; });

    out << "timestamp: " << ts.str() << "\n";
    out << "frame_id: " << frameId << "\n";
    out << "hit_count: " << hitCount << "\n";
    out << "miss_count: " << (hits.size() - hitCount) << "\n";
    out << "beams_h: " << beamsH << "\n";
    out << "beams_v: " << beamsV << "\n";
    out << "fov_h_deg: " << fovH_deg << "\n";
    out << "fov_v_deg: " << fovV_deg << "\n";
    out << "beams:\n";

    out << std::fixed << std::setprecision(4);

    size_t idx = 0;
    const float denomV = static_cast<float>(std::max(1, beamsV - 1));
    const float denomH = static_cast<float>(std::max(1, beamsH - 1));
    for (int j = 0; j < beamsV; ++j) {
        float v = ((j / denomV) - 0.5f) * glm::radians(fovV_deg);
        for (int i = 0; i < beamsH; ++i) {
            float h = ((i / denomH) - 0.5f) * glm::radians(fovH_deg);
            const auto& hit = hits[idx++];

            out << "  - index: " << (idx - 1) << "\n";
            out << "    h_index: " << i << "\n";
            out << "    v_index: " << j << "\n";
            out << "    azimuth_deg: " << glm::degrees(h) << "\n";
            out << "    elevation_deg: " << glm::degrees(v) << "\n";
            out << "    distance: " << hit.range << "\n";
            out << "    hit: " << (hit.ok ? "true" : "false") << "\n";
            out << "    id: " << hit.objId << "\n";
        }
    }
}
static inline std::vector<Hit3D> simulateLidar3D(const Pose& p,
                                                 const WorldGeometry& world,
                                                 int beamsH, int beamsV,
                                                 float fovH_deg, float fovV_deg,
                                                 float maxR) {
    std::vector<Hit3D> hits;
    hits.reserve(beamsH * beamsV);
    const float rayLimit = sceneRayLimit(p.pos, world, maxR);

    // Basis from DRONE pose (not camera!)
    float yaw=glm::radians(p.yaw), pit=glm::radians(p.pitch);
    glm::vec3 F = glm::normalize(glm::vec3(std::cos(yaw)*std::cos(pit), std::sin(pit), std::sin(yaw)*std::cos(pit)));
    glm::vec3 R = glm::normalize(glm::cross(F, {0,1,0}));
    glm::vec3 U = glm::normalize(glm::cross(R, F));

    for (int j=0; j<beamsV; ++j) {
        float v = ((j/(float)(beamsV-1)) - 0.5f) * glm::radians(fovV_deg);
        for (int i=0; i<beamsH; ++i) {
            float h = ((i/(float)(beamsH-1)) - 0.5f) * glm::radians(fovH_deg);
            glm::vec3 dir = glm::normalize(F + std::tan(h)*R + std::tan(v)*U);

            RayHit hit = rayWorld(p.pos, dir, world, rayLimit);
            if (hit.ok) hits.push_back({true, hit.t, hit.point, hit.id, dir});
            else        hits.push_back({false, rayLimit, p.pos + dir*rayLimit, -1, dir});
        }
    }
    return hits;
}

struct RadarParams {
    int beamsH = 60, beamsV = 8;
    float fovH = 90.f, fovV = 20.f;
    float maxR = 150.f, minR = 2.f;
    float snr0 = 40.f, snrMin = 8.f;
};
struct RadarDet { bool ok; float range, vr, az, el; glm::vec3 point; int objId; };

static inline void writeRadarFrameYaml(const std::string& outputDir,
                                       int frameId,
                                       const std::vector<RadarDet>& dets,
                                       const RadarParams& R) {
    if (dets.empty()) return;

    std::error_code ec;
    std::filesystem::create_directories(outputDir, ec);
    if (ec) {
        std::cerr << "Radar export: could not create directory '" << outputDir
                  << "' (" << ec.message() << ")\n";
        return;
    }

    std::ostringstream filename;
    filename << outputDir << "/radar_frame_" << std::setw(6) << std::setfill('0') << frameId << ".yaml";
    std::ofstream out(filename.str());
    if (!out.is_open()) {
        std::cerr << "Radar export: could not open file " << filename.str() << "\n";
        return;
    }

    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &now_c);
#else
    localtime_r(&now_c, &tm);
#endif
    std::ostringstream ts;
    ts << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S")
       << '.' << std::setw(3) << std::setfill('0') << ms.count();

    const auto hitCount = std::count_if(dets.begin(), dets.end(), [](const RadarDet& d){ return d.ok; });

    out << "timestamp: " << ts.str() << "\n";
    out << "frame_id: " << frameId << "\n";
    out << "hit_count: " << hitCount << "\n";
    out << "miss_count: " << (dets.size() - hitCount) << "\n";
    out << "beams_h: " << R.beamsH << "\n";
    out << "beams_v: " << R.beamsV << "\n";
    out << "fov_h_deg: " << R.fovH << "\n";
    out << "fov_v_deg: " << R.fovV << "\n";
    out << "max_range: " << R.maxR << "\n";
    out << "min_range: " << R.minR << "\n";
    out << "snr0: " << R.snr0 << "\n";
    out << "snr_min: " << R.snrMin << "\n";
    out << "detections:\n";

    out << std::fixed << std::setprecision(4);

    size_t idx = 0;
    for (int v = 0; v < R.beamsV; ++v) {
        for (int h = 0; h < R.beamsH; ++h) {
            const auto& det = dets[idx++];

            out << "  - index: " << (idx - 1) << "\n";
            out << "    h_index: " << h << "\n";
            out << "    v_index: " << v << "\n";
            out << "    azimuth_deg: " << glm::degrees(det.az) << "\n";
            out << "    elevation_deg: " << glm::degrees(det.el) << "\n";
            out << "    range: " << det.range << "\n";
            out << "    vr: " << det.vr << "\n";
            out << "    hit: " << (det.ok ? "true" : "false") << "\n";
            out << "    id: " << det.objId << "\n";
        }
    }
}

static inline std::vector<RadarDet> simulateRadar3D(const Pose& sensorPose,
                                                    const glm::vec3& sensorVel,
                                                    const WorldGeometry& world,
                                                    const RadarParams& R) {
    std::vector<RadarDet> dets;
    dets.reserve(R.beamsH * R.beamsV);
    const float rayLimit = sceneRayLimit(sensorPose.pos, world, R.maxR);

    float yaw = glm::radians(sensorPose.yaw), pit = glm::radians(sensorPose.pitch);
    glm::vec3 F = glm::normalize(glm::vec3(std::cos(yaw)*std::cos(pit), std::sin(pit), std::sin(yaw)*std::cos(pit)));
    glm::vec3 Rv = glm::normalize(glm::cross(F, glm::vec3(0,1,0)));
    glm::vec3 U  = glm::normalize(glm::cross(Rv, F));

    for (int v = 0; v < R.beamsV; ++v) {
        float el = ((v / (float)(R.beamsV-1)) - 0.5f) * glm::radians(R.fovV);
        for (int h = 0; h < R.beamsH; ++h) {
            float az = ((h / (float)(R.beamsH-1)) - 0.5f) * glm::radians(R.fovH);

            glm::vec3 dir = glm::normalize(F + std::tan(az)*Rv + std::tan(el)*U);

            RayHit hit = rayWorld(sensorPose.pos, dir, world, rayLimit);
            if (!hit.ok || hit.t <= R.minR) {
                dets.push_back({false, rayLimit, 0.f, az, el, sensorPose.pos + dir*rayLimit, -1});
                continue;
            }

            // SNR ~ snr0 - 40 log10(R) (1/R^4 power law)
            float snr = R.snr0 - 40.f * std::log10(std::max(hit.t, 1e-2f));
            if (snr < R.snrMin) {
                dets.push_back({false, rayLimit, 0.f, az, el, sensorPose.pos + dir*rayLimit, -1});
                continue;
            }

            // Radial velocity: projection of platform velocity onto LOS, + closing
            // float vr = glm::dot(sensorVel, dir) * (-1.0f); // flip to make +closing
            float vr = glm::dot(sensorVel, dir); // flip to make +closing

            dets.push_back({true, hit.t, vr, az, el, hit.point, hit.id});
        }
    }
    return dets;
}

// Zeichnet einen Würfel an der Lichtposition (unbeleuchtet)
static inline void drawLightSource(const glm::vec3& lightPos, GLuint shaderProgram, GLuint vao, const glm::mat4& view, const glm::mat4& projection) {
    glUseProgram(shaderProgram);
    
    // Setze View/Projection Matrizen
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uProjection"), 1, GL_FALSE, glm::value_ptr(projection));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uView"), 1, GL_FALSE, glm::value_ptr(view));
    
    // Model Matrix (Würfel auf Lichtposition skalieren/platzieren)
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, lightPos);
    model = glm::scale(model, glm::vec3(0.2f)); // Kleinere Licht-Kugel
    
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(model));
    glUniform3f(glGetUniformLocation(shaderProgram, "uColor"), 1.0f, 1.0f, 0.0f); // Gelbe Lichtfarbe
    glUniform1i(glGetUniformLocation(shaderProgram, "uIsPoint"), 0);

    glBindVertexArray(vao);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);
    glUseProgram(0);
}

// ===================== Draw lines and points helpers =====================
// Helfer zum Zeichnen von Punkten (LiDAR, Radar)
static void drawPoints(const std::vector<glm::vec3>& points, const glm::vec3& color, float pointSize,
                       GLuint shaderProgram, GLuint vao, GLuint vbo,
                       const glm::mat4& view, const glm::mat4& projection,
                       bool depthTest) {
    if (points.empty()) return;

    // 1. Daten in VBO übertragen (dynamisches Update)
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    // Ersetze den VBO-Inhalt jedes Frame
    glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(glm::vec3), points.data(), GL_DYNAMIC_DRAW); 
    
    // 2. Shader aktivieren
    glUseProgram(shaderProgram);

    // 3. Uniforms senden
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uProjection"), 1, GL_FALSE, glm::value_ptr(projection));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uView"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(glm::mat4(1.0f))); // Identitätsmatrix
    glUniform3fv(glGetUniformLocation(shaderProgram, "uColor"), 1, glm::value_ptr(color));
    glUniform1f(glGetUniformLocation(shaderProgram, "uPointSize"), pointSize);
    glUniform1i(glGetUniformLocation(shaderProgram, "uIsPoint"), 1);

    // 4. Zeichnen
    glEnable(GL_PROGRAM_POINT_SIZE);
    if (depthTest) glEnable(GL_DEPTH_TEST);
    else glDisable(GL_DEPTH_TEST);
    glBindVertexArray(vao);
    glDrawArrays(GL_POINTS, 0, points.size());
    glBindVertexArray(0);
    glDisable(GL_PROGRAM_POINT_SIZE);

    if (depthTest) glEnable(GL_DEPTH_TEST);
    else glDisable(GL_DEPTH_TEST);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glUseProgram(0);
}

// Helfer zum Zeichnen von Linien (Radar Beams, LiDAR Beams)
static void drawLines(const std::vector<glm::vec3>& vertices, const glm::vec3& color, float lineWidth, GLuint shaderProgram, GLuint vao, GLuint vbo, const glm::mat4& view, const glm::mat4& projection) {
    if (vertices.empty()) return;

    // 1. Daten in VBO übertragen (dynamisches Update)
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), vertices.data(), GL_DYNAMIC_DRAW); 
    
    // 2. Shader aktivieren
    glUseProgram(shaderProgram);

    // 3. Uniforms senden
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uProjection"), 1, GL_FALSE, glm::value_ptr(projection));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uView"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(glm::mat4(1.0f))); // Identitätsmatrix
    glUniform3fv(glGetUniformLocation(shaderProgram, "uColor"), 1, glm::value_ptr(color));
    glUniform1i(glGetUniformLocation(shaderProgram, "uIsPoint"), 0);
    
    glLineWidth(lineWidth);

    // 4. Zeichnen
    glEnable(GL_DEPTH_TEST);
    glBindVertexArray(vao);
    glDrawArrays(GL_LINES, 0, vertices.size());
    glBindVertexArray(0);
    
    glLineWidth(1.0f); // Zurücksetzen
    
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glUseProgram(0);
}

// ===================== Shadow map (depth-only FBO) =====================
struct ShadowMap {
    GLuint fbo = 0;
    GLuint depthTex = 0;
    unsigned w = 4096, h = 4096;

    void init() {
        GLint maxTexSize = 0;
        glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTexSize);
        if (maxTexSize > 0) {
            if (w > static_cast<unsigned>(maxTexSize)) w = static_cast<unsigned>(maxTexSize);
            if (h > static_cast<unsigned>(maxTexSize)) h = static_cast<unsigned>(maxTexSize);
        }

        glGenFramebuffers(1, &fbo);
        glGenTextures(1, &depthTex);

        glBindTexture(GL_TEXTURE_2D, depthTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, w, h, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        float border[] = {1, 1, 1, 1};
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthTex, 0);
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            std::cerr << "ERROR::FRAMEBUFFER:: Shadow Map FBO incomplete\n";
        } else {
            std::cout << "Shadow map size: " << w << "x" << h << " (GL_MAX_TEXTURE_SIZE=" << maxTexSize << ")\n";
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void bindForWrite() const {
        glViewport(0, 0, w, h);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glClear(GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
    }

    void bindForRead(GLuint shader, GLint samplerUniform = 0) const {
        glActiveTexture(GL_TEXTURE0 + samplerUniform);
        glBindTexture(GL_TEXTURE_2D, depthTex);
        glUniform1i(glGetUniformLocation(shader, "uShadowMap"), samplerUniform);
    }
};

// ===================== Drone camera (FBO + overlay) =====================
struct CameraFBO {
    GLuint fbo = 0;
    GLuint colorTex = 0;
    GLuint depthRbo = 0;
    unsigned w = 640, h = 360;

    void init(unsigned width, unsigned height) {
        w = width; h = height;
        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);

        glGenTextures(1, &colorTex);
        glBindTexture(GL_TEXTURE_2D, colorTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTex, 0);

        glGenRenderbuffers(1, &depthRbo);
        glBindRenderbuffer(GL_RENDERBUFFER, depthRbo);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, w, h);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depthRbo);

        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            std::cerr << "ERROR::FRAMEBUFFER:: Camera FBO incomplete\n";
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void bind() const {
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glViewport(0, 0, w, h);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
    }

    void unbind(unsigned windowW, unsigned windowH) const {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, windowW, windowH);
    }
};

static void saveCameraFrame(const std::string& outputDir, int frameId, const CameraFBO& camFbo) {
    std::error_code ec;
    std::filesystem::create_directories(outputDir, ec);
    if (ec) {
        std::cerr << "Camera export: could not create directory '" << outputDir
                  << "' (" << ec.message() << ")\n";
        return;
    }

    std::vector<std::uint8_t> pixels(camFbo.w * camFbo.h * 4);
    glBindFramebuffer(GL_FRAMEBUFFER, camFbo.fbo);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, camFbo.w, camFbo.h, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    std::vector<std::uint8_t> flipped(camFbo.w * camFbo.h * 4);
    for (unsigned y = 0; y < camFbo.h; ++y) {
        const auto srcY = camFbo.h - 1 - y;
        std::memcpy(&flipped[y * camFbo.w * 4], &pixels[srcY * camFbo.w * 4], camFbo.w * 4);
    }

    sf::Image image({camFbo.w, camFbo.h}, flipped.data());

    std::ostringstream filename;
    filename << outputDir << "/camera_frame_" << std::setw(6) << std::setfill('0') << frameId << ".png";
    if (!image.saveToFile(filename.str())) {
        std::cerr << "Camera export: could not save file " << filename.str() << "\n";
    }
}

static void drawCameraOverlay(GLuint shader, GLuint quadVAO, GLuint texture,
                              unsigned windowW, unsigned windowH,
                              unsigned overlayW, unsigned overlayH,
                              unsigned margin = 16) {
    if (overlayW == 0 || overlayH == 0) return;

    unsigned x = windowW > (overlayW + margin) ? (windowW - overlayW - margin) : 0;
    unsigned y = margin;

    glViewport(x, y, overlayW, overlayH);
    glDisable(GL_DEPTH_TEST);

    glUseProgram(shader);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture);
    glUniform1i(glGetUniformLocation(shader, "uTex"), 0);

    glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glBindVertexArray(0);

    glBindTexture(GL_TEXTURE_2D, 0);
    glUseProgram(0);

    glViewport(0, 0, windowW, windowH);
    glEnable(GL_DEPTH_TEST);
}

static void drawCameraFrame(GLuint shader, GLuint quadVAO,
                            unsigned windowW, unsigned windowH,
                            unsigned overlayW, unsigned overlayH,
                            unsigned border = 6,
                            unsigned margin = 16,
                            const glm::vec3& color = glm::vec3(0.0f, 0.0f, 0.0f)) {
    if (overlayW == 0 || overlayH == 0) return;

    unsigned frameW = overlayW + border * 2;
    unsigned frameH = overlayH + border * 2;

    unsigned x = windowW > (overlayW + margin)
                   ? (windowW - overlayW - margin - border)
                   : 0;
    unsigned y = margin > border ? (margin - border) : 0;

    glViewport(x, y, frameW, frameH);
    glDisable(GL_DEPTH_TEST);

    glUseProgram(shader);
    glUniform3fv(glGetUniformLocation(shader, "uColor"), 1, glm::value_ptr(color));

    glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glBindVertexArray(0);

    glUseProgram(0);
    glViewport(0, 0, windowW, windowH);
    glEnable(GL_DEPTH_TEST);
}

// ===================== Camera state/update =====================
struct Camera {
    glm::vec3 pos{0.0f, 3.0f, 8.0f};
    glm::vec3 up{0.0f, 1.0f, 0.0f};
    glm::vec3 front{0.0f, 0.0f, -1.0f};
    glm::vec3 right{1.0f, 0.0f, 0.0f};
    float yaw = -90.f, pitch = -10.f;

    glm::vec3 calcFront(float yaw_, float pitch_) const {
        glm::vec3 f;
        f.x = std::cos(glm::radians(yaw_)) * std::cos(glm::radians(pitch_));
        f.y = std::sin(glm::radians(pitch_));
        f.z = std::sin(glm::radians(yaw_)) * std::cos(glm::radians(pitch_));
        return glm::normalize(f);
    }
};

// Apply free-camera movement (WASD + CV for up/down)
static void updateFreeCamera(Camera& cam, float dt, float speed) {
    glm::vec3 move(0.0f);
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) move += cam.front;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)) move -= cam.front;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) move -= cam.right;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) move += cam.right;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::C)) move += cam.up;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::V)) move -= cam.up;

    if (glm::length(move) > 0.0f) cam.pos += glm::normalize(move) * (speed * dt);
}

// Place follow-camera behind/above drone and look forward
static void updateFollowCamera(const Drone& drone, Camera& cam) {
    const float back = 4.0f, up = 2.0f, lead = 2.0f;

    float y = glm::radians(drone.p.yaw);
    glm::vec3 F_cam = glm::normalize(glm::vec3(std::cos(y), 0.0f, std::sin(y)));

    cam.pos = drone.p.pos - F_cam * back + cam.up * up;
    glm::vec3 target = drone.p.pos + F_cam * lead;

    cam.front = glm::normalize(target - cam.pos);
    cam.right = glm::normalize(glm::cross(cam.front, cam.up));
}

// ===================== Scene load helpers =====================
// Load objects from config.yaml; if missing, create defaults
static std::vector<std::unique_ptr<Object3D>> loadObjectsOrDefault(const std::string& cfgPath) {
    std::vector<std::unique_ptr<Object3D>> objects;
    try {
        YAML::Node config = YAML::LoadFile(cfgPath);
        for (const auto& objectConfig : config["objects"]) {
            objects.push_back(std::make_unique<Object3D>(objectConfig));
        }
        std::cout << "Loaded " << objects.size() << " objects from " << cfgPath << "\n";
    } catch (...) {
        std::cerr << "Error loading " << cfgPath << ", creating default objects...\n";
        YAML::Node o1; o1["name"]="RedCube";   o1["position"]=std::vector<float>{0,1,0};  o1["dimensions"]=std::vector<float>{1,1,1};   o1["color"]=std::vector<float>{1,0,0};
        YAML::Node o2; o2["name"]="GreenCube"; o2["position"]=std::vector<float>{3,1,2};  o2["dimensions"]=std::vector<float>{1,2,1};   o2["color"]=std::vector<float>{0,1,0};
        YAML::Node o3; o3["name"]="BlueCube";  o3["position"]=std::vector<float>{-2,0.5f,1}; o3["dimensions"]=std::vector<float>{1,0.5f,1}; o3["color"]=std::vector<float>{0,0,1};
        objects.push_back(std::make_unique<Object3D>(o1));
        objects.push_back(std::make_unique<Object3D>(o2));
        objects.push_back(std::make_unique<Object3D>(o3));
    }
    return objects;
}

struct LidarConfig {
    int beamsH = 36;
    int beamsV = 15;
    float fovH_deg = 360.f;
    float fovV_deg = 45.f;
    float maxRange = 50.f;
    float fps = 10.0f;
    std::string outputDir = "lidar_output";
};

struct RadarConfig {
    int beamsH = 60;
    int beamsV = 8;
    float fovH_deg = 90.f;
    float fovV_deg = 20.f;
    float maxRange = 150.f;
    float minRange = 2.f;
    float snr0 = 40.f;
    float snrMin = 8.f;
    float fps = 10.0f;
    std::string outputDir = "radar_output";
};

struct CameraConfig {
    unsigned width = 640;
    unsigned height = 360;
    float fov_deg = 60.f;
    float fps = 10.0f;
    std::string outputDir = "camera_output";
};

struct SensorConfig {
    LidarConfig lidar;
    RadarConfig radar;
    CameraConfig camera;
};

static SensorConfig loadSensorConfig(const std::string& cfgPath) {
    SensorConfig cfg;
    try {
        YAML::Node root = YAML::LoadFile(cfgPath);
        if (root["lidar"]) {
            auto n = root["lidar"];
            if (n["beamsH"]) cfg.lidar.beamsH = n["beamsH"].as<int>();
            if (n["beamsV"]) cfg.lidar.beamsV = n["beamsV"].as<int>();
            if (n["fovH_deg"]) cfg.lidar.fovH_deg = n["fovH_deg"].as<float>();
            if (n["fovV_deg"]) cfg.lidar.fovV_deg = n["fovV_deg"].as<float>();
            if (n["maxRange"]) cfg.lidar.maxRange = n["maxRange"].as<float>();
            if (n["fps"]) cfg.lidar.fps = n["fps"].as<float>();
            if (n["outputDir"]) cfg.lidar.outputDir = n["outputDir"].as<std::string>();
        }
        if (root["radar"]) {
            auto n = root["radar"];
            if (n["beamsH"]) cfg.radar.beamsH = n["beamsH"].as<int>();
            if (n["beamsV"]) cfg.radar.beamsV = n["beamsV"].as<int>();
            if (n["fovH_deg"]) cfg.radar.fovH_deg = n["fovH_deg"].as<float>();
            if (n["fovV_deg"]) cfg.radar.fovV_deg = n["fovV_deg"].as<float>();
            if (n["maxRange"]) cfg.radar.maxRange = n["maxRange"].as<float>();
            if (n["minRange"]) cfg.radar.minRange = n["minRange"].as<float>();
            if (n["snr0"]) cfg.radar.snr0 = n["snr0"].as<float>();
            if (n["snrMin"]) cfg.radar.snrMin = n["snrMin"].as<float>();
            if (n["fps"]) cfg.radar.fps = n["fps"].as<float>();
            if (n["outputDir"]) cfg.radar.outputDir = n["outputDir"].as<std::string>();
        }
        if (root["camera"]) {
            auto n = root["camera"];
            if (n["width"]) cfg.camera.width = n["width"].as<unsigned>();
            if (n["height"]) cfg.camera.height = n["height"].as<unsigned>();
            if (n["fov_deg"]) cfg.camera.fov_deg = n["fov_deg"].as<float>();
            if (n["fps"]) cfg.camera.fps = n["fps"].as<float>();
            if (n["outputDir"]) cfg.camera.outputDir = n["outputDir"].as<std::string>();
        }
        std::cout << "Loaded sensor config from " << cfgPath << "\n";
    } catch (...) {
        std::cerr << "Error loading " << cfgPath << ", using default sensor settings...\n";
    }
    return cfg;
}

static inline void expandWorldBounds(WorldGeometry& world, const glm::vec3& p) {
    if (!world.hasBounds) {
        world.boundsMin = p;
        world.boundsMax = p;
        world.hasBounds = true;
    } else {
        world.boundsMin = glm::min(world.boundsMin, p);
        world.boundsMax = glm::max(world.boundsMax, p);
    }
}

static void addMeshTrianglesToWorld(WorldGeometry& world, const Mesh& mesh, const glm::mat4& transform, int id) {
    if (mesh.cpuVertices.size() < 3) return;
    for (size_t i = 0; i + 2 < mesh.cpuVertices.size(); i += 3) {
        glm::vec3 a = glm::vec3(transform * glm::vec4(mesh.cpuVertices[i + 0].pos, 1.0f));
        glm::vec3 b = glm::vec3(transform * glm::vec4(mesh.cpuVertices[i + 1].pos, 1.0f));
        glm::vec3 c = glm::vec3(transform * glm::vec4(mesh.cpuVertices[i + 2].pos, 1.0f));
        world.triangles.push_back({a, b, c, id});
        expandWorldBounds(world, a);
        expandWorldBounds(world, b);
        expandWorldBounds(world, c);
    }
}

static void addCityModelToWorld(WorldGeometry& world, const CityModel& city, const glm::mat4& transform, int idStart) {
    int id = idStart;
    for (const auto& part : city.parts) {
        addMeshTrianglesToWorld(world, part.mesh, transform, id++);
    }
}

// Build static geometry for ray tests and collision.
static WorldGeometry buildWorldGeometry(const std::vector<std::unique_ptr<Object3D>>& objects,
                                        const CityModel* city,
                                        const glm::mat4* cityTransform,
                                        const CityModel* terrain,
                                        const glm::mat4* terrainTransform) {
    WorldGeometry world;
    world.boxes.reserve(objects.size());
    for (size_t i = 0; i < objects.size(); ++i) {
        auto box = makeAABB(*objects[i]);
        box.id = static_cast<int>(i);
        world.boxes.push_back(box);
        expandWorldBounds(world, box.mn);
        expandWorldBounds(world, box.mx);
    }

    int nextId = static_cast<int>(world.boxes.size());
    if (city && cityTransform && city->valid()) {
        addCityModelToWorld(world, *city, *cityTransform, nextId);
        nextId += static_cast<int>(city->parts.size());
    }
    if (terrain && terrainTransform && terrain->valid()) {
        addCityModelToWorld(world, *terrain, *terrainTransform, nextId);
    }
    buildTriangleBVH(world);
    std::cout << "World collision geometry: " << world.boxes.size()
              << " boxes, " << world.triangles.size() << " triangles, "
              << world.triangleBvh.size() << " BVH nodes\n";
    if (world.hasBounds) {
        std::cout << "World bounds: min(" << world.boundsMin.x << ", " << world.boundsMin.y << ", " << world.boundsMin.z
                  << ") max(" << world.boundsMax.x << ", " << world.boundsMax.y << ", " << world.boundsMax.z << ")\n";
    }
    return world;
}

static inline glm::vec3 closestPointOnTriangle(const glm::vec3& p, const Triangle& tri) {
    const glm::vec3 ab = tri.b - tri.a;
    const glm::vec3 ac = tri.c - tri.a;
    const glm::vec3 ap = p - tri.a;

    float d1 = glm::dot(ab, ap);
    float d2 = glm::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return tri.a;

    const glm::vec3 bp = p - tri.b;
    float d3 = glm::dot(ab, bp);
    float d4 = glm::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return tri.b;

    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        float v = d1 / (d1 - d3);
        return tri.a + v * ab;
    }

    const glm::vec3 cp = p - tri.c;
    float d5 = glm::dot(ab, cp);
    float d6 = glm::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return tri.c;

    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        float w = d2 / (d2 - d6);
        return tri.a + w * ac;
    }

    float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return tri.b + w * (tri.c - tri.b);
    }

    float denom = 1.0f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    return tri.a + ab * v + ac * w;
}

static bool resolveSphereWorldCollision(glm::vec3& pos, float radius, const WorldGeometry& world) {
    bool collided = false;
    const float radiusSq = radius * radius;

    for (const auto& box : world.boxes) {
        glm::vec3 closest = glm::clamp(pos, box.mn, box.mx);
        glm::vec3 delta = pos - closest;
        float distSq = glm::dot(delta, delta);
        if (distSq >= radiusSq) continue;

        collided = true;
        if (distSq > 1e-8f) {
            float dist = std::sqrt(distSq);
            pos += (delta / dist) * (radius - dist + 0.01f);
        } else {
            glm::vec3 toMin = pos - box.mn;
            glm::vec3 toMax = box.mx - pos;
            int axis = 0;
            float depth = std::min(toMin.x, toMax.x);
            if (std::min(toMin.y, toMax.y) < depth) { axis = 1; depth = std::min(toMin.y, toMax.y); }
            if (std::min(toMin.z, toMax.z) < depth) { axis = 2; depth = std::min(toMin.z, toMax.z); }
            float sign = (toMin[axis] < toMax[axis]) ? -1.0f : 1.0f;
            pos[axis] += sign * (depth + radius + 0.01f);
        }
    }

    auto resolveTriangle = [&](const Triangle& tri) {
        glm::vec3 closest = closestPointOnTriangle(pos, tri);
        glm::vec3 delta = pos - closest;
        float distSq = glm::dot(delta, delta);
        if (distSq >= radiusSq) return false;

        glm::vec3 normal = glm::normalize(glm::cross(tri.b - tri.a, tri.c - tri.a));
        if (!std::isfinite(normal.x) || glm::length(normal) < 1e-5f) return false;
        if (glm::dot(normal, delta) < 0.0f) normal = -normal;

        float dist = std::sqrt(std::max(distSq, 0.0f));
        glm::vec3 pushDir = dist > 1e-5f ? (delta / dist) : normal;
        pos += pushDir * (radius - dist + 0.01f);
        return true;
    };

    auto sphereOverlapsBounds = [&](const glm::vec3& mn, const glm::vec3& mx) {
        glm::vec3 closest = glm::clamp(pos, mn, mx);
        glm::vec3 delta = pos - closest;
        return glm::dot(delta, delta) <= radiusSq;
    };

    if (!world.triangleBvh.empty()) {
        std::array<int, 128> stack{};
        int stackSize = 0;
        stack[stackSize++] = 0;
        while (stackSize > 0) {
            int nodeIndex = stack[--stackSize];
            const BVHNode& node = world.triangleBvh[static_cast<size_t>(nodeIndex)];
            if (!sphereOverlapsBounds(node.mn, node.mx)) continue;

            if (node.leaf()) {
                for (size_t i = node.start; i < node.start + node.count; ++i) {
                    const Triangle& tri = world.triangles[static_cast<size_t>(world.triangleIndices[i])];
                    collided = resolveTriangle(tri) || collided;
                }
            } else {
                if (node.left >= 0 && stackSize < static_cast<int>(stack.size())) stack[stackSize++] = node.left;
                if (node.right >= 0 && stackSize < static_cast<int>(stack.size())) stack[stackSize++] = node.right;
            }
        }
    } else {
        for (const auto& tri : world.triangles) {
            collided = resolveTriangle(tri) || collided;
        }
    }

    return collided;
}

static inline void resolveDroneCollision(Drone& drone, const glm::vec3& previousPos, const WorldGeometry& world) {
    glm::vec3 corrected = drone.p.pos;
    if (resolveSphereWorldCollision(corrected, drone.collisionRadius, world)) {
        drone.p.pos = corrected;
        if (resolveSphereWorldCollision(drone.p.pos, drone.collisionRadius, world)) {
            drone.p.pos = previousPos;
        }
    }
    drone.p.pos.y = std::max(drone.p.pos.y, drone.collisionRadius);
}

static glm::vec3 parseSimPoint(const YAML::Node& n, const GeoReference* geoRef) {
    if (n.IsSequence() && n.size() >= 3) {
        return glm::vec3(n[0].as<float>(), n[1].as<float>(), n[2].as<float>());
    }
    if (n.IsMap()) {
        if (n["position"]) return parseSimPoint(n["position"], geoRef);
        if (n["local"]) return parseSimPoint(n["local"], geoRef);
        if (n["utm"] && geoRef && geoRef->valid) {
            const auto u = n["utm"];
            return geoRef->worldToSim(u[0].as<double>(), u[1].as<double>(), u[2].as<float>());
        }
        if (n["easting"] && n["northing"] && n["altitude"] && geoRef && geoRef->valid) {
            return geoRef->worldToSim(n["easting"].as<double>(), n["northing"].as<double>(), n["altitude"].as<float>());
        }
    }
    return glm::vec3(0.0f, 30.0f, 0.0f);
}

static std::vector<Drone> loadDronesFromConfig(const std::string& cfgPath, const GeoReference* geoRef) {
    std::vector<Drone> drones;
    try {
        YAML::Node root = YAML::LoadFile(cfgPath);
        if (!root["drones"]) return drones;

        int idx = 0;
        for (const auto& n : root["drones"]) {
            Drone d;
            d.name = n["name"] ? n["name"].as<std::string>() : ("drone" + std::to_string(idx + 1));
            if (n["position"]) d.p.pos = parseSimPoint(n["position"], geoRef);
            if (n["yaw"]) d.p.yaw = n["yaw"].as<float>();
            if (n["pitch"]) d.p.pitch = n["pitch"].as<float>();
            if (n["speed"]) d.speed = n["speed"].as<float>();
            if (n["collisionRadius"]) d.collisionRadius = n["collisionRadius"].as<float>();
            if (n["color"] && n["color"].IsSequence() && n["color"].size() >= 3) {
                d.color = glm::vec3(n["color"][0].as<float>(), n["color"][1].as<float>(), n["color"][2].as<float>());
            }
            if (n["route"]) {
                for (const auto& wp : n["route"]) d.route.push_back(parseSimPoint(wp, geoRef));
                d.routeEnabled = d.route.size() >= 2;
                if (!d.route.empty() && !n["position"]) d.p.pos = d.route.front();
            }
            drones.push_back(std::move(d));
            idx++;
        }
        std::cout << "Loaded " << drones.size() << " drones from " << cfgPath << "\n";
    } catch (...) {
        std::cerr << "Error loading drones from " << cfgPath << ", using default drone routes...\n";
    }
    return drones;
}

static std::vector<Drone> createDefaultDrones(const WorldGeometry& world) {
    glm::vec3 mn = world.hasBounds ? world.boundsMin : glm::vec3(-80.f, 0.f, -80.f);
    glm::vec3 mx = world.hasBounds ? world.boundsMax : glm::vec3(80.f, 80.f, 80.f);
    float minX = std::max(mn.x + 20.0f, -120.0f);
    float maxX = std::min(mx.x - 20.0f, 120.0f);
    float minZ = std::max(mn.z + 20.0f, -120.0f);
    float maxZ = std::min(mx.z - 20.0f, 120.0f);
    if (minX >= maxX) { minX = -60.0f; maxX = 60.0f; }
    if (minZ >= maxZ) { minZ = -60.0f; maxZ = 60.0f; }

    float cruiseY = std::max(mx.y + 8.0f, 45.0f);

    Drone lead;
    lead.name = "drone1";
    lead.color = glm::vec3(0.12f, 0.16f, 0.18f);
    lead.speed = 10.0f;
    lead.p.pos = glm::vec3(minX, cruiseY, minZ);
    lead.route = {
        glm::vec3(minX, cruiseY, minZ),
        glm::vec3(maxX, cruiseY, maxZ)
    };
    lead.routeEnabled = true;

    Drone wing;
    wing.name = "drone2";
    wing.color = glm::vec3(0.05f, 0.35f, 0.55f);
    wing.speed = 8.0f;
    wing.p.pos = glm::vec3(maxX, cruiseY + 8.0f, minZ);
    wing.route = {
        glm::vec3(maxX, cruiseY + 8.0f, minZ),
        glm::vec3(minX, cruiseY + 8.0f, maxZ)
    };
    wing.routeEnabled = true;

    return {lead, wing};
}

// ===================== Rendering helpers =====================
// Render all cubes (objects + drone) with object shader
static void renderSceneCubes(GLuint shader, GLuint cubeVAO,
                             const std::vector<std::unique_ptr<Object3D>>& objects,
                             const std::vector<Drone>& drones,
                             const DroneModel* droneModel,
                             float propAngle,
                             const CityModel* city,
                             const glm::mat4* cityTransform,
                             const CityModel* terrain,
                             const glm::mat4* terrainTransform) {
    for (const auto& object : objects) object->draw(shader, cubeVAO);

    for (const auto& drone : drones) {
        if (droneModel && droneModel->loaded) {
            drawDroneModel(drone, *droneModel, propAngle, shader);
        } else {
            drawDroneBox(drone.p, drone.bodyScale, shader, cubeVAO, drone.color);
        }
    }

    if (city && cityTransform && city->valid()) {
        for (const auto& part : city->parts) {
            drawMesh(part.mesh, shader, *cityTransform, part.color, part.textureId, part.hasTexture);
        }
    }

    if (terrain && terrainTransform && terrain->valid()) {
        const glm::vec3 terrainDebugColor(0.05f, 0.75f, 0.05f);
        for (const auto& part : terrain->parts) {
            drawMesh(part.mesh, shader, *terrainTransform, terrainDebugColor, 0, false);
        }
    }
}

static void renderGridLines(GLuint shader, GLuint gridVAO, int gridVertexCount,
                            const glm::mat4& proj, const glm::mat4& view) {

    // WICHTIG: Tiefentest deaktivieren, damit die Linien ÜBER der Bodenfläche gezeichnet werden,
    // ohne Z-Fighting, oder verwenden Sie einen Polygon-Offset.
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_POLYGON_OFFSET_LINE);
    glPolygonOffset(-2.0f, -2.0f);
    glDepthMask(GL_FALSE);
    glDepthFunc(GL_LEQUAL);
    glLineWidth(1.0f);
    
    glUseProgram(shader);
    glUniformMatrix4fv(glGetUniformLocation(shader, "uProjection"), 1, GL_FALSE, glm::value_ptr(proj));
    glUniformMatrix4fv(glGetUniformLocation(shader, "uView"), 1, GL_FALSE, glm::value_ptr(view));
    
    // Model-Matrix muss auf Identity gesetzt werden, da die Linien im Weltraum sind.
    glm::mat4 model = glm::mat4(1.0f);
    glUniformMatrix4fv(glGetUniformLocation(shader, "uModel"), 1, GL_FALSE, glm::value_ptr(model));
    glUniform1i(glGetUniformLocation(shader, "uIsPoint"), 0);

    // Setzen Sie eine Gitterfarbe (heller als der Boden)
    glm::vec3 lineColor = glm::vec3(0.5f, 0.5f, 0.5f); 
    glUniform3fv(glGetUniformLocation(shader, "uColor"), 1, glm::value_ptr(lineColor));

    glBindVertexArray(gridVAO);
    glDrawArrays(GL_LINES, 0, gridVertexCount); // Zeichnet die Linien!
    glBindVertexArray(0);

    glLineWidth(1.0f);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_TRUE);
    glDisable(GL_POLYGON_OFFSET_LINE);
}

// Render full grid using grid shader + VAO
static void renderGrid(GLuint shader, GLuint gridVAO, int gridVertexCount,
                       const glm::mat4& proj, const glm::mat4& view,
                       const glm::mat4& lightSpace, const glm::vec3& lightPos,
                       const ShadowMap& sm,
                       const glm::vec3& lineColor) {
    glUseProgram(shader);
    glUniformMatrix4fv(glGetUniformLocation(shader, "uProjection"), 1, GL_FALSE, glm::value_ptr(proj));
    glUniformMatrix4fv(glGetUniformLocation(shader, "uView"), 1, GL_FALSE, glm::value_ptr(view));

    glUniformMatrix4fv(glGetUniformLocation(shader, "uLightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(lightSpace));
    glUniform3fv(glGetUniformLocation(shader, "uLightPos"), 1, glm::value_ptr(lightPos));

    // Light Space Matrix (needed to calculate fragment position in light space)
    glUniformMatrix4fv(glGetUniformLocation(shader, "uLightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(lightSpace));

    glUniform3fv(glGetUniformLocation(shader, "uGroundColor"), 1, glm::value_ptr(lineColor));

    sm.bindForRead(shader, 0); // Bind shadow map to texture unit 0

    glBindVertexArray(gridVAO);
    glDrawArrays(GL_LINES, 0, gridVertexCount);
    glBindVertexArray(0);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);

    glUseProgram(0);
}

static void renderGround(GLuint shader, GLuint groundVAO,
                       const glm::mat4& proj, const glm::mat4& view,
                       const glm::mat4& lightSpace, const glm::vec3& lightPos,
                       const ShadowMap& sm // Die ShadowMap-Struktur
                       ) {
    glUseProgram(shader);

    // Camera Matrices
    glUniformMatrix4fv(glGetUniformLocation(shader, "uProjection"), 1, GL_FALSE, glm::value_ptr(proj));
    glUniformMatrix4fv(glGetUniformLocation(shader, "uView"), 1, GL_FALSE, glm::value_ptr(view));

    // Shadow Mapping Uniforms
    glUniformMatrix4fv(glGetUniformLocation(shader, "uLightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(lightSpace));
    glUniform3fv(glGetUniformLocation(shader, "uLightPos"), 1, glm::value_ptr(lightPos));

    // Bodenfarbe setzen
    glm::vec3 groundColor = glm::vec3(1.0f, 1.0f, 1.0f);
    glUniform3fv(glGetUniformLocation(shader, "uGroundColor"), 1, glm::value_ptr(groundColor));

    // Shadow Map Binden (Textur-Einheit 0)
    sm.bindForRead(shader, 0);

    glBindVertexArray(groundVAO);
    // WICHTIG: Draw call für Dreiecke
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    // Aufräumen
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

// One-pass depth render for shadow map
static void shadowPass(const ShadowMap& sm, GLuint shadowShader,
                       const glm::mat4& lightSpace,
                       GLuint cubeVAO,
                       const std::vector<std::unique_ptr<Object3D>>& objects,
                       const std::vector<Drone>& drones,
                       const DroneModel* droneModel,
                       float propAngle,
                       const CityModel* city,
                       const glm::mat4* cityTransform,
                       const CityModel* terrain,
                       const glm::mat4* terrainTransform) {
    sm.bindForWrite();
    glUseProgram(shadowShader);
    glUniformMatrix4fv(glGetUniformLocation(shadowShader, "uLightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(lightSpace));

    // Draw objects using the same VAO; shader uses uModel only
    // Reuse the same routine which sets uModel for each object
    // but bind shadow shader instead of object shader.
    renderSceneCubes(shadowShader, cubeVAO, objects, drones, droneModel, propAngle,
                     city, cityTransform, terrain, terrainTransform);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// Main pass (color) with shadow sampling
static void mainPass(GLuint objectShader, GLuint cubeVAO,
                     const glm::mat4& proj, const glm::mat4& view,
                     const glm::mat4& lightSpace, const glm::vec3& lightPos,
                     const ShadowMap& sm,
                     const std::vector<std::unique_ptr<Object3D>>& objects,
                     const std::vector<Drone>& drones,
                     const DroneModel* droneModel,
                     float propAngle,
                     const CityModel* city,
                     const glm::mat4* cityTransform,
                     const CityModel* terrain,
                     const glm::mat4* terrainTransform) {
    // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    glUseProgram(objectShader);
    glUniformMatrix4fv(glGetUniformLocation(objectShader, "uProjection"), 1, GL_FALSE, glm::value_ptr(proj));
    glUniformMatrix4fv(glGetUniformLocation(objectShader, "uView"), 1, GL_FALSE, glm::value_ptr(view));
    glUniform3fv(glGetUniformLocation(objectShader, "uLightPos"), 1, glm::value_ptr(lightPos));
    glUniformMatrix4fv(glGetUniformLocation(objectShader, "uLightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(lightSpace));

    sm.bindForRead(objectShader, 0);

    renderSceneCubes(objectShader, cubeVAO, objects, drones, droneModel, propAngle,
                     city, cityTransform, terrain, terrainTransform);

    glUseProgram(0);
    // glDisable(GL_DEPTH_TEST);
}

// ===================== Application (window, loop) =====================
int main() {
    std::cout << "=== UAM Simulator — modularized ===\n";

    // ---- Create window and GL context
    sf::ContextSettings settings;
    settings.depthBits = 24;
    settings.stencilBits = 8;
    settings.antiAliasingLevel = 4;
    settings.majorVersion = 4;
    settings.minorVersion = 1;
    settings.attributeFlags = sf::ContextSettings::Core;

    sf::RenderWindow window(
        sf::VideoMode({1200, 900}),
        "UAM Simulator",
        sf::Style::Default,
        sf::State::Windowed,
        settings
    );
    if (!window.setActive(true)) std::cerr << "Warning: Could not activate OpenGL context!\n";

    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) { std::cerr << "GLEW init failed\n"; return -1; }

    window.setVerticalSyncEnabled(true);
    std::cout << "OpenGL: " << glGetString(GL_VERSION) << " | Renderer: " << glGetString(GL_RENDERER) << "\n";

    // ---- Compile shaders
    GLuint gridShader   = glutil::createShaderProgram("shaders/grid.vert", "shaders/grid.frag");
    GLuint objectShader = glutil::createShaderProgram("shaders/object.vert", "shaders/object.frag");
    GLuint shadowShader = glutil::createShaderProgram("shaders/shadow.vert", "shaders/shadow.frag");
    GLuint pointLineShaderProgram = glutil::createShaderProgram("shaders/point_line.vert", "shaders/point_line.frag");
    if (!gridShader || !objectShader || !shadowShader || !pointLineShaderProgram) return -1;

    const std::string quadVertSrc = R"(
        #version 330 core
        layout(location = 0) in vec2 aPos;
        layout(location = 1) in vec2 aTex;
        out vec2 vTex;
        void main() {
            vTex = aTex;
            gl_Position = vec4(aPos, 0.0, 1.0);
        }
    )";
    const std::string quadFragSrc = R"(
        #version 330 core
        in vec2 vTex;
        out vec4 FragColor;
        uniform sampler2D uTex;
        void main() {
            FragColor = texture(uTex, vTex);
        }
    )";
    GLuint cameraOverlayShader = glutil::createShaderProgramFromSource(quadVertSrc, quadFragSrc);
    if (!cameraOverlayShader) return -1;

    const std::string frameFragSrc = R"(
        #version 330 core
        out vec4 FragColor;
        uniform vec3 uColor;
        void main() {
            FragColor = vec4(uColor, 1.0);
        }
    )";
    GLuint cameraFrameShader = glutil::createShaderProgramFromSource(quadVertSrc, frameFragSrc);
    if (!cameraFrameShader) return -1;

    // ---- Create geometry
    GLuint gridVAO = 0, gridVBO = 0; int gridVertexCount = 0;

    GLuint cubeVAO = 0, cubeVBO = 0;
    createCubeVAO(cubeVAO, cubeVBO);

    GLuint sensorVAO = 0, sensorVBO = 0; 
    createSensorVAO(sensorVAO, sensorVBO);

    GLuint groundVAO = 0, groundVBO = 0;

    GLuint quadVAO = 0, quadVBO = 0;
    createScreenQuadVAO(quadVAO, quadVBO);

    auto sensorCfg = loadSensorConfig("sensors.yaml");

    const float modelScale = 0.001f; // mm -> m
    DroneModel droneModel = loadDroneModel("model", modelScale);

    CityModel city = loadCityModel("map/hh_clip.obj", 1.0f);
    const float cityYawDeg = 0.0f; // rotate clockwise if model appears left-rotated
    const bool cityMirrorX = true; // flip X if the OBJ appears mirrored
    glm::mat4 cityTransform = glm::rotate(glm::mat4(1.0f), glm::radians(cityYawDeg), glm::vec3(0.f, 1.f, 0.f));
    if (cityMirrorX) {
        cityTransform = cityTransform * glm::scale(glm::mat4(1.0f), glm::vec3(-1.f, 1.f, 1.f));
    }
    if (!city.valid()) {
        std::cerr << "City clip not loaded (map/hh_clip.obj).\n";
    }
    GeoReference geoRef;
    if (city.utmCenter) {
        geoRef.valid = true;
        geoRef.utmCenter = *city.utmCenter;
        geoRef.mapLocalToSim = cityTransform;
        geoRef.simToMapLocal = glm::inverse(cityTransform);
        std::cout << "World CRS: " << geoRef.crs << " center=("
                  << geoRef.utmCenter.x << ", " << geoRef.utmCenter.y << ")\n";
    }

    CityModel terrain = loadCityModel("map/surface_ground.obj", 1.0f);
    if (!terrain.valid()) {
        terrain = loadCityModel("map/surface.obj", 1.0f);
    }
    if (!terrain.valid()) {
        terrain = loadCityModel("map/terrain.obj", 1.0f);
    }
    glm::mat4 terrainTransform = cityTransform;
    if (terrain.valid() && terrain.hasBounds) {
        // Keep converted terrain heights as-is (no auto-lift), and use a neutral debug color.
        for (auto& part : terrain.parts) {
            part.color = glm::vec3(0.45f, 0.45f, 0.45f);
            part.hasTexture = false;
            if (part.textureId) {
                glDeleteTextures(1, &part.textureId);
                part.textureId = 0;
            }
        }

        std::cout << "Terrain bounds before transform: min(" << terrain.boundsMin.x << ", " << terrain.boundsMin.y << ", " << terrain.boundsMin.z
                  << ") max(" << terrain.boundsMax.x << ", " << terrain.boundsMax.y << ", " << terrain.boundsMax.z << ")\n";
    }
    if (!terrain.valid()) {
        std::cerr << "Surface model not loaded (expected map/surface_ground.obj, map/surface.obj or map/terrain.obj).\n";
    }

    CameraFBO droneCamFbo;
    droneCamFbo.init(sensorCfg.camera.width, sensorCfg.camera.height);
    

    // ---- Shadow map
    ShadowMap shadow; shadow.init();

    // ---- Static GL state
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // ---- Scene objects
    auto objects = loadObjectsOrDefault("config.yaml");
    auto world = buildWorldGeometry(objects, &city, &cityTransform, &terrain, &terrainTransform);

    float sceneHalfExtent = 250.0f;
    if (world.hasBounds) {
        sceneHalfExtent = std::max({std::abs(world.boundsMin.x), std::abs(world.boundsMax.x),
                                    std::abs(world.boundsMin.z), std::abs(world.boundsMax.z),
                                    250.0f}) + 50.0f;
    }
    createGridVAO(gridVAO, gridVBO, gridVertexCount, static_cast<int>(std::ceil(sceneHalfExtent)));
    createGroundVAO(groundVAO, groundVBO, sceneHalfExtent);

    auto drones = loadDronesFromConfig("config.yaml", geoRef.valid ? &geoRef : nullptr);
    if (drones.empty()) drones = createDefaultDrones(world);
    if (drones.empty()) drones.push_back(Drone{});

    // ---- Lighting
    glm::vec3 lightPos(5.0f, 50.0f, 3.0f);
    bool enableShadows = true;

    // ---- Camera & modes
    Camera cam;
    float yawSpeed = 60.f;
    bool enableMouseLook = false;
    bool followDrone = true;
    sf::Vector2i windowCenter(window.getSize().x / 2, window.getSize().y / 2);

    float propAngle = 0.0f;
    const float propSpeed = glm::radians(1800.0f); // deg/s

    bool enableLidar = false;
    bool enableRadar = false;
    bool enableLightSource = false;
    bool enableCamera = true;
    bool showLidarInCam = false;
    bool showRadarInCam = false;
    RadarParams radarCfg;
    glm::vec3 prevDronePos = drones.front().p.pos, droneVel{0,0,0};
    bool prevPosValid = false;

    // ---- Lidar config
    const int   beamsH   = sensorCfg.lidar.beamsH;
    const int   beamsV   = sensorCfg.lidar.beamsV;
    const float fovH_deg = sensorCfg.lidar.fovH_deg;
    const float fovV_deg = sensorCfg.lidar.fovV_deg;
    const float lidarMax = sensorCfg.lidar.maxRange;
    const std::string lidarOutputDir = sensorCfg.lidar.outputDir;
    int lidarFrameId = 0;
    const float lidarFps = sensorCfg.lidar.fps;
    const float lidarPeriod = 1.0f / std::max(lidarFps, 0.1f);
    float lidarWriteAccumulator = lidarPeriod;
    std::vector<Hit3D> lidarHitsCache;
    std::vector<glm::vec3> lidarPointsCache;

    radarCfg.beamsH = sensorCfg.radar.beamsH;
    radarCfg.beamsV = sensorCfg.radar.beamsV;
    radarCfg.fovH = sensorCfg.radar.fovH_deg;
    radarCfg.fovV = sensorCfg.radar.fovV_deg;
    radarCfg.maxR = sensorCfg.radar.maxRange;
    radarCfg.minR = sensorCfg.radar.minRange;
    radarCfg.snr0 = sensorCfg.radar.snr0;
    radarCfg.snrMin = sensorCfg.radar.snrMin;

    const std::string radarOutputDir = sensorCfg.radar.outputDir;
    int radarFrameId = 0;
    const float radarFps = sensorCfg.radar.fps;
    const float radarPeriod = 1.0f / std::max(radarFps, 0.1f);
    float radarWriteAccumulator = radarPeriod;
    std::vector<RadarDet> radarDetsCache;
    std::vector<glm::vec3> radarPointsCache;

    const std::string cameraOutputDir = sensorCfg.camera.outputDir;
    int cameraFrameId = 0;
    const float cameraFps = sensorCfg.camera.fps;
    const float cameraPeriod = 1.0f / cameraFps;
    float cameraRenderAccumulator = 0.0f;

    // ---- Timing
    sf::Clock deltaClock, fpsCounter;
    int frameCount = 0;
    float dtSmooth = 1.f/60.f;

    // ---- Events handlers (use your window.handleEvents helper)
    const auto onClose = [&](const sf::Event::Closed&) { window.close(); };
    const auto onKeyPressed = [&](const sf::Event::KeyPressed& keyPressed) {
        using sc = sf::Keyboard::Scancode;
        if (keyPressed.scancode == sc::M) {
            enableMouseLook = !enableMouseLook;
            window.setMouseCursorVisible(!enableMouseLook);
            if (enableMouseLook) sf::Mouse::setPosition(windowCenter, window);
            std::cout << "Mouse look: " << (enableMouseLook ? "ON" : "OFF") << "\n";
        } else if (keyPressed.scancode == sc::B) {
            enableLightSource = !enableLightSource;
            std::cout << "Light source cube: " << (enableLightSource ? "ON" : "OFF") << "\n";
        } else if (keyPressed.scancode == sc::H) {
            enableShadows = !enableShadows;
            std::cout << "Shadows: " << (enableShadows ? "ON" : "OFF") << "\n";
        } else if (keyPressed.scancode == sc::F) {
            followDrone = !followDrone;
            std::cout << "Follow drone: " << (followDrone ? "ON" : "OFF") << "\n";
        } else if (keyPressed.scancode == sc::Escape) {
            window.close();
        } else if (keyPressed.scancode == sc::L) {
            enableLidar = !enableLidar;
            std::cout << "Lidar: " << (enableLidar ? "ON" : "OFF") << "\n";
        } else if (keyPressed.scancode == sc::R) {
            enableRadar = !enableRadar;
            std::cout << "Radar: " << (enableRadar ? "ON" : "OFF") << "\n";
        } else if (keyPressed.scancode == sc::P) {
            enableCamera = !enableCamera;
            std::cout << "Camera: " << (enableCamera ? "ON" : "OFF") << "\n";
        } else if (keyPressed.scancode == sc::K) {
            showLidarInCam = !showLidarInCam;
            std::cout << "Camera LiDAR points: " << (showLidarInCam ? "ON" : "OFF") << "\n";
        } else if (keyPressed.scancode == sc::T) {
            showRadarInCam = !showRadarInCam;
            std::cout << "Camera RADAR points: " << (showRadarInCam ? "ON" : "OFF") << "\n";
        }

        // Light control (arrow keys + PageUp/Down)
        float lightSpeed = 0.5f;
        if (keyPressed.scancode == sc::Up)       lightPos.z -= lightSpeed;
        if (keyPressed.scancode == sc::Down)     lightPos.z += lightSpeed;
        if (keyPressed.scancode == sc::Left)     lightPos.x -= lightSpeed;
        if (keyPressed.scancode == sc::Right)    lightPos.x += lightSpeed;
        if (keyPressed.scancode == sc::PageUp)   lightPos.y += lightSpeed;
        if (keyPressed.scancode == sc::PageDown) lightPos.y -= lightSpeed;
    };
    const auto onMouseMoved = [&](const sf::Event::MouseMoved& e) {
        if (enableMouseLook && !followDrone) {
            float xoffset = static_cast<float>(e.position.x - windowCenter.x);
            float yoffset = static_cast<float>(windowCenter.y - e.position.y);
            sf::Mouse::setPosition(windowCenter, window);

            float sensitivity = 0.1f;
            cam.yaw   += xoffset * sensitivity;
            cam.pitch += yoffset * sensitivity;
            cam.pitch = glm::clamp(cam.pitch, -89.f, 89.f);

            cam.front = cam.calcFront(cam.yaw, cam.pitch);
            cam.right = glm::normalize(glm::cross(cam.front, cam.up));
        }
    };

    std::cout << "\n=== Controls ===\n"
              << "WASD: move, Q/E: yaw (drone), C/V: up/down\n"
              << "F: follow cam, M: mouse look (free cam), L: LiDAR, R: RADAR, P: camera, K: cam LiDAR, T: cam RADAR, H: shadows, ESC: quit\n"
              << "Arrows/PgUp/PgDn: move light\n";

    // ---- Main loop
    while (window.isOpen()) {
        float dt = deltaClock.restart().asSeconds();
        dt = std::min(dt, 0.1f);
        dtSmooth = 0.9f * dtSmooth + 0.1f * dt;

        window.handleEvents(
            onClose,
            [](const sf::Event::Resized&) {},
            onKeyPressed,
            [](const sf::Event::KeyReleased&) {},
            [](const sf::Event::MouseButtonPressed&) {},
            [](const sf::Event::MouseButtonReleased&) {},
            onMouseMoved,
            [](const sf::Event::MouseWheelScrolled&) {}
        );

        // ---- Update camera/drone
        if (!followDrone) {
            // Free camera movement and yaw (Q/E) like before
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::E)) {
                cam.yaw += yawSpeed * dtSmooth;
                cam.front = cam.calcFront(cam.yaw, cam.pitch);
                cam.right = glm::normalize(glm::cross(cam.front, cam.up));
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Q)) {
                cam.yaw -= yawSpeed * dtSmooth;
                cam.front = cam.calcFront(cam.yaw, cam.pitch);
                cam.right = glm::normalize(glm::cross(cam.front, cam.up));
            }
            updateFreeCamera(cam, dtSmooth, 5.0f);

            for (auto& droneUnit : drones) {
                glm::vec3 previous = droneUnit.p.pos;
                updateDroneRoute(droneUnit, dtSmooth);
                resolveDroneCollision(droneUnit, previous, world);
            }
            Drone& primaryDrone = drones.front();
            if (!prevPosValid) { prevPosValid = true; prevDronePos = primaryDrone.p.pos; }
            droneVel = (primaryDrone.p.pos - prevDronePos) / std::max(dtSmooth, 1e-4f);
            prevDronePos = primaryDrone.p.pos;
        } else {
            bool manualInput = hasManualDroneInput();
            for (size_t i = 0; i < drones.size(); ++i) {
                glm::vec3 previous = drones[i].p.pos;
                if (i == 0 && manualInput) {
                    updateDroneManual(drones[i], dtSmooth);
                } else {
                    updateDroneRoute(drones[i], dtSmooth);
                }
                resolveDroneCollision(drones[i], previous, world);
            }

            Drone& primaryDrone = drones.front();
            if (!prevPosValid) { prevPosValid = true; prevDronePos = primaryDrone.p.pos; }
            droneVel = (primaryDrone.p.pos - prevDronePos) / std::max(dtSmooth, 1e-4f);
            prevDronePos = primaryDrone.p.pos;

            updateFollowCamera(primaryDrone, cam);
        }

        // ---- Matrices
        glClearColor(0.8f, 0.9f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        float aspect = static_cast<float>(window.getSize().x) / static_cast<float>(window.getSize().y);
        float viewFar = std::max(1000.0f, sceneHalfExtent * 4.0f);
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), aspect, 0.2f, viewFar);
        glm::mat4 view = glm::lookAt(cam.pos, cam.pos + cam.front, cam.up);

        // ---- Light space (orthographic, directional-like)
        float nearPlane = 0.1f, farPlane = std::max(200.0f, sceneHalfExtent * 3.0f);
        float lightRange = std::max(80.0f, sceneHalfExtent * 1.5f);
        glm::mat4 lightProj = glm::ortho(-lightRange, lightRange, -lightRange, lightRange, nearPlane, farPlane);
        glm::mat4 lightView = glm::lookAt(lightPos, glm::vec3(0.0f, 0.0f, 0.0f), cam.up);
        glm::mat4 lightSpace = lightProj * lightView;

        // ---- Sensors: compute at configured sensor FPS, reuse cached hits for rendering.
        bool lidarActive = enableLidar || showLidarInCam;
        if (lidarActive) {
            lidarWriteAccumulator += dtSmooth;
            if (lidarHitsCache.empty() || lidarWriteAccumulator >= lidarPeriod) {
                lidarWriteAccumulator = std::fmod(lidarWriteAccumulator, lidarPeriod);
                lidarHitsCache = simulateLidar3D(drones.front().p, world, beamsH, beamsV, fovH_deg, fovV_deg, lidarMax);
                lidarPointsCache.clear();
                lidarPointsCache.reserve(lidarHitsCache.size());
                for (const auto& hit : lidarHitsCache) {
                    if (hit.ok) lidarPointsCache.push_back(hit.point);
                }
                if (enableLidar) {
                    writeLidarFrameYaml(lidarOutputDir, lidarFrameId++, lidarHitsCache, beamsH, beamsV, fovH_deg, fovV_deg);
                }
            }
        } else {
            lidarWriteAccumulator = lidarPeriod;
            lidarHitsCache.clear();
            lidarPointsCache.clear();
        }

        bool radarActive = enableRadar || showRadarInCam;
        if (radarActive) {
            radarWriteAccumulator += dtSmooth;
            if (radarDetsCache.empty() || radarWriteAccumulator >= radarPeriod) {
                radarWriteAccumulator = std::fmod(radarWriteAccumulator, radarPeriod);
                radarDetsCache = simulateRadar3D(drones.front().p, droneVel, world, radarCfg);
                radarPointsCache.clear();
                radarPointsCache.reserve(radarDetsCache.size());
                for (const auto& det : radarDetsCache) {
                    if (det.ok) radarPointsCache.push_back(det.point);
                }
                if (enableRadar) {
                    writeRadarFrameYaml(radarOutputDir, radarFrameId++, radarDetsCache, radarCfg);
                }
            }
        } else {
            radarWriteAccumulator = radarPeriod;
            radarDetsCache.clear();
            radarPointsCache.clear();
        }

        // ---- Pass 1: Shadow depth
        if (enableShadows) {
            shadowPass(shadow, shadowShader, lightSpace, cubeVAO, objects, drones, &droneModel, propAngle,
                       &city, &cityTransform, &terrain, &terrainTransform);
        }

        // ---- Pass 2: Grid + scene
        glViewport(0, 0, window.getSize().x, window.getSize().y);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Grid (no lighting)
        
        renderGround(gridShader, groundVAO, projection, view, lightSpace, lightPos, shadow);
        
        // Opaque scene with shadow map
        mainPass(objectShader, cubeVAO, projection, view, lightSpace, lightPos, shadow, objects, drones, &droneModel, propAngle,
             &city, &cityTransform, &terrain, &terrainTransform);
        
        renderGrid(gridShader, gridVAO, gridVertexCount, projection, view, lightSpace, lightPos, shadow,
               glm::vec3(0.5f, 0.5f, 0.5f));

        if (enableLightSource) {
            drawLightSource(lightPos, pointLineShaderProgram, cubeVAO, view, projection);
        }

        // ---- Drone camera: render to texture
        if (enableCamera) {
            cameraRenderAccumulator += dtSmooth;
            if (cameraRenderAccumulator >= cameraPeriod) {
                cameraRenderAccumulator = std::fmod(cameraRenderAccumulator, cameraPeriod);

                glm::vec3 camPos = drones.front().p.pos + glm::vec3(0.0f, 0.3f, 0.0f);
                glm::vec3 camFwd = forwardFrom(drones.front().p);
                glm::mat4 camView = glm::lookAt(camPos, camPos + camFwd, glm::vec3(0.0f, 1.0f, 0.0f));
                float camAspect = static_cast<float>(droneCamFbo.w) / static_cast<float>(droneCamFbo.h);
                glm::mat4 camProj = glm::perspective(glm::radians(sensorCfg.camera.fov_deg), camAspect, 0.2f, viewFar);

                droneCamFbo.bind();
                glClearColor(0.8f, 0.9f, 1.0f, 1.0f);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                renderGround(gridShader, groundVAO, camProj, camView, lightSpace, lightPos, shadow);
                mainPass(objectShader, cubeVAO, camProj, camView, lightSpace, lightPos, shadow, objects, drones, &droneModel, propAngle,
                         &city, &cityTransform, &terrain, &terrainTransform);
                renderGrid(gridShader, gridVAO, gridVertexCount, camProj, camView, lightSpace, lightPos, shadow,
                           glm::vec3(0.5f, 0.5f, 0.5f));

                if (showLidarInCam && !lidarPointsCache.empty()) {
                    drawPoints(lidarPointsCache, glm::vec3(1.0f, 0.0f, 0.0f), 3.0f, pointLineShaderProgram, sensorVAO, sensorVBO, camView, camProj, false);
                }
                if (showRadarInCam && !radarPointsCache.empty()) {
                    drawPoints(radarPointsCache, glm::vec3(0.0f, 1.0f, 0.0f), 3.0f, pointLineShaderProgram, sensorVAO, sensorVBO, camView, camProj, false);
                }

                droneCamFbo.unbind(window.getSize().x, window.getSize().y);

                saveCameraFrame(cameraOutputDir, cameraFrameId++, droneCamFbo);
            }

            unsigned windowW = window.getSize().x;
            unsigned windowH = window.getSize().y;
            unsigned overlayW = windowW / 4;
            unsigned overlayH = static_cast<unsigned>(overlayW * (static_cast<float>(droneCamFbo.h) / droneCamFbo.w));
            if (overlayH > windowH / 3) {
                overlayH = windowH / 3;
                overlayW = static_cast<unsigned>(overlayH * (static_cast<float>(droneCamFbo.w) / droneCamFbo.h));
            }

            const unsigned frameBorder = 6;
            drawCameraFrame(cameraFrameShader, quadVAO, windowW, windowH,
                            overlayW, overlayH, frameBorder, 16,
                            glm::vec3(0.0f, 0.0f, 0.0f));

            drawCameraOverlay(cameraOverlayShader, quadVAO, droneCamFbo.colorTex,
                              windowW, windowH, overlayW, overlayH, 16);
        } else {
            cameraRenderAccumulator = 0.0f;
        }

        // ---- Sensors (debug render)
        if (enableLidar) {
            if (!lidarPointsCache.empty()) {
                drawPoints(lidarPointsCache, glm::vec3(1.0f, 0.0f, 0.0f), 3.0f, pointLineShaderProgram, sensorVAO, sensorVBO, view, projection, true);
            }
        }

        if (enableRadar) {
            if (!radarPointsCache.empty()) {
                drawPoints(radarPointsCache, glm::vec3(0.0f, 1.0f, 0.0f), 2.0f, pointLineShaderProgram, sensorVAO, sensorVBO, view, projection, true);
            }
        }

        window.display();

        // ---- Prop animation
        propAngle += propSpeed * dtSmooth;
        if (propAngle > 2.0f * M_PI) propAngle -= 2.0f * M_PI;

        // ---- FPS
        frameCount++;
        if (fpsCounter.getElapsedTime().asSeconds() >= 1.0f) {
            const Drone& primaryDrone = drones.front();
            std::cout << "FPS: " << frameCount << " | Light: ("
                      << lightPos.x << ", " << lightPos.y << ", " << lightPos.z << ")"
                      << " | " << primaryDrone.name << " local: ("
                      << primaryDrone.p.pos.x << ", " << primaryDrone.p.pos.y << ", " << primaryDrone.p.pos.z << ")";
            if (geoRef.valid) {
                glm::dvec2 worldPos = geoRef.simToWorld(primaryDrone.p.pos);
                std::cout << " " << geoRef.crs << ": (" << worldPos.x << ", " << worldPos.y << ")";
            }
            std::cout << "\n";
            frameCount = 0;
            fpsCounter.restart();
        }
    }

    // ---- Cleanup
    glDeleteVertexArrays(1, &gridVAO);
    glDeleteBuffers(1, &gridVBO);
    glDeleteProgram(gridShader);
    
    glDeleteVertexArrays(1, &groundVAO);
    glDeleteBuffers(1, &groundVBO);

    glDeleteVertexArrays(1, &cubeVAO);
    glDeleteBuffers(1, &cubeVBO);
    glDeleteProgram(objectShader);
    glDeleteProgram(shadowShader);

    glDeleteVertexArrays(1, &sensorVAO);
    glDeleteBuffers(1, &sensorVBO);
    glDeleteProgram(pointLineShaderProgram);

    glDeleteVertexArrays(1, &quadVAO);
    glDeleteBuffers(1, &quadVBO);
    glDeleteProgram(cameraOverlayShader);
    glDeleteProgram(cameraFrameShader);

    glDeleteFramebuffers(1, &droneCamFbo.fbo);
    glDeleteTextures(1, &droneCamFbo.colorTex);
    glDeleteRenderbuffers(1, &droneCamFbo.depthRbo);

    city.destroy();
    droneModel.destroy();

    std::cout << "Exiting successfully!\n";
    return 0;
}

// ===================== Implementations left as-is =====================
static std::vector<glm::vec3> convexHullXZ(std::vector<glm::vec3> pts) {
    if (pts.size() <= 3) return pts;
    std::sort(pts.begin(), pts.end(), [](const glm::vec3& a, const glm::vec3& b){
        if (a.x != b.x) return a.x < b.x;
        return a.z < b.z;
    });
    auto cross2D = [](const glm::vec3& O, const glm::vec3& A, const glm::vec3& B){
        return (A.x - O.x)*(B.z - O.z) - (A.z - O.z)*(B.x - O.x);
    };
    std::vector<glm::vec3> H; H.reserve(pts.size()*2);
    for (const auto& p : pts){
        while (H.size() >= 2 && cross2D(H[H.size()-2], H.back(), p) <= 0.f) H.pop_back();
        H.push_back(p);
    }
    size_t t = H.size()+1;
    for (int i=(int)pts.size()-2; i>=0; --i){
        const auto& p = pts[i];
        while (H.size() >= t && cross2D(H[H.size()-2], H.back(), p) <= 0.f) H.pop_back();
        H.push_back(p);
    }
    H.pop_back();
    return H;
}
