#import <Cocoa/Cocoa.h>
#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>
#import <QuartzCore/QuartzCore.h>

#include "core/SimulatorCore.hpp"
#include "core/SceneMesh.hpp"
#include "core/SensorGeometry.hpp"
#include "metal/MetalMesh.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <cstring>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <glm/gtc/matrix_transform.hpp>
#include <simd/simd.h>
#include <vector>

struct MetalUniforms {
    matrix_float4x4 viewProjection;
    matrix_float4x4 model;
    matrix_float4x4 lightViewProjection;
    float pointSize;
    float shadowStrength;
};


struct RadarDetection {
    bool hit = false;
    float range = 0.0f;
    float radialVelocity = 0.0f;
    float azimuth = 0.0f;
    float elevation = 0.0f;
    float snr = 0.0f;
    glm::vec3 point{0.0f};
    glm::vec3 dir{1.0f, 0.0f, 0.0f};
    int objectId = -1;
};

static std::string timestampString() {
    auto now = std::chrono::system_clock::now();
    std::time_t nowC = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::tm tm{};
    localtime_r(&nowC, &tm);
    std::ostringstream ts;
    ts << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S")
       << '.' << std::setw(3) << std::setfill('0') << ms.count();
    return ts.str();
}

static bool ensureOutputDir(const std::string& outputDir, const char* label) {
    std::error_code ec;
    std::filesystem::create_directories(outputDir, ec);
    if (ec) {
        std::cerr << label << " export: could not create directory '" << outputDir << "' (" << ec.message() << ")\n";
        return false;
    }
    return true;
}

static void writeLidarFrameYaml(const std::string& outputDir,
                                int frameId,
                                const std::vector<uam::LidarHit>& hits,
                                const uam::SensorConfig& cfg) {
    if (hits.empty() || !ensureOutputDir(outputDir, "LiDAR")) return;

    std::ostringstream filename;
    filename << outputDir << "/lidar_frame_" << std::setw(6) << std::setfill('0') << frameId << ".yaml";
    std::ofstream out(filename.str());
    if (!out.is_open()) {
        std::cerr << "LiDAR export: could not open file " << filename.str() << "\n";
        return;
    }

    auto hitCount = std::count_if(hits.begin(), hits.end(), [](const uam::LidarHit& h) { return h.hit; });
    out << "timestamp: " << timestampString() << "\n";
    out << "frame_id: " << frameId << "\n";
    out << "hit_count: " << hitCount << "\n";
    out << "miss_count: " << (hits.size() - hitCount) << "\n";
    out << "beams_h: " << cfg.lidarBeamsH << "\n";
    out << "beams_v: " << cfg.lidarBeamsV << "\n";
    out << "fov_h_deg: " << cfg.lidarFovH << "\n";
    out << "fov_v_deg: " << cfg.lidarFovV << "\n";
    out << "max_range: " << cfg.lidarMaxRange << "\n";
    out << "beams:\n";
    out << std::fixed << std::setprecision(4);

    std::size_t idx = 0;
    int beamsH = std::max(1, cfg.lidarBeamsH);
    int beamsV = std::max(1, cfg.lidarBeamsV);
    float denomH = static_cast<float>(std::max(1, beamsH - 1));
    float denomV = static_cast<float>(std::max(1, beamsV - 1));
    for (int v = 0; v < beamsV; ++v) {
        float el = ((static_cast<float>(v) / denomV) - 0.5f) * glm::radians(cfg.lidarFovV);
        for (int h = 0; h < beamsH; ++h) {
            if (idx >= hits.size()) return;
            const auto& hit = hits[idx++];
            float az = ((static_cast<float>(h) / denomH) - 0.5f) * glm::radians(cfg.lidarFovH);
            out << "  - index: " << (idx - 1) << "\n";
            out << "    h_index: " << h << "\n";
            out << "    v_index: " << v << "\n";
            out << "    azimuth_deg: " << glm::degrees(az) << "\n";
            out << "    elevation_deg: " << glm::degrees(el) << "\n";
            out << "    distance: " << hit.range << "\n";
            out << "    hit: " << (hit.hit ? "true" : "false") << "\n";
            out << "    id: " << hit.objectId << "\n";
        }
    }
}

static void writeRadarFrameYaml(const std::string& outputDir,
                                int frameId,
                                const std::vector<RadarDetection>& detections,
                                const uam::SensorConfig& cfg) {
    if (detections.empty() || !ensureOutputDir(outputDir, "RADAR")) return;

    std::ostringstream filename;
    filename << outputDir << "/radar_frame_" << std::setw(6) << std::setfill('0') << frameId << ".yaml";
    std::ofstream out(filename.str());
    if (!out.is_open()) {
        std::cerr << "RADAR export: could not open file " << filename.str() << "\n";
        return;
    }

    auto hitCount = std::count_if(detections.begin(), detections.end(), [](const RadarDetection& d) { return d.hit; });
    out << "timestamp: " << timestampString() << "\n";
    out << "frame_id: " << frameId << "\n";
    out << "hit_count: " << hitCount << "\n";
    out << "miss_count: " << (detections.size() - hitCount) << "\n";
    out << "beams_h: " << cfg.radarBeamsH << "\n";
    out << "beams_v: " << cfg.radarBeamsV << "\n";
    out << "fov_h_deg: " << cfg.radarFovH << "\n";
    out << "fov_v_deg: " << cfg.radarFovV << "\n";
    out << "max_range: " << cfg.radarMaxRange << "\n";
    out << "min_range: " << cfg.radarMinRange << "\n";
    out << "snr0: " << cfg.radarSnr0 << "\n";
    out << "snr_min: " << cfg.radarSnrMin << "\n";
    out << "detections:\n";
    out << std::fixed << std::setprecision(4);

    std::size_t idx = 0;
    for (int v = 0; v < std::max(1, cfg.radarBeamsV); ++v) {
        for (int h = 0; h < std::max(1, cfg.radarBeamsH); ++h) {
            if (idx >= detections.size()) return;
            const auto& det = detections[idx++];
            out << "  - index: " << (idx - 1) << "\n";
            out << "    h_index: " << h << "\n";
            out << "    v_index: " << v << "\n";
            out << "    azimuth_deg: " << glm::degrees(det.azimuth) << "\n";
            out << "    elevation_deg: " << glm::degrees(det.elevation) << "\n";
            out << "    range: " << det.range << "\n";
            out << "    vr: " << det.radialVelocity << "\n";
            out << "    snr: " << det.snr << "\n";
            out << "    hit: " << (det.hit ? "true" : "false") << "\n";
            out << "    id: " << det.objectId << "\n";
        }
    }
}

static vector_float3 cameraForward(float yawRadians, float pitchRadians) {
    float cp = std::cos(pitchRadians);
    return simd_normalize(vector_float3{
        cp * std::cos(yawRadians),
        std::sin(pitchRadians),
        cp * std::sin(yawRadians)
    });
}

static vector_float3 cameraRight(float yawRadians) {
    vector_float3 f = cameraForward(yawRadians, 0.0f);
    return simd_normalize(simd_cross(f, vector_float3{0.0f, 1.0f, 0.0f}));
}

static vector_float3 droneForward(float yawDeg) {
    float yaw = glm::radians(yawDeg);
    return simd_normalize(vector_float3{std::cos(yaw), 0.0f, std::sin(yaw)});
}

static void yawPitchFromDirection(vector_float3 dir, float& yaw, float& pitch) {
    dir = simd_normalize(dir);
    pitch = std::asin(std::clamp(dir.y, -1.0f, 1.0f));
    yaw = std::atan2(dir.z, dir.x);
}

static vector_float3 toFloat3(const glm::vec3& v) {
    return vector_float3{v.x, v.y, v.z};
}


static matrix_float4x4 makePerspective(float fovyRadians, float aspect, float nearZ, float farZ) {
    float yScale = 1.0f / std::tan(fovyRadians * 0.5f);
    float xScale = yScale / aspect;
    float zScale = farZ / (nearZ - farZ);
    float wz = (nearZ * farZ) / (nearZ - farZ);

    return matrix_float4x4{{
        {xScale, 0.0f, 0.0f, 0.0f},
        {0.0f, yScale, 0.0f, 0.0f},
        {0.0f, 0.0f, zScale, -1.0f},
        {0.0f, 0.0f, wz, 0.0f},
    }};
}

static matrix_float4x4 makeOrthographic(float left, float right, float bottom, float top, float nearZ, float farZ) {
    return matrix_float4x4{{
        {2.0f / (right - left), 0.0f, 0.0f, 0.0f},
        {0.0f, 2.0f / (top - bottom), 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f / (nearZ - farZ), 0.0f},
        {-(right + left) / (right - left), -(top + bottom) / (top - bottom), nearZ / (nearZ - farZ), 1.0f},
    }};
}

static matrix_float4x4 makeLookAt(vector_float3 eye, vector_float3 target, vector_float3 up) {
    vector_float3 f = simd_normalize(target - eye);
    vector_float3 r = simd_normalize(simd_cross(f, up));
    vector_float3 u = simd_cross(r, f);

    return matrix_float4x4{{
        {r.x, u.x, -f.x, 0.0f},
        {r.y, u.y, -f.y, 0.0f},
        {r.z, u.z, -f.z, 0.0f},
        {-simd_dot(r, eye), -simd_dot(u, eye), simd_dot(f, eye), 1.0f},
    }};
}

static vector_float3 rotateYaw(vector_float3 p, float yawRadians) {
    float c = std::cos(yawRadians);
    float s = std::sin(yawRadians);
    return vector_float3{p.x * c - p.z * s, p.y, p.x * s + p.z * c};
}

static void addLine(std::vector<MetalVertex>& vertices, vector_float3 a, vector_float3 b, vector_float3 color) {
    vertices.push_back({a, color});
    vertices.push_back({b, color});
}

static std::vector<MetalVertex> makeViewportBorderVertices(double width, double height, double pixelThickness) {
    std::vector<MetalVertex> vertices;
    vertices.reserve(24);
    float bx = static_cast<float>(std::clamp(2.0 * pixelThickness / std::max(width, 1.0), 0.004, 0.18));
    float by = static_cast<float>(std::clamp(2.0 * pixelThickness / std::max(height, 1.0), 0.004, 0.18));
    vector_float3 c{0.02f, 0.03f, 0.04f};
    auto addQuad = [&](float x0, float y0, float x1, float y1) {
        vertices.push_back({vector_float3{x0, y0, 0.0f}, c});
        vertices.push_back({vector_float3{x1, y0, 0.0f}, c});
        vertices.push_back({vector_float3{x1, y1, 0.0f}, c});
        vertices.push_back({vector_float3{x0, y0, 0.0f}, c});
        vertices.push_back({vector_float3{x1, y1, 0.0f}, c});
        vertices.push_back({vector_float3{x0, y1, 0.0f}, c});
    };
    addQuad(-1.0f, -1.0f,  1.0f, -1.0f + by);
    addQuad(-1.0f,  1.0f - by, 1.0f, 1.0f);
    addQuad(-1.0f, -1.0f, -1.0f + bx, 1.0f);
    addQuad( 1.0f - bx, -1.0f, 1.0f, 1.0f);
    return vertices;
}

static glm::mat4 translateRotate(const glm::vec3& position, float yawDeg) {
    glm::mat4 m(1.0f);
    m = glm::translate(m, position);
    m = glm::rotate(m, glm::radians(-yawDeg), glm::vec3(0.0f, 1.0f, 0.0f));
    return m;
}

static matrix_float4x4 toMetalMatrix(const glm::mat4& m) {
    return matrix_float4x4{{
        {m[0][0], m[0][1], m[0][2], m[0][3]},
        {m[1][0], m[1][1], m[1][2], m[1][3]},
        {m[2][0], m[2][1], m[2][2], m[2][3]},
        {m[3][0], m[3][1], m[3][2], m[3][3]},
    }};
}

static matrix_float4x4 identityMatrix() {
    return matrix_identity_float4x4;
}

static constexpr NSUInteger kUniformStride = 256;
static constexpr NSUInteger kMaxUniformDraws = 256;
static constexpr NSUInteger kShadowMapSize = 2048;
static constexpr vector_float3 kSunDirection = vector_float3{-0.45f, 0.82f, -0.35f};

static void setUniforms(id<MTLBuffer> uniformBuffer,
                        NSUInteger& uniformCursor,
                        id<MTLRenderCommandEncoder> encoder,
                        matrix_float4x4 viewProjection,
                        matrix_float4x4 model,
                        float pointSize = 5.0f,
                        matrix_float4x4 lightViewProjection = matrix_identity_float4x4,
                        float shadowStrength = 0.0f) {
    if (!uniformBuffer) return;
    if (uniformCursor >= kMaxUniformDraws) uniformCursor = 0;

    MetalUniforms uniforms{viewProjection, model, lightViewProjection, pointSize, shadowStrength};
    NSUInteger offset = uniformCursor * kUniformStride;
    std::memcpy(static_cast<char*>([uniformBuffer contents]) + offset, &uniforms, sizeof(MetalUniforms));
    [encoder setVertexBuffer:uniformBuffer offset:offset atIndex:1];
    ++uniformCursor;
}


static void addCube(std::vector<MetalVertex>& vertices,
                    vector_float3 center,
                    vector_float3 halfExtents,
                    float yawRadians,
                    vector_float3 color) {
    vector_float3 local[8] = {
        {-halfExtents.x, -halfExtents.y, -halfExtents.z},
        { halfExtents.x, -halfExtents.y, -halfExtents.z},
        { halfExtents.x,  halfExtents.y, -halfExtents.z},
        {-halfExtents.x,  halfExtents.y, -halfExtents.z},
        {-halfExtents.x, -halfExtents.y,  halfExtents.z},
        { halfExtents.x, -halfExtents.y,  halfExtents.z},
        { halfExtents.x,  halfExtents.y,  halfExtents.z},
        {-halfExtents.x,  halfExtents.y,  halfExtents.z},
    };

    vector_float3 p[8];
    for (int i = 0; i < 8; ++i) p[i] = center + rotateYaw(local[i], yawRadians);

    auto tri = [&](int a, int b, int c, vector_float3 shade) {
        vertices.push_back({p[a], shade});
        vertices.push_back({p[b], shade});
        vertices.push_back({p[c], shade});
    };

    vector_float3 top = color;
    vector_float3 side = color * 0.82f;
    vector_float3 dark = color * 0.65f;

    tri(0, 1, 2, dark); tri(0, 2, 3, dark); // -Z
    tri(4, 6, 5, side); tri(4, 7, 6, side); // +Z
    tri(0, 3, 7, side); tri(0, 7, 4, side); // -X
    tri(1, 5, 6, side); tri(1, 6, 2, side); // +X
    tri(3, 2, 6, top);  tri(3, 6, 7, top);  // +Y
    tri(0, 4, 5, dark); tri(0, 5, 1, dark); // -Y
}



static glm::mat4 droneModelFixTransform() {
    glm::mat4 modelFix(1.0f);
    modelFix = glm::rotate(modelFix, glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    modelFix = glm::rotate(modelFix, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    return modelFix;
}

static glm::mat4 droneBodyFixTransform() {
    glm::mat4 bodyFix(1.0f);
    bodyFix = glm::rotate(bodyFix, glm::radians(270.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    bodyFix = glm::rotate(bodyFix, glm::radians(270.0f), glm::vec3(1.0f, 0.0f, 0.0f));
    return bodyFix;
}

static glm::mat4 droneModelFrame(const uam::Drone& drone) {
    return translateRotate(drone.position, drone.yawDeg) * droneModelFixTransform();
}

static glm::mat4 droneBodyTransform(const uam::Drone& drone) {
    return droneModelFrame(drone) * droneBodyFixTransform();
}

static std::array<glm::mat4, 4> dronePropTransforms(const uam::Drone& drone, float propAngle) {
    constexpr float propHalf = 0.212132f;
    constexpr float propLift = 0.06f;
    const std::array<glm::vec3, 4> offsets{{
        { propHalf, propLift, -propHalf },
        { propHalf, propLift,  propHalf },
        {-propHalf, propLift, -propHalf },
        {-propHalf, propLift,  propHalf },
    }};
    const std::array<float, 4> spins{{propAngle, -propAngle, -propAngle, propAngle}};
    std::array<glm::mat4, 4> transforms{};
    glm::mat4 frame = droneModelFrame(drone);
    for (std::size_t i = 0; i < transforms.size(); ++i) {
        glm::mat4 m = glm::translate(frame, offsets[i]);
        transforms[i] = glm::rotate(m, spins[i], glm::vec3(0.0f, 1.0f, 0.0f));
    }
    return transforms;
}

struct DroneRenderModel {
    MetalMesh body;
    MetalMesh propFL;
    MetalMesh propFR;
    MetalMesh propRL;
    MetalMesh propRR;
    MetalMesh propSweep;

    bool loaded() const { return body.valid(); }
};

struct DroneSensorModel {
    uam::SensorGeometry body;
    uam::SensorGeometry propFL;
    uam::SensorGeometry propFR;
    uam::SensorGeometry propRL;
    uam::SensorGeometry propRR;
    bool hasGeometry = false;

    bool loaded() const { return hasGeometry; }
};

static void buildSensorPart(uam::SensorGeometry& geometry, const uam::SceneMesh& mesh, int objectId) {
    geometry.clear();
    geometry.addMesh(mesh, objectId);
    geometry.build();
}

static uam::SceneMesh makePropSweepMesh() {
    uam::SceneMesh mesh;
    constexpr float radius = 0.215f;
    constexpr float inner = 0.018f;
    constexpr float y = 0.007f;
    constexpr float a0 = -0.20f;
    constexpr float a1 = 0.20f;

    auto p = [](float r, float a) {
        return glm::vec3(std::sin(a) * r, y, std::cos(a) * r);
    };
    auto push = [&](glm::vec3 pos, glm::vec3 color) {
        mesh.vertices.push_back({pos, color});
    };

    glm::vec3 hub{0.0f, y, 0.0f};
    glm::vec3 bright{0.95f, 0.95f, 0.42f};
    glm::vec3 blue{0.12f, 0.48f, 1.0f};
    push(p(inner, a0), bright);
    push(p(radius, 0.0f), blue);
    push(p(inner, a1), bright);
    push(hub, bright);
    push(p(inner, a0), bright);
    push(p(inner, a1), bright);

    mesh.boundsMin = glm::vec3(-radius, y, -radius);
    mesh.boundsMax = glm::vec3(radius, y, radius);
    mesh.hasBounds = true;
    return mesh;
}

static uam::SceneMesh makeGroundFillMesh(const uam::SceneMesh& cityMesh) {
    uam::SceneMesh mesh;
    if (!cityMesh.hasBounds) return mesh;

    constexpr float gridStep = 25.0f;
    constexpr float margin = 8.0f;
    float minX = cityMesh.boundsMin.x - margin;
    float maxX = cityMesh.boundsMax.x + margin;
    float minZ = cityMesh.boundsMin.z - margin;
    float maxZ = cityMesh.boundsMax.z + margin;
    float y = cityMesh.boundsMin.y - 0.04f;

    glm::vec3 color{0.24f, 0.42f, 0.24f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    auto push = [&](float x, float z) {
        mesh.vertices.push_back({glm::vec3{x, y, z}, color, normal, glm::vec2{0.0f}});
    };

    for (float x = minX; x < maxX; x += gridStep) {
        float x1 = std::min(x + gridStep, maxX);
        for (float z = minZ; z < maxZ; z += gridStep) {
            float z1 = std::min(z + gridStep, maxZ);
            push(x, z);
            push(x1, z);
            push(x, z1);
            push(x1, z);
            push(x1, z1);
            push(x, z1);
        }
    }

    if (!mesh.vertices.empty()) {
        mesh.boundsMin = glm::vec3{minX, y, minZ};
        mesh.boundsMax = glm::vec3{maxX, y, maxZ};
        mesh.hasBounds = true;
        mesh.materialRanges.push_back({0, mesh.vertices.size(), color, "", false});
    }
    return mesh;
}

static void drawMeshWithModel(id<MTLBuffer> uniformBuffer,
                              NSUInteger& uniformCursor,
                              id<MTLRenderCommandEncoder> encoder,
                              const MetalMesh& mesh,
                              matrix_float4x4 viewProjection,
                              const glm::mat4& model,
                              matrix_float4x4 lightViewProjection = matrix_identity_float4x4,
                              float shadowStrength = 0.0f) {
    if (!mesh.valid()) return;
    setUniforms(uniformBuffer, uniformCursor, encoder, viewProjection, toMetalMatrix(model), 5.0f, lightViewProjection, shadowStrength);
    mesh.draw(encoder);
}

static void drawDroneModel(id<MTLBuffer> uniformBuffer,
                           NSUInteger& uniformCursor,
                           id<MTLRenderCommandEncoder> encoder,
                           const DroneRenderModel& model,
                           const uam::Drone& drone,
                           float propAngle,
                           matrix_float4x4 viewProjection,
                           matrix_float4x4 lightViewProjection = matrix_identity_float4x4,
                           float shadowStrength = 0.0f) {
    if (!model.loaded()) return;

    drawMeshWithModel(uniformBuffer, uniformCursor, encoder, model.body, viewProjection, droneBodyTransform(drone), lightViewProjection, shadowStrength);

    const auto props = dronePropTransforms(drone, propAngle);
    const glm::mat4& fl = props[0];
    const glm::mat4& fr = props[1];
    const glm::mat4& rl = props[2];
    const glm::mat4& rr = props[3];

    drawMeshWithModel(uniformBuffer, uniformCursor, encoder, model.propFL, viewProjection, fl, lightViewProjection, shadowStrength);
    drawMeshWithModel(uniformBuffer, uniformCursor, encoder, model.propRR, viewProjection, rr, lightViewProjection, shadowStrength);
    drawMeshWithModel(uniformBuffer, uniformCursor, encoder, model.propFR, viewProjection, fr, lightViewProjection, shadowStrength);
    drawMeshWithModel(uniformBuffer, uniformCursor, encoder, model.propRL, viewProjection, rl, lightViewProjection, shadowStrength);

    drawMeshWithModel(uniformBuffer, uniformCursor, encoder, model.propSweep, viewProjection, fl, lightViewProjection, shadowStrength);
    drawMeshWithModel(uniformBuffer, uniformCursor, encoder, model.propSweep, viewProjection, rr, lightViewProjection, shadowStrength);
    drawMeshWithModel(uniformBuffer, uniformCursor, encoder, model.propSweep, viewProjection, fr, lightViewProjection, shadowStrength);
    drawMeshWithModel(uniformBuffer, uniformCursor, encoder, model.propSweep, viewProjection, rl, lightViewProjection, shadowStrength);
}

static bool rayAabb(const glm::vec3& ro, const glm::vec3& rd, const glm::vec3& mn, const glm::vec3& mx, float tMax) {
    float tmin = 0.001f;
    float tmax = tMax;
    for (int i = 0; i < 3; ++i) {
        float d = rd[i];
        if (std::abs(d) < 1e-6f) d = d < 0.0f ? -1e-6f : 1e-6f;
        float invD = 1.0f / d;
        float t0 = (mn[i] - ro[i]) * invD;
        float t1 = (mx[i] - ro[i]) * invD;
        if (invD < 0.0f) std::swap(t0, t1);
        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);
        if (tmax <= tmin) return false;
    }
    return true;
}

static bool rayDroneBounds(const glm::vec3& origin, const glm::vec3& dir, const uam::Drone& drone, float maxRange) {
    glm::mat4 inv = glm::inverse(droneModelFrame(drone));
    glm::vec3 ro = glm::vec3(inv * glm::vec4(origin, 1.0f));
    glm::vec3 rd = glm::normalize(glm::vec3(inv * glm::vec4(dir, 0.0f)));
    return rayAabb(ro, rd, glm::vec3(-0.52f, -0.20f, -0.52f), glm::vec3(0.52f, 0.20f, 0.52f), maxRange);
}

static bool raycastPart(const uam::SensorGeometry& geometry,
                        const glm::mat4& partTransform,
                        const glm::vec3& origin,
                        const glm::vec3& dir,
                        float& bestT,
                        int objectId) {
    if (geometry.empty()) return false;

    glm::mat4 inv = glm::inverse(partTransform);
    glm::vec3 localOrigin = glm::vec3(inv * glm::vec4(origin, 1.0f));
    glm::vec3 localDir = glm::normalize(glm::vec3(inv * glm::vec4(dir, 0.0f)));
    uam::SensorGeometry::RayHit hit = geometry.raycast(localOrigin, localDir, bestT);
    if (!hit.hit) return false;

    glm::vec3 localPoint = localOrigin + localDir * hit.t;
    glm::vec3 worldPoint = glm::vec3(partTransform * glm::vec4(localPoint, 1.0f));
    float worldT = glm::length(worldPoint - origin);
    if (worldT <= 0.001f || worldT >= bestT) return false;

    bestT = worldT;
    (void)objectId;
    return true;
}

static std::vector<uam::LidarHit> simulateLidarScene(const uam::SensorGeometry& staticGeometry,
                                                     const DroneSensorModel& droneGeometry,
                                                     const std::vector<uam::Drone>& drones,
                                                     std::size_t sensorIndex,
                                                     float propAngle,
                                                     const uam::SensorConfig& cfg) {
    std::vector<uam::LidarHit> hits;
    if (drones.empty()) return hits;

    int beamsH = std::max(1, cfg.lidarBeamsH);
    int beamsV = std::max(1, cfg.lidarBeamsV);
    hits.reserve(static_cast<std::size_t>(beamsH * beamsV));

    sensorIndex = std::min(sensorIndex, drones.size() - 1);
    const uam::Drone& sensorDrone = drones[sensorIndex];
    glm::vec3 origin = sensorDrone.position;
    float rayLimit = cfg.lidarMaxRange > 0.0f ? cfg.lidarMaxRange : 1000.0f;
    float baseYaw = glm::radians(sensorDrone.yawDeg);
    float denomH = static_cast<float>(std::max(1, beamsH - 1));
    float denomV = static_cast<float>(std::max(1, beamsV - 1));

    for (int v = 0; v < beamsV; ++v) {
        float el = ((static_cast<float>(v) / denomV) - 0.5f) * glm::radians(cfg.lidarFovV);
        constexpr float halfPi = 1.57079632679f;
        float pitch = glm::clamp(el, -halfPi + 1e-4f, halfPi - 1e-4f);
        float cosPitch = std::cos(pitch);

        for (int h = 0; h < beamsH; ++h) {
            float az = ((static_cast<float>(h) / denomH) - 0.5f) * glm::radians(cfg.lidarFovH);
            float yaw = baseYaw + az;
            glm::vec3 dir = glm::normalize(glm::vec3(std::cos(yaw) * cosPitch,
                                                     std::sin(pitch),
                                                     std::sin(yaw) * cosPitch));

            float bestT = rayLimit;
            int bestObjectId = -1;
            if (!staticGeometry.empty()) {
                auto staticHit = staticGeometry.raycast(origin, dir, bestT);
                if (staticHit.hit) {
                    bestT = staticHit.t;
                    bestObjectId = staticHit.objectId;
                }
            }

            if (droneGeometry.loaded()) {
                for (std::size_t i = 0; i < drones.size(); ++i) {
                    if (i == sensorIndex) continue;
                    const uam::Drone& targetDrone = drones[i];
                    if (!rayDroneBounds(origin, dir, targetDrone, bestT)) continue;

                    int droneObjectId = 10000 + static_cast<int>(i);
                    if (raycastPart(droneGeometry.body, droneBodyTransform(targetDrone), origin, dir, bestT, droneObjectId)) bestObjectId = droneObjectId;
                    const auto props = dronePropTransforms(targetDrone, propAngle);
                    if (raycastPart(droneGeometry.propFL, props[0], origin, dir, bestT, droneObjectId)) bestObjectId = droneObjectId;
                    if (raycastPart(droneGeometry.propFR, props[1], origin, dir, bestT, droneObjectId)) bestObjectId = droneObjectId;
                    if (raycastPart(droneGeometry.propRL, props[2], origin, dir, bestT, droneObjectId)) bestObjectId = droneObjectId;
                    if (raycastPart(droneGeometry.propRR, props[3], origin, dir, bestT, droneObjectId)) bestObjectId = droneObjectId;
                }
            }

            if (bestObjectId >= 0) hits.push_back({true, bestT, origin + dir * bestT, dir, bestObjectId});
            else hits.push_back({false, rayLimit, origin + dir * rayLimit, dir, -1});
        }
    }

    return hits;
}

static std::vector<RadarDetection> simulateRadarScene(const uam::SensorGeometry& staticGeometry,
                                                       const DroneSensorModel& droneGeometry,
                                                       const std::vector<uam::Drone>& drones,
                                                       std::size_t sensorIndex,
                                                       float propAngle,
                                                       const uam::SensorConfig& cfg) {
    std::vector<RadarDetection> detections;
    if (drones.empty()) return detections;

    int beamsH = std::max(1, cfg.radarBeamsH);
    int beamsV = std::max(1, cfg.radarBeamsV);
    detections.reserve(static_cast<std::size_t>(beamsH * beamsV));

    sensorIndex = std::min(sensorIndex, drones.size() - 1);
    const uam::Drone& sensorDrone = drones[sensorIndex];
    glm::vec3 origin = sensorDrone.position;
    glm::vec3 sensorVelocity = sensorDrone.velocity;
    float rayLimit = cfg.radarMaxRange > 0.0f ? cfg.radarMaxRange : 1000.0f;

    float yaw = glm::radians(sensorDrone.yawDeg);
    glm::vec3 forward = glm::normalize(glm::vec3(std::cos(yaw), 0.0f, std::sin(yaw)));
    glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)));
    glm::vec3 up = glm::normalize(glm::cross(right, forward));

    float denomH = static_cast<float>(std::max(1, beamsH - 1));
    float denomV = static_cast<float>(std::max(1, beamsV - 1));
    for (int v = 0; v < beamsV; ++v) {
        float el = ((static_cast<float>(v) / denomV) - 0.5f) * glm::radians(cfg.radarFovV);
        for (int h = 0; h < beamsH; ++h) {
            float az = ((static_cast<float>(h) / denomH) - 0.5f) * glm::radians(cfg.radarFovH);
            glm::vec3 dir = glm::normalize(forward + std::tan(az) * right + std::tan(el) * up);

            float bestT = rayLimit;
            int bestObjectId = -1;
            glm::vec3 bestTargetVelocity(0.0f);
            if (!staticGeometry.empty()) {
                auto staticHit = staticGeometry.raycast(origin, dir, bestT);
                if (staticHit.hit) {
                    bestT = staticHit.t;
                    bestObjectId = staticHit.objectId;
                    bestTargetVelocity = glm::vec3(0.0f);
                }
            }

            if (droneGeometry.loaded()) {
                for (std::size_t i = 0; i < drones.size(); ++i) {
                    if (i == sensorIndex) continue;
                    const uam::Drone& targetDrone = drones[i];
                    if (!rayDroneBounds(origin, dir, targetDrone, bestT)) continue;

                    int droneObjectId = 10000 + static_cast<int>(i);
                    bool partHit = false;
                    if (raycastPart(droneGeometry.body, droneBodyTransform(targetDrone), origin, dir, bestT, droneObjectId)) partHit = true;
                    const auto props = dronePropTransforms(targetDrone, propAngle);
                    if (raycastPart(droneGeometry.propFL, props[0], origin, dir, bestT, droneObjectId)) partHit = true;
                    if (raycastPart(droneGeometry.propFR, props[1], origin, dir, bestT, droneObjectId)) partHit = true;
                    if (raycastPart(droneGeometry.propRL, props[2], origin, dir, bestT, droneObjectId)) partHit = true;
                    if (raycastPart(droneGeometry.propRR, props[3], origin, dir, bestT, droneObjectId)) partHit = true;
                    if (partHit) {
                        bestObjectId = droneObjectId;
                        bestTargetVelocity = targetDrone.velocity;
                    }
                }
            }

            RadarDetection det;
            det.range = rayLimit;
            det.azimuth = az;
            det.elevation = el;
            det.point = origin + dir * rayLimit;
            det.dir = dir;
            det.objectId = -1;

            if (bestObjectId >= 0 && bestT >= cfg.radarMinRange) {
                float snr = cfg.radarSnr0 - 40.0f * std::log10(std::max(bestT, 1e-2f));
                if (snr >= cfg.radarSnrMin) {
                    det.hit = true;
                    det.range = bestT;
                    det.point = origin + dir * bestT;
                    det.objectId = bestObjectId;
                    det.snr = snr;
                    det.radialVelocity = glm::dot(bestTargetVelocity - sensorVelocity, dir);
                }
            }

            detections.push_back(det);
        }
    }

    return detections;
}


@interface InputMetalView : MTKView
@end

@implementation InputMetalView
- (BOOL)acceptsFirstResponder { return YES; }
- (BOOL)canBecomeKeyView { return YES; }
@end

@interface Renderer : NSObject <MTKViewDelegate>
- (instancetype)initWithView:(MTKView*)view;
- (void)handleKeyDown:(NSEvent*)event;
- (void)handleKeyUp:(NSEvent*)event;
@end

@implementation Renderer {
    id<MTLDevice> _device;
    id<MTLCommandQueue> _queue;
    id<MTLRenderPipelineState> _pipeline;
    id<MTLRenderPipelineState> _shadowPipeline;
    id<MTLDepthStencilState> _depthState;
    id<MTLDepthStencilState> _overlayDepthState;
    id<MTLTexture> _whiteTexture;
    id<MTLTexture> _shadowDepthTexture;
    id<MTLSamplerState> _defaultSampler;
    id<MTLSamplerState> _shadowSampler;
    id<MTLBuffer> _uniformBuffer;
    NSUInteger _uniformCursor;
    id<MTLBuffer> _gridBuffer;
    NSUInteger _gridVertexCount;
    id<MTLBuffer> _insetBackgroundBuffer;
    NSUInteger _insetBackgroundVertexCount;
    NSTextField* _hudLabel;
    float _hudFps;
    MetalMesh _cityMesh;
    MetalMesh _terrainMesh;
    DroneRenderModel _droneModel;
    DroneSensorModel _droneSensorModel;
    uam::SensorGeometry _sensorGeometry;
    std::size_t _selectedDrone;
    bool _onboardCamera;
    bool _insetCameraEnabled;
    bool _cameraRecordingEnabled;
    bool _lidarEnabled;
    bool _radarEnabled;
    float _lidarAccumulator;
    float _radarAccumulator;
    float _cameraOutputAccumulator;
    float _propAngle;
    int _lidarFrameId;
    int _radarFrameId;
    int _cameraFrameId;
    std::vector<uam::LidarHit> _lastLidarHits;
    std::vector<RadarDetection> _lastRadarDetections;
    std::vector<MetalVertex> _lidarPointVertices;
    std::vector<MetalVertex> _radarPointVertices;
    id<MTLTexture> _cameraOutputTexture;
    id<MTLTexture> _cameraOutputMsaaTexture;
    id<MTLTexture> _cameraOutputDepthTexture;
    unsigned _cameraOutputWidth;
    unsigned _cameraOutputHeight;
    id _keyDownMonitor;
    id _keyUpMonitor;
    std::array<bool, 256> _keys;
    bool _followDrone;
    vector_float3 _cameraPos;
    float _cameraYaw;
    float _cameraPitch;
    uam::SimulatorCore _sim;
    std::chrono::steady_clock::time_point _lastFrame;
}

- (instancetype)initWithView:(MTKView*)view {
    self = [super init];
    if (!self) return nil;

    _device = view.device;
    _queue = [_device newCommandQueue];
    MTLTextureDescriptor* shadowDesc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
                                                                                          width:kShadowMapSize
                                                                                         height:kShadowMapSize
                                                                                      mipmapped:NO];
    shadowDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
    shadowDesc.storageMode = MTLStorageModePrivate;
    _shadowDepthTexture = [_device newTextureWithDescriptor:shadowDesc];

    MTLSamplerDescriptor* shadowSamplerDesc = [[MTLSamplerDescriptor alloc] init];
    shadowSamplerDesc.minFilter = MTLSamplerMinMagFilterLinear;
    shadowSamplerDesc.magFilter = MTLSamplerMinMagFilterLinear;
    shadowSamplerDesc.sAddressMode = MTLSamplerAddressModeClampToEdge;
    shadowSamplerDesc.tAddressMode = MTLSamplerAddressModeClampToEdge;
    _shadowSampler = [_device newSamplerStateWithDescriptor:shadowSamplerDesc];

    MTLTextureDescriptor* whiteDesc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                                                                         width:1
                                                                                        height:1
                                                                                     mipmapped:NO];
    whiteDesc.usage = MTLTextureUsageShaderRead;
    _whiteTexture = [_device newTextureWithDescriptor:whiteDesc];
    std::uint8_t whitePixel[4] = {255, 255, 255, 255};
    [_whiteTexture replaceRegion:MTLRegionMake2D(0, 0, 1, 1) mipmapLevel:0 withBytes:whitePixel bytesPerRow:4];
    MTLSamplerDescriptor* defaultSamplerDesc = [[MTLSamplerDescriptor alloc] init];
    defaultSamplerDesc.minFilter = MTLSamplerMinMagFilterLinear;
    defaultSamplerDesc.magFilter = MTLSamplerMinMagFilterLinear;
    defaultSamplerDesc.sAddressMode = MTLSamplerAddressModeRepeat;
    defaultSamplerDesc.tAddressMode = MTLSamplerAddressModeRepeat;
    _defaultSampler = [_device newSamplerStateWithDescriptor:defaultSamplerDesc];
    _uniformBuffer = [_device newBufferWithLength:kUniformStride * kMaxUniformDraws
                                          options:MTLResourceStorageModeShared];
    _uniformCursor = 0;
    _gridBuffer = nil;
    _gridVertexCount = 0;
    _insetBackgroundBuffer = nil;
    _insetBackgroundVertexCount = 0;
    _hudLabel = nil;
    _hudFps = 0.0f;
    _keys.fill(false);
    _selectedDrone = 0;
    _onboardCamera = false;
    _insetCameraEnabled = true;
    _cameraRecordingEnabled = false;
    _followDrone = true;
    _lidarEnabled = false;
    _radarEnabled = false;
    _lidarAccumulator = 0.0f;
    _radarAccumulator = 0.0f;
    _cameraOutputAccumulator = 0.0f;
    _propAngle = 0.0f;
    _lidarFrameId = 0;
    _radarFrameId = 0;
    _cameraFrameId = 0;
    _cameraOutputTexture = nil;
    _cameraOutputMsaaTexture = nil;
    _cameraOutputDepthTexture = nil;
    _cameraOutputWidth = 0;
    _cameraOutputHeight = 0;
    _cameraPos = vector_float3{210.0f, 190.0f, 230.0f};
    _cameraYaw = -2.31f;
    _cameraPitch = -0.39f;
    view.colorPixelFormat = MTLPixelFormatBGRA8Unorm;
    view.depthStencilPixelFormat = MTLPixelFormatDepth32Float;
    view.clearColor = MTLClearColorMake(0.80, 0.90, 1.0, 1.0);

    NSString* source = @R"(
        #include <metal_stdlib>
        using namespace metal;

        struct VertexIn {
            float3 position [[attribute(0)]];
            float3 color [[attribute(1)]];
            float3 normal [[attribute(2)]];
            float2 texCoord [[attribute(3)]];
            float useTexture [[attribute(4)]];
            float useLighting [[attribute(5)]];
        };

        struct Uniforms {
            float4x4 viewProjection;
            float4x4 model;
            float4x4 lightViewProjection;
            float pointSize;
            float shadowStrength;
        };

        struct VertexOut {
            float4 position [[position]];
            float3 color;
            float3 normal;
            float2 texCoord;
            float useTexture;
            float useLighting;
            float4 lightPosition;
            float shadowStrength;
            float pointSize [[point_size]];
        };

        vertex VertexOut vertex_main(VertexIn in [[stage_in]],
                                     constant Uniforms& uniforms [[buffer(1)]]) {
            VertexOut out;
            out.position = uniforms.viewProjection * uniforms.model * float4(in.position, 1.0);
            out.color = in.color;
            out.normal = normalize((uniforms.model * float4(in.normal, 0.0)).xyz);
            out.texCoord = in.texCoord;
            out.useTexture = in.useTexture;
            out.useLighting = in.useLighting;
            out.lightPosition = uniforms.lightViewProjection * uniforms.model * float4(in.position, 1.0);
            out.shadowStrength = uniforms.shadowStrength;
            out.pointSize = uniforms.pointSize;
            return out;
        }

        vertex float4 shadow_vertex(VertexIn in [[stage_in]],
                                    constant Uniforms& uniforms [[buffer(1)]]) {
            return uniforms.viewProjection * uniforms.model * float4(in.position, 1.0);
        }

        fragment float4 fragment_main(VertexOut in [[stage_in]],
                                      texture2d<float> diffuseTexture [[texture(0)]],
                                      depth2d<float> shadowTexture [[texture(1)]],
                                      sampler diffuseSampler [[sampler(0)]],
                                      sampler shadowSampler [[sampler(1)]]) {
            float light = 1.0;
            float shadow = 1.0;
            if (in.useLighting > 0.5) {
                float3 sunDir = normalize(float3(-0.45, 0.82, -0.35));
                float ndotl = saturate(dot(normalize(in.normal), sunDir));
                light = 0.58 + 0.42 * ndotl;
                float3 lightNdc = in.lightPosition.xyz / max(in.lightPosition.w, 0.0001);
                float2 shadowUv = float2(lightNdc.x * 0.5 + 0.5, 0.5 - lightNdc.y * 0.5);
                if (all(shadowUv >= float2(0.0)) && all(shadowUv <= float2(1.0)) &&
                    lightNdc.z >= 0.0 && lightNdc.z <= 1.0) {
                    float2 texel = 1.0 / float2(shadowTexture.get_width(), shadowTexture.get_height());
                    float bias = max(0.0012 * (1.0 - ndotl), 0.00045);
                    float pcf = 0.0;
                    for (int y = -1; y <= 1; ++y) {
                        for (int x = -1; x <= 1; ++x) {
                            float depth = shadowTexture.sample(shadowSampler, shadowUv + float2(x, y) * texel);
                            pcf += (lightNdc.z - bias) <= depth ? 1.0 : 0.0;
                        }
                    }
                    shadow = mix(1.0 - in.shadowStrength, 1.0, pcf / 9.0);
                }
            }
            float3 litColor = in.color * light * shadow;
            if (in.useTexture > 0.5) {
                float4 tex = diffuseTexture.sample(diffuseSampler, in.texCoord);
                return float4(tex.rgb * litColor, tex.a);
            }
            return float4(litColor, 1.0);
        }
    )";

    NSError* error = nil;
    id<MTLLibrary> library = [_device newLibraryWithSource:source options:nil error:&error];
    if (!library) {
        std::cerr << "Metal library compile failed: " << [[error localizedDescription] UTF8String] << "\n";
        return nil;
    }

    MTLVertexDescriptor* vertexDescriptor = [[MTLVertexDescriptor alloc] init];
    vertexDescriptor.attributes[0].format = MTLVertexFormatFloat3;
    vertexDescriptor.attributes[0].offset = offsetof(MetalVertex, position);
    vertexDescriptor.attributes[0].bufferIndex = 0;
    vertexDescriptor.attributes[1].format = MTLVertexFormatFloat3;
    vertexDescriptor.attributes[1].offset = offsetof(MetalVertex, color);
    vertexDescriptor.attributes[1].bufferIndex = 0;
    vertexDescriptor.attributes[2].format = MTLVertexFormatFloat3;
    vertexDescriptor.attributes[2].offset = offsetof(MetalVertex, normal);
    vertexDescriptor.attributes[2].bufferIndex = 0;
    vertexDescriptor.attributes[3].format = MTLVertexFormatFloat2;
    vertexDescriptor.attributes[3].offset = offsetof(MetalVertex, texCoord);
    vertexDescriptor.attributes[3].bufferIndex = 0;
    vertexDescriptor.attributes[4].format = MTLVertexFormatFloat;
    vertexDescriptor.attributes[4].offset = offsetof(MetalVertex, useTexture);
    vertexDescriptor.attributes[4].bufferIndex = 0;
    vertexDescriptor.attributes[5].format = MTLVertexFormatFloat;
    vertexDescriptor.attributes[5].offset = offsetof(MetalVertex, useLighting);
    vertexDescriptor.attributes[5].bufferIndex = 0;
    vertexDescriptor.layouts[0].stride = sizeof(MetalVertex);
    vertexDescriptor.layouts[0].stepFunction = MTLVertexStepFunctionPerVertex;

    MTLRenderPipelineDescriptor* desc = [[MTLRenderPipelineDescriptor alloc] init];
    desc.vertexFunction = [library newFunctionWithName:@"vertex_main"];
    desc.fragmentFunction = [library newFunctionWithName:@"fragment_main"];
    desc.colorAttachments[0].pixelFormat = view.colorPixelFormat;
    desc.depthAttachmentPixelFormat = view.depthStencilPixelFormat;
    desc.vertexDescriptor = vertexDescriptor;

    _pipeline = [_device newRenderPipelineStateWithDescriptor:desc error:&error];
    if (!_pipeline) {
        std::cerr << "Pipeline creation failed: " << [[error localizedDescription] UTF8String] << "\n";
        return nil;
    }

    MTLRenderPipelineDescriptor* shadowPipelineDesc = [[MTLRenderPipelineDescriptor alloc] init];
    shadowPipelineDesc.vertexFunction = [library newFunctionWithName:@"shadow_vertex"];
    shadowPipelineDesc.fragmentFunction = nil;
    shadowPipelineDesc.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;
    shadowPipelineDesc.vertexDescriptor = vertexDescriptor;
    _shadowPipeline = [_device newRenderPipelineStateWithDescriptor:shadowPipelineDesc error:&error];
    if (!_shadowPipeline) {
        std::cerr << "Shadow pipeline creation failed: " << [[error localizedDescription] UTF8String] << "\n";
        return nil;
    }

    MTLDepthStencilDescriptor* depthDesc = [[MTLDepthStencilDescriptor alloc] init];
    depthDesc.depthCompareFunction = MTLCompareFunctionLess;
    depthDesc.depthWriteEnabled = YES;
    _depthState = [_device newDepthStencilStateWithDescriptor:depthDesc];

    MTLDepthStencilDescriptor* overlayDepthDesc = [[MTLDepthStencilDescriptor alloc] init];
    overlayDepthDesc.depthCompareFunction = MTLCompareFunctionAlways;
    overlayDepthDesc.depthWriteEnabled = NO;
    _overlayDepthState = [_device newDepthStencilStateWithDescriptor:overlayDepthDesc];

    std::vector<MetalVertex> insetBackground = {
        {vector_float3{-1.0f, -1.0f, 0.0f}, vector_float3{0.78f, 0.88f, 1.0f}},
        {vector_float3{ 1.0f, -1.0f, 0.0f}, vector_float3{0.78f, 0.88f, 1.0f}},
        {vector_float3{ 1.0f,  1.0f, 0.0f}, vector_float3{0.78f, 0.88f, 1.0f}},
        {vector_float3{-1.0f, -1.0f, 0.0f}, vector_float3{0.78f, 0.88f, 1.0f}},
        {vector_float3{ 1.0f,  1.0f, 0.0f}, vector_float3{0.78f, 0.88f, 1.0f}},
        {vector_float3{-1.0f,  1.0f, 0.0f}, vector_float3{0.78f, 0.88f, 1.0f}},
    };
    _insetBackgroundBuffer = makeMetalBuffer(_device, insetBackground);
    _insetBackgroundVertexCount = static_cast<NSUInteger>(insetBackground.size());

    _sim.load("config.yaml", "sensors.yaml");

    std::vector<MetalVertex> gridVertices;
    gridVertices.reserve(64);
    vector_float3 gridColor = {0.50f, 0.54f, 0.56f};
    for (int i = -140; i <= 140; i += 20) {
        float v = static_cast<float>(i);
        addLine(gridVertices, vector_float3{-140.0f, 0.0f, v}, vector_float3{140.0f, 0.0f, v}, gridColor);
        addLine(gridVertices, vector_float3{v, 0.0f, -140.0f}, vector_float3{v, 0.0f, 140.0f}, gridColor);
    }
    addLine(gridVertices, vector_float3{-150.0f, 0.02f, 0.0f}, vector_float3{150.0f, 0.02f, 0.0f}, vector_float3{0.25f, 0.30f, 0.32f});
    addLine(gridVertices, vector_float3{0.0f, 0.02f, -150.0f}, vector_float3{0.0f, 0.02f, 150.0f}, vector_float3{0.25f, 0.30f, 0.32f});
    _gridBuffer = makeMetalBuffer(_device, gridVertices);
    _gridVertexCount = static_cast<NSUInteger>(gridVertices.size());

    uam::SceneMesh cityMesh = uam::loadObjSceneMesh("map/hh_clip.obj", glm::vec3(0.62f, 0.64f, 0.62f), true);
    if (_cityMesh.upload(_device, cityMesh)) {
        std::cout << "Uploaded city mesh to Metal: " << _cityMesh.vertexCount() / 3 << " triangles\n";
    }

    uam::SceneMesh importedGroundMesh = uam::loadObjSceneMesh("map/surface_ground.obj", glm::vec3(0.20f, 0.45f, 0.25f), true);
    if (importedGroundMesh.hasBounds && cityMesh.hasBounds) {
        float importedArea = (importedGroundMesh.boundsMax.x - importedGroundMesh.boundsMin.x) *
                             (importedGroundMesh.boundsMax.z - importedGroundMesh.boundsMin.z);
        float cityArea = (cityMesh.boundsMax.x - cityMesh.boundsMin.x) *
                         (cityMesh.boundsMax.z - cityMesh.boundsMin.z);
        std::cout << "Imported ground footprint covers about "
                  << (cityArea > 1e-3f ? (100.0f * importedArea / cityArea) : 0.0f)
                  << "% of city bounds; using generated fill mesh for full footprint\n";
    }

    uam::SceneMesh terrainMesh = makeGroundFillMesh(cityMesh);
    if (_terrainMesh.upload(_device, terrainMesh)) {
        std::cout << "Uploaded terrain mesh to Metal: " << _terrainMesh.vertexCount() / 3 << " triangles\n";
    }

    _sensorGeometry.addMesh(cityMesh, 1000);
    _sensorGeometry.addMesh(terrainMesh, 2000);
    _sensorGeometry.build();

    constexpr float droneScale = 0.001f;
    uam::SceneMesh droneBody = uam::loadObjSceneMesh("model/body.obj", glm::vec3(0.18f, 0.18f, 0.18f), false, droneScale);
    uam::SceneMesh propFL = uam::loadObjSceneMesh("model/prop_FL.obj", glm::vec3(0.04f, 0.04f, 0.04f), false, droneScale);
    uam::SceneMesh propFR = uam::loadObjSceneMesh("model/prop_FR.obj", glm::vec3(0.04f, 0.04f, 0.04f), false, droneScale);
    uam::SceneMesh propRL = uam::loadObjSceneMesh("model/prop_RL.obj", glm::vec3(0.04f, 0.04f, 0.04f), false, droneScale);
    uam::SceneMesh propRR = uam::loadObjSceneMesh("model/prop_RR.obj", glm::vec3(0.04f, 0.04f, 0.04f), false, droneScale);
    _droneModel.body.upload(_device, droneBody);
    _droneModel.propFL.upload(_device, propFL);
    _droneModel.propFR.upload(_device, propFR);
    _droneModel.propRL.upload(_device, propRL);
    _droneModel.propRR.upload(_device, propRR);
    _droneModel.propSweep.upload(_device, makePropSweepMesh());
    buildSensorPart(_droneSensorModel.body, droneBody, 3000);
    buildSensorPart(_droneSensorModel.propFL, propFL, 3001);
    buildSensorPart(_droneSensorModel.propFR, propFR, 3002);
    buildSensorPart(_droneSensorModel.propRL, propRL, 3003);
    buildSensorPart(_droneSensorModel.propRR, propRR, 3004);
    _droneSensorModel.hasGeometry = !droneBody.vertices.empty();
    if (_droneModel.loaded()) {
        std::cout << "Uploaded drone OBJ model to Metal with animated propellers\n";
    }
    if (_droneSensorModel.loaded()) {
        std::cout << "Built drone sensor BVHs for mesh LiDAR targets\n";
    }

    __weak Renderer* weakSelf = self;
    _keyDownMonitor = [NSEvent addLocalMonitorForEventsMatchingMask:NSEventMaskKeyDown handler:^NSEvent*(NSEvent* event) {
        [weakSelf handleKeyDown:event];
        return nil;
    }];
    _keyUpMonitor = [NSEvent addLocalMonitorForEventsMatchingMask:NSEventMaskKeyUp handler:^NSEvent*(NSEvent* event) {
        [weakSelf handleKeyUp:event];
        return nil;
    }];

    _hudLabel = [[NSTextField alloc] initWithFrame:NSMakeRect(14.0, view.bounds.size.height - 170.0, 390.0, 150.0)];
    [_hudLabel setEditable:NO];
    [_hudLabel setSelectable:NO];
    [_hudLabel setBezeled:NO];
    [_hudLabel setBordered:NO];
    [_hudLabel setDrawsBackground:YES];
    [_hudLabel setBackgroundColor:[NSColor colorWithCalibratedWhite:0.02 alpha:0.72]];
    [_hudLabel setTextColor:[NSColor colorWithCalibratedWhite:0.95 alpha:1.0]];
    [_hudLabel setFont:[NSFont monospacedSystemFontOfSize:12.0 weight:NSFontWeightRegular]];
    [_hudLabel setAutoresizingMask:NSViewMaxXMargin | NSViewMinYMargin];
    [_hudLabel setWantsLayer:YES];
    _hudLabel.layer.cornerRadius = 5.0;
    _hudLabel.layer.masksToBounds = YES;
    [[_hudLabel cell] setWraps:YES];
    [[_hudLabel cell] setScrollable:NO];
    [view addSubview:_hudLabel];

    _lastFrame = std::chrono::steady_clock::now();
    std::cout << "Controls: Tab switch drone, M manual drone, V onboard camera, P PiP camera, O camera record, F follow/free, L LiDAR, R RADAR, Esc quit, WASD move, Space/C up/down, Q/E yaw\n";
    return self;
}

- (void)dealloc {
    if (_keyDownMonitor) [NSEvent removeMonitor:_keyDownMonitor];
    if (_keyUpMonitor) [NSEvent removeMonitor:_keyUpMonitor];
}

- (void)currentViewEye:(vector_float3*)eye target:(vector_float3*)target {
    if (_onboardCamera && !_sim.drones().empty()) {
        _selectedDrone = std::min(_selectedDrone, _sim.drones().size() - 1);
        const auto& drone = _sim.drones()[_selectedDrone];
        vector_float3 fwd = droneForward(drone.yawDeg);
        *eye = toFloat3(drone.position) + fwd * 1.1f + vector_float3{0.0f, 0.28f, 0.0f};
        *target = *eye + fwd * 80.0f + vector_float3{0.0f, -8.0f, 0.0f};
        return;
    }

    if (_followDrone && !_sim.drones().empty()) {
        _selectedDrone = std::min(_selectedDrone, _sim.drones().size() - 1);
        *target = toFloat3(_sim.drones()[_selectedDrone].position);
        vector_float3 fwd = cameraForward(_cameraYaw, _cameraPitch);
        *eye = *target - fwd * 22.0f + vector_float3{0.0f, 22.0f, 0.0f};
        return;
    }

    vector_float3 fwd = cameraForward(_cameraYaw, _cameraPitch);
    *eye = _cameraPos;
    *target = _cameraPos + fwd;
}

- (void)captureCurrentViewAsFreeCamera {
    vector_float3 eye;
    vector_float3 target;
    [self currentViewEye:&eye target:&target];
    _cameraPos = eye;
    yawPitchFromDirection(target - eye, _cameraYaw, _cameraPitch);
    _cameraPitch = std::clamp(_cameraPitch, -1.35f, 1.15f);
}

- (matrix_float4x4)selectedDroneCameraViewMatrix {
    if (_sim.drones().empty()) {
        vector_float3 fwd = cameraForward(_cameraYaw, _cameraPitch);
        return makeLookAt(_cameraPos, _cameraPos + fwd, vector_float3{0.0f, 1.0f, 0.0f});
    }

    _selectedDrone = std::min(_selectedDrone, _sim.drones().size() - 1);
    const auto& drone = _sim.drones()[_selectedDrone];
    vector_float3 fwd = droneForward(drone.yawDeg);
    vector_float3 eye = toFloat3(drone.position) + fwd * 1.1f + vector_float3{0.0f, 0.28f, 0.0f};
    vector_float3 target = eye + fwd * 80.0f + vector_float3{0.0f, -8.0f, 0.0f};
    return makeLookAt(eye, target, vector_float3{0.0f, 1.0f, 0.0f});
}

- (matrix_float4x4)sunViewProjectionMatrix {
    vector_float3 center = vector_float3{0.0f, 80.0f, 0.0f};
    vector_float3 sunDir = simd_normalize(kSunDirection);
    vector_float3 eye = center + sunDir * 520.0f;
    matrix_float4x4 lightView = makeLookAt(eye, center, vector_float3{0.0f, 1.0f, 0.0f});
    matrix_float4x4 lightProjection = makeOrthographic(-340.0f, 340.0f, -340.0f, 340.0f, 1.0f, 1100.0f);
    return matrix_multiply(lightProjection, lightView);
}

- (void)drawOverlayBuffer:(id<MTLBuffer>)buffer
              vertexCount:(NSUInteger)vertexCount
              primitiveType:(MTLPrimitiveType)primitiveType
                 encoder:(id<MTLRenderCommandEncoder>)enc {
    if (!buffer || vertexCount == 0) return;
    setUniforms(_uniformBuffer, _uniformCursor, enc, identityMatrix(), identityMatrix());
    [enc setVertexBuffer:buffer offset:0 atIndex:0];
    [enc drawPrimitives:primitiveType vertexStart:0 vertexCount:vertexCount];
}

- (void)drawSceneWithEncoder:(id<MTLRenderCommandEncoder>)enc
              viewProjection:(matrix_float4x4)viewProjection
               triangleBuffer:(id<MTLBuffer>)triangleBuffer
                triangleCount:(NSUInteger)triangleCount
                   lidarBuffer:(id<MTLBuffer>)lidarBuffer
                    lidarCount:(NSUInteger)lidarCount
                   radarBuffer:(id<MTLBuffer>)radarBuffer
                    radarCount:(NSUInteger)radarCount
            lightViewProjection:(matrix_float4x4)lightViewProjection
                  shadowStrength:(float)shadowStrength
             skipSelectedDrone:(BOOL)skipSelectedDrone {
    setUniforms(_uniformBuffer, _uniformCursor, enc, viewProjection, identityMatrix(), 5.0f, lightViewProjection, shadowStrength);

    if (_gridBuffer) {
        [enc setVertexBuffer:_gridBuffer offset:0 atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypeLine vertexStart:0 vertexCount:_gridVertexCount];
    }
    _terrainMesh.draw(enc);
    _cityMesh.draw(enc);
    if (triangleBuffer && triangleCount > 0) {
        [enc setVertexBuffer:triangleBuffer offset:0 atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:triangleCount];
    }
    if (_droneModel.loaded()) {
        for (std::size_t i = 0; i < _sim.drones().size(); ++i) {
            if (skipSelectedDrone && i == _selectedDrone) continue;
            drawDroneModel(_uniformBuffer, _uniformCursor, enc, _droneModel, _sim.drones()[i], _propAngle, viewProjection, lightViewProjection, shadowStrength);
        }
        setUniforms(_uniformBuffer, _uniformCursor, enc, viewProjection, identityMatrix(), 5.0f, lightViewProjection, shadowStrength);
    }
    if (lidarBuffer && lidarCount > 0) {
        setUniforms(_uniformBuffer, _uniformCursor, enc, viewProjection, identityMatrix(), 5.0f);
        [enc setVertexBuffer:lidarBuffer offset:0 atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypePoint vertexStart:0 vertexCount:lidarCount];
    }
    if (radarBuffer && radarCount > 0) {
        setUniforms(_uniformBuffer, _uniformCursor, enc, viewProjection, identityMatrix(), 8.0f);
        [enc setVertexBuffer:radarBuffer offset:0 atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypePoint vertexStart:0 vertexCount:radarCount];
    }
}

- (void)drawShadowSceneWithEncoder:(id<MTLRenderCommandEncoder>)enc
                     viewProjection:(matrix_float4x4)viewProjection
                      triangleBuffer:(id<MTLBuffer>)triangleBuffer
                       triangleCount:(NSUInteger)triangleCount
                 skipSelectedDrone:(BOOL)skipSelectedDrone {
    setUniforms(_uniformBuffer, _uniformCursor, enc, viewProjection, identityMatrix());
    _terrainMesh.draw(enc);
    _cityMesh.draw(enc);
    if (triangleBuffer && triangleCount > 0) {
        [enc setVertexBuffer:triangleBuffer offset:0 atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:triangleCount];
    }
    if (_droneModel.loaded()) {
        for (std::size_t i = 0; i < _sim.drones().size(); ++i) {
            if (skipSelectedDrone && i == _selectedDrone) continue;
            drawDroneModel(_uniformBuffer, _uniformCursor, enc, _droneModel, _sim.drones()[i], _propAngle, viewProjection);
        }
    }
}

- (void)renderShadowMapWithCommandBuffer:(id<MTLCommandBuffer>)command
                         viewProjection:(matrix_float4x4)viewProjection
                          triangleBuffer:(id<MTLBuffer>)triangleBuffer
                           triangleCount:(NSUInteger)triangleCount
                       skipSelectedDrone:(BOOL)skipSelectedDrone {
    if (!_shadowDepthTexture || !_shadowPipeline) return;

    MTLRenderPassDescriptor* shadowPass = [MTLRenderPassDescriptor renderPassDescriptor];
    shadowPass.depthAttachment.texture = _shadowDepthTexture;
    shadowPass.depthAttachment.loadAction = MTLLoadActionClear;
    shadowPass.depthAttachment.storeAction = MTLStoreActionStore;
    shadowPass.depthAttachment.clearDepth = 1.0;

    id<MTLRenderCommandEncoder> shadowEnc = [command renderCommandEncoderWithDescriptor:shadowPass];
    [shadowEnc setRenderPipelineState:_shadowPipeline];
    [shadowEnc setDepthStencilState:_depthState];
    MTLViewport viewport{0.0, 0.0, static_cast<double>(kShadowMapSize), static_cast<double>(kShadowMapSize), 0.0, 1.0};
    MTLScissorRect scissor{0, 0, kShadowMapSize, kShadowMapSize};
    [shadowEnc setViewport:viewport];
    [shadowEnc setScissorRect:scissor];
    [self drawShadowSceneWithEncoder:shadowEnc
                       viewProjection:viewProjection
                        triangleBuffer:triangleBuffer
                         triangleCount:triangleCount
                    skipSelectedDrone:skipSelectedDrone];
    [shadowEnc endEncoding];
}

- (void)updateHUDWithView:(MTKView*)view dt:(float)dt {
    if (!_hudLabel) return;

    if (dt > 1e-4f) {
        float instantFps = 1.0f / dt;
        _hudFps = _hudFps <= 1.0f ? instantFps : _hudFps * 0.92f + instantFps * 0.08f;
    }

    NSRect bounds = view.bounds;
    CGFloat width = 420.0;
    CGFloat height = 154.0;
    [_hudLabel setFrame:NSMakeRect(14.0, bounds.size.height - height - 14.0, width, height)];

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1);
    ss << "FPS " << _hudFps << "  Drones " << _sim.drones().size() << "\n";

    if (_sim.drones().empty()) {
        ss << "Selected: none\n";
    } else {
        _selectedDrone = std::min(_selectedDrone, _sim.drones().size() - 1);
        const auto& drone = _sim.drones()[_selectedDrone];
        float speed = glm::length(drone.velocity);
        ss << "Selected " << (_selectedDrone + 1) << "/" << _sim.drones().size() << ": " << drone.name << "\n";
        ss << "Mode " << (drone.manual ? "manual" : "route")
           << "  Camera " << (_onboardCamera ? "onboard" : (_followDrone ? "follow" : "free"))
           << "  PiP " << (_insetCameraEnabled ? "on" : "off")
           << "  Rec " << (_cameraRecordingEnabled ? "on" : "off") << "\n";
        ss << "LiDAR " << (_lidarEnabled ? "on" : "off")
           << "  pts " << _lidarPointVertices.size()
           << "  RADAR " << (_radarEnabled ? "on" : "off")
           << "  pts " << _radarPointVertices.size() << "\n";
        ss << std::setprecision(2);
        ss << "Pos  " << drone.position.x << ", " << drone.position.y << ", " << drone.position.z << "\n";
        ss << "Vel  " << drone.velocity.x << ", " << drone.velocity.y << ", " << drone.velocity.z
           << "  |v| " << speed << "\n";
        ss << std::setprecision(1);
        ss << "Yaw  " << drone.yawDeg << " deg";
    }

    [_hudLabel setStringValue:[NSString stringWithUTF8String:ss.str().c_str()]];
}

- (void)ensureCameraOutputTexturesWithWidth:(unsigned)width height:(unsigned)height {
    width = std::max(1u, width);
    height = std::max(1u, height);
    if (_cameraOutputTexture && _cameraOutputDepthTexture &&
        _cameraOutputWidth == width && _cameraOutputHeight == height) return;

    _cameraOutputWidth = width;
    _cameraOutputHeight = height;

    MTLTextureDescriptor* colorDesc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
                                                                                         width:width
                                                                                        height:height
                                                                                     mipmapped:NO];
    colorDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
    colorDesc.storageMode = MTLStorageModeShared;
    _cameraOutputTexture = [_device newTextureWithDescriptor:colorDesc];
    _cameraOutputMsaaTexture = nil;

    MTLTextureDescriptor* depthDesc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
                                                                                         width:width
                                                                                        height:height
                                                                                     mipmapped:NO];
    depthDesc.usage = MTLTextureUsageRenderTarget;
    depthDesc.storageMode = MTLStorageModePrivate;
    _cameraOutputDepthTexture = [_device newTextureWithDescriptor:depthDesc];
}

- (void)saveCameraOutputTextureFrame:(int)frameId {
    if (!_cameraOutputTexture || _cameraOutputWidth == 0 || _cameraOutputHeight == 0) return;
    const auto& cfg = _sim.sensors();
    if (!ensureOutputDir(cfg.cameraOutputDir, "Camera")) return;

    NSUInteger bytesPerRow = static_cast<NSUInteger>(_cameraOutputWidth) * 4;
    std::vector<std::uint8_t> pixels(static_cast<std::size_t>(bytesPerRow) * _cameraOutputHeight);
    MTLRegion region = MTLRegionMake2D(0, 0, _cameraOutputWidth, _cameraOutputHeight);
    [_cameraOutputTexture getBytes:pixels.data() bytesPerRow:bytesPerRow fromRegion:region mipmapLevel:0];

    auto linearToSrgbByte = [](float c) -> std::uint8_t {
        c = std::clamp(c, 0.0f, 1.0f);
        float srgb = c <= 0.0031308f ? 12.92f * c : 1.055f * std::pow(c, 1.0f / 2.4f) - 0.055f;
        return static_cast<std::uint8_t>(std::round(std::clamp(srgb, 0.0f, 1.0f) * 255.0f));
    };

    float exposure = std::max(0.0f, cfg.cameraExposure);
    float contrast = std::max(0.0f, cfg.cameraContrast);
    float saturation = std::max(0.0f, cfg.cameraSaturation);

    NSUInteger outputBytesPerRow = static_cast<NSUInteger>(_cameraOutputWidth) * 3;
    std::vector<std::uint8_t> outputPixels(static_cast<std::size_t>(outputBytesPerRow) * _cameraOutputHeight);
    for (unsigned y = 0; y < _cameraOutputHeight; ++y) {
        for (unsigned x = 0; x < _cameraOutputWidth; ++x) {
            std::size_t src = static_cast<std::size_t>(y) * bytesPerRow + static_cast<std::size_t>(x) * 4;
            std::size_t dst = static_cast<std::size_t>(y) * outputBytesPerRow + static_cast<std::size_t>(x) * 3;
            float r = (static_cast<float>(pixels[src + 2]) / 255.0f) * exposure;
            float g = (static_cast<float>(pixels[src + 1]) / 255.0f) * exposure;
            float b = (static_cast<float>(pixels[src + 0]) / 255.0f) * exposure;
            float luma = r * 0.2126f + g * 0.7152f + b * 0.0722f;
            r = luma + (r - luma) * saturation;
            g = luma + (g - luma) * saturation;
            b = luma + (b - luma) * saturation;
            r = (r - 0.5f) * contrast + 0.5f;
            g = (g - 0.5f) * contrast + 0.5f;
            b = (b - 0.5f) * contrast + 0.5f;
            outputPixels[dst + 0] = linearToSrgbByte(r);
            outputPixels[dst + 1] = linearToSrgbByte(g);
            outputPixels[dst + 2] = linearToSrgbByte(b);
        }
    }

    CGColorSpaceRef colorSpace = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    if (!colorSpace) colorSpace = CGColorSpaceCreateDeviceRGB();
    CGDataProviderRef provider = CGDataProviderCreateWithData(nullptr,
                                                             outputPixels.data(),
                                                             outputPixels.size(),
                                                             nullptr);
    CGImageRef image = CGImageCreate(_cameraOutputWidth,
                                     _cameraOutputHeight,
                                     8,
                                     24,
                                     outputBytesPerRow,
                                     colorSpace,
                                     kCGImageAlphaNone,
                                     provider,
                                     nullptr,
                                     false,
                                     kCGRenderingIntentDefault);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(colorSpace);
    if (!image) {
        std::cerr << "Camera export: could not create CG image\n";
        return;
    }

    NSBitmapImageRep* rep = [[NSBitmapImageRep alloc] initWithCGImage:image];
    CGImageRelease(image);
    if (!rep) {
        std::cerr << "Camera export: could not create bitmap image rep\n";
        return;
    }

    NSData* png = [rep representationUsingType:NSBitmapImageFileTypePNG properties:@{}];
    if (!png) {
        std::cerr << "Camera export: could not encode PNG\n";
        return;
    }

    std::ostringstream filename;
    filename << cfg.cameraOutputDir << "/camera_frame_" << std::setw(6) << std::setfill('0') << frameId << ".png";
    NSString* path = [NSString stringWithUTF8String:filename.str().c_str()];
    if (![png writeToFile:path atomically:YES]) {
        std::cerr << "Camera export: could not save file " << filename.str() << "\n";
    }
}

- (void)renderCameraOutputIfNeededWithTriangleBuffer:(id<MTLBuffer>)triangleBuffer
                                      triangleCount:(NSUInteger)triangleCount
                                        lidarBuffer:(id<MTLBuffer>)lidarBuffer
                                         lidarCount:(NSUInteger)lidarCount
                                        radarBuffer:(id<MTLBuffer>)radarBuffer
                                         radarCount:(NSUInteger)radarCount
                                                 dt:(float)dt {
    if (!_cameraRecordingEnabled || _sim.drones().empty()) {
        _cameraOutputAccumulator = 0.0f;
        return;
    }

    const auto& cfg = _sim.sensors();
    float period = 1.0f / std::max(cfg.cameraFps, 0.1f);
    _cameraOutputAccumulator += dt;
    if (_cameraFrameId > 0 && _cameraOutputAccumulator < period) return;
    _cameraOutputAccumulator = std::fmod(_cameraOutputAccumulator, period);

    [self ensureCameraOutputTexturesWithWidth:cfg.cameraWidth height:cfg.cameraHeight];
    if (!_cameraOutputTexture || !_cameraOutputDepthTexture) return;

    MTLRenderPassDescriptor* cameraPass = [MTLRenderPassDescriptor renderPassDescriptor];
    cameraPass.colorAttachments[0].texture = _cameraOutputTexture;
    cameraPass.colorAttachments[0].loadAction = MTLLoadActionClear;
    cameraPass.colorAttachments[0].storeAction = MTLStoreActionStore;
    cameraPass.colorAttachments[0].clearColor = MTLClearColorMake(0.80, 0.90, 1.0, 1.0);
    cameraPass.depthAttachment.texture = _cameraOutputDepthTexture;
    cameraPass.depthAttachment.loadAction = MTLLoadActionClear;
    cameraPass.depthAttachment.storeAction = MTLStoreActionDontCare;
    cameraPass.depthAttachment.clearDepth = 1.0;

    id<MTLCommandBuffer> command = [_queue commandBuffer];
    id<MTLRenderCommandEncoder> enc = [command renderCommandEncoderWithDescriptor:cameraPass];
    [enc setRenderPipelineState:_pipeline];
    [enc setFragmentTexture:_whiteTexture atIndex:0];
    [enc setFragmentTexture:_shadowDepthTexture atIndex:1];
    [enc setFragmentSamplerState:_defaultSampler atIndex:0];
    [enc setFragmentSamplerState:_shadowSampler atIndex:1];
    [enc setDepthStencilState:_depthState];
    MTLViewport viewport{0.0, 0.0, static_cast<double>(cfg.cameraWidth), static_cast<double>(cfg.cameraHeight), 0.0, 1.0};
    MTLScissorRect scissor{0, 0, static_cast<NSUInteger>(cfg.cameraWidth), static_cast<NSUInteger>(cfg.cameraHeight)};
    [enc setViewport:viewport];
    [enc setScissorRect:scissor];

    _uniformCursor = 0;
    matrix_float4x4 projection = makePerspective(glm::radians(cfg.cameraFov),
                                                 static_cast<float>(cfg.cameraWidth) / static_cast<float>(std::max(1u, cfg.cameraHeight)),
                                                 0.05f,
                                                 1000.0f);
    matrix_float4x4 viewProjection = matrix_multiply(projection, [self selectedDroneCameraViewMatrix]);
    [self drawSceneWithEncoder:enc
                viewProjection:viewProjection
                 triangleBuffer:triangleBuffer
                  triangleCount:triangleCount
                    lidarBuffer:nil
                     lidarCount:0
                    radarBuffer:nil
                     radarCount:0
            lightViewProjection:identityMatrix()
                  shadowStrength:0.0f
              skipSelectedDrone:YES];

    [enc setDepthStencilState:_overlayDepthState];
    if (lidarBuffer && lidarCount > 0) {
        setUniforms(_uniformBuffer, _uniformCursor, enc, viewProjection, identityMatrix(), 2.5f);
        [enc setVertexBuffer:lidarBuffer offset:0 atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypePoint vertexStart:0 vertexCount:lidarCount];
    }
    if (radarBuffer && radarCount > 0) {
        setUniforms(_uniformBuffer, _uniformCursor, enc, viewProjection, identityMatrix(), 4.0f);
        [enc setVertexBuffer:radarBuffer offset:0 atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypePoint vertexStart:0 vertexCount:radarCount];
    }
    [enc endEncoding];
    [command commit];
    [command waitUntilCompleted];

    [self saveCameraOutputTextureFrame:_cameraFrameId++];
}

- (void)handleKeyDown:(NSEvent*)event {
    unsigned short code = event.keyCode;
    if (code == 53) { // Esc
        [NSApp terminate:nil];
        return;
    }
    if (code < _keys.size()) _keys[code] = true;
    if (code == 37 && !event.isARepeat) { // L
        _lidarEnabled = !_lidarEnabled;
        _lidarAccumulator = 0.0f;
        _lidarPointVertices.clear();
        _lastLidarHits.clear();
        std::cout << "LiDAR: " << (_lidarEnabled ? "ON" : "OFF") << "\n";
    }
    if (code == 15 && !event.isARepeat) { // R
        _radarEnabled = !_radarEnabled;
        _radarAccumulator = 0.0f;
        _radarPointVertices.clear();
        _lastRadarDetections.clear();
        std::cout << "RADAR: " << (_radarEnabled ? "ON" : "OFF") << "\n";
    }
    if (code == 3 && !event.isARepeat) { // F
        bool goingFree = _followDrone || _onboardCamera;
        if (goingFree) [self captureCurrentViewAsFreeCamera];
        _followDrone = !goingFree;
        if (goingFree) _onboardCamera = false;
        std::cout << "Camera: " << (_followDrone ? "follow" : "free") << "\n";
    }
    if (code == 9 && !event.isARepeat) { // V
        _onboardCamera = !_onboardCamera;
        if (_onboardCamera) _followDrone = true;
        std::cout << "Onboard camera: " << (_onboardCamera ? "ON" : "OFF") << "\n";
    }
    if (code == 35 && !event.isARepeat) { // P
        _insetCameraEnabled = !_insetCameraEnabled;
        std::cout << "PiP camera: " << (_insetCameraEnabled ? "ON" : "OFF") << "\n";
    }
    if (code == 31 && !event.isARepeat) { // O
        _cameraRecordingEnabled = !_cameraRecordingEnabled;
        _cameraOutputAccumulator = 0.0f;
        std::cout << "Camera recording: " << (_cameraRecordingEnabled ? "ON" : "OFF") << "\n";
    }
    if (code == 46 && !event.isARepeat && !_sim.drones().empty()) { // M
        _selectedDrone = std::min(_selectedDrone, _sim.drones().size() - 1);
        bool manual = !_sim.droneManual(_selectedDrone);
        _sim.setDroneManual(_selectedDrone, manual);
        std::cout << "Manual " << _sim.drones()[_selectedDrone].name << ": " << (manual ? "ON" : "OFF") << "\n";
    }
    if (code == 48 && !event.isARepeat && !_sim.drones().empty()) { // Tab
        _selectedDrone = (_selectedDrone + 1) % _sim.drones().size();
        std::cout << "Selected drone: " << _sim.drones()[_selectedDrone].name
                  << " (" << (_selectedDrone + 1) << "/" << _sim.drones().size() << ")"
                  << (_sim.droneManual(_selectedDrone) ? " manual" : " route") << "\n";
    }
}

- (void)handleKeyUp:(NSEvent*)event {
    unsigned short code = event.keyCode;
    if (code < _keys.size()) _keys[code] = false;
}

- (void)updateManualDrone:(float)dt {
    if (_sim.drones().empty()) return;
    _selectedDrone = std::min(_selectedDrone, _sim.drones().size() - 1);
    if (!_sim.droneManual(_selectedDrone)) return;
    if (!_followDrone && !_onboardCamera) return;

    float forward = 0.0f;
    float right = 0.0f;
    float up = 0.0f;
    float yawRate = 0.0f;
    if (_keys[13]) forward += 1.0f; // W
    if (_keys[1]) forward -= 1.0f;  // S
    if (_keys[2]) right += 1.0f;    // D
    if (_keys[0]) right -= 1.0f;    // A
    if (_keys[49]) up += 1.0f;      // Space
    if (_keys[8]) up -= 1.0f;       // C
    if (_keys[14]) yawRate += 90.0f; // E
    if (_keys[12]) yawRate -= 90.0f; // Q

    _sim.manualControlDrone(_selectedDrone, forward, right, up, yawRate, dt);
}

- (void)updateCamera:(float)dt {
    const float turnSpeed = 1.8f;
    bool manualSelected = !_sim.drones().empty() && _sim.droneManual(std::min(_selectedDrone, _sim.drones().size() - 1));
    bool drivingDrone = manualSelected && (_followDrone || _onboardCamera);

    if (!drivingDrone) {
        if (_keys[123] || _keys[12]) _cameraYaw -= turnSpeed * dt; // Left or Q
        if (_keys[124] || _keys[14]) _cameraYaw += turnSpeed * dt; // Right or E
    } else {
        if (_keys[123]) _cameraYaw -= turnSpeed * dt;              // Left
        if (_keys[124]) _cameraYaw += turnSpeed * dt;              // Right
    }
    if (_keys[126]) _cameraPitch += turnSpeed * dt;                // Up
    if (_keys[125]) _cameraPitch -= turnSpeed * dt;                // Down
    _cameraPitch = std::clamp(_cameraPitch, -1.35f, 1.15f);

    if (_followDrone || drivingDrone) return;

    vector_float3 move = {0.0f, 0.0f, 0.0f};
    vector_float3 fwd = cameraForward(_cameraYaw, _cameraPitch);
    vector_float3 right = cameraRight(_cameraYaw);
    if (_keys[13]) move += fwd;                                // W
    if (_keys[1]) move -= fwd;                                 // S
    if (_keys[0]) move -= right;                               // A
    if (_keys[2]) move += right;                               // D
    if (_keys[49]) move += vector_float3{0.0f, 1.0f, 0.0f};    // Space
    if (_keys[8]) move -= vector_float3{0.0f, 1.0f, 0.0f};     // C

    if (simd_length(move) > 1e-5f) {
        _cameraPos += simd_normalize(move) * (90.0f * dt);
    }
}

- (matrix_float4x4)viewMatrix {
    vector_float3 eye;
    vector_float3 target;
    [self currentViewEye:&eye target:&target];
    return makeLookAt(eye, target, vector_float3{0.0f, 1.0f, 0.0f});
}

- (void)mtkView:(MTKView*)view drawableSizeWillChange:(CGSize)size {
    (void)view;
    (void)size;
}

- (void)drawInMTKView:(MTKView*)view {
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - _lastFrame).count();
    _lastFrame = now;
    if (dt > 0.1f) dt = 0.1f;
    [self updateManualDrone:dt];
    _sim.update(dt);
    _propAngle = std::fmod(_propAngle + glm::radians(1800.0f) * dt, 2.0f * static_cast<float>(M_PI));
    [self updateCamera:dt];

    if (_lidarEnabled && !_sim.drones().empty() && (!_sensorGeometry.empty() || _droneSensorModel.loaded())) {
        const auto& cfg = _sim.sensors();
        float period = 1.0f / std::max(cfg.lidarFps, 0.1f);
        _lidarAccumulator += dt;
        if (_lidarPointVertices.empty() || _lidarAccumulator >= period) {
            _lidarAccumulator = std::fmod(_lidarAccumulator, period);
            _selectedDrone = std::min(_selectedDrone, _sim.drones().size() - 1);
            _lastLidarHits = simulateLidarScene(_sensorGeometry,
                                                _droneSensorModel,
                                                _sim.drones(),
                                                _selectedDrone,
                                                _propAngle,
                                                cfg);
            writeLidarFrameYaml(cfg.lidarOutputDir, _lidarFrameId++, _lastLidarHits, cfg);
            _lidarPointVertices.clear();
            _lidarPointVertices.reserve(_lastLidarHits.size());
            for (const auto& hit : _lastLidarHits) {
                if (!hit.hit) continue;
                bool dynamicDroneHit = hit.objectId >= 10000;
                glm::vec3 p = dynamicDroneHit ? hit.point : hit.point - hit.dir * 0.15f;
                vector_float3 color = dynamicDroneHit ? vector_float3{0.05f, 0.85f, 1.0f} : vector_float3{1.0f, 0.05f, 0.02f};
                _lidarPointVertices.push_back({vector_float3{p.x, p.y, p.z}, color});
            }
        }
    } else if (!_lidarPointVertices.empty() || !_lastLidarHits.empty()) {
        _lidarPointVertices.clear();
        _lastLidarHits.clear();
    }

    if (_radarEnabled && !_sim.drones().empty() && (!_sensorGeometry.empty() || _droneSensorModel.loaded())) {
        const auto& cfg = _sim.sensors();
        float period = 1.0f / std::max(cfg.radarFps, 0.1f);
        _radarAccumulator += dt;
        if (_radarPointVertices.empty() || _radarAccumulator >= period) {
            _radarAccumulator = std::fmod(_radarAccumulator, period);
            _selectedDrone = std::min(_selectedDrone, _sim.drones().size() - 1);
            _lastRadarDetections = simulateRadarScene(_sensorGeometry,
                                                      _droneSensorModel,
                                                      _sim.drones(),
                                                      _selectedDrone,
                                                      _propAngle,
                                                      cfg);
            writeRadarFrameYaml(cfg.radarOutputDir, _radarFrameId++, _lastRadarDetections, cfg);
            _radarPointVertices.clear();
            _radarPointVertices.reserve(_lastRadarDetections.size());
            for (const auto& det : _lastRadarDetections) {
                if (!det.hit) continue;
                bool dynamicDroneHit = det.objectId >= 10000;
                glm::vec3 p = det.point;
                vector_float3 color = dynamicDroneHit ? vector_float3{1.0f, 0.88f, 0.05f} : vector_float3{1.0f, 0.45f, 0.02f};
                _radarPointVertices.push_back({vector_float3{p.x, p.y, p.z}, color});
            }
        }
    } else if (!_radarPointVertices.empty() || !_lastRadarDetections.empty()) {
        _radarPointVertices.clear();
        _lastRadarDetections.clear();
    }

    [self updateHUDWithView:view dt:dt];

    MTLRenderPassDescriptor* pass = view.currentRenderPassDescriptor;
    id<CAMetalDrawable> drawable = view.currentDrawable;
    if (!pass || !drawable) return;

    std::vector<MetalVertex> triangleVertices;
    triangleVertices.reserve(_droneModel.loaded() ? 0 : _sim.drones().size() * 36);

    if (!_droneModel.loaded()) {
        for (const auto& drone : _sim.drones()) {
            vector_float3 pos = {drone.position.x, drone.position.y, drone.position.z};
            vector_float3 color = {drone.color.r, drone.color.g, drone.color.b};
            addCube(triangleVertices, pos, vector_float3{3.0f, 1.2f, 3.0f}, glm::radians(-drone.yawDeg), color);
        }
    }

    float width = std::max(1.0, view.drawableSize.width);
    float height = std::max(1.0, view.drawableSize.height);
    float aspect = static_cast<float>(width / height);
    matrix_float4x4 projection = makePerspective(60.0f * static_cast<float>(M_PI) / 180.0f, aspect, 0.1f, 1000.0f);
    matrix_float4x4 viewMatrix = [self viewMatrix];
    matrix_float4x4 viewProjection = matrix_multiply(projection, viewMatrix);

    id<MTLBuffer> triangleBuffer = makeMetalBuffer(_device, triangleVertices);
    id<MTLBuffer> lidarBuffer = makeMetalBuffer(_device, _lidarPointVertices);
    id<MTLBuffer> radarBuffer = makeMetalBuffer(_device, _radarPointVertices);
    NSUInteger triangleCount = static_cast<NSUInteger>(triangleVertices.size());
    NSUInteger lidarCount = static_cast<NSUInteger>(_lidarPointVertices.size());
    NSUInteger radarCount = static_cast<NSUInteger>(_radarPointVertices.size());

    [self renderCameraOutputIfNeededWithTriangleBuffer:triangleBuffer
                                        triangleCount:triangleCount
                                          lidarBuffer:lidarBuffer
                                           lidarCount:lidarCount
                                          radarBuffer:radarBuffer
                                           radarCount:radarCount
                                                   dt:dt];

    matrix_float4x4 lightViewProjection = [self sunViewProjectionMatrix];

    id<MTLCommandBuffer> command = [_queue commandBuffer];
    _uniformCursor = 0;
    [self renderShadowMapWithCommandBuffer:command
                            viewProjection:lightViewProjection
                             triangleBuffer:triangleBuffer
                              triangleCount:triangleCount
                          skipSelectedDrone:_onboardCamera];

    id<MTLRenderCommandEncoder> enc = [command renderCommandEncoderWithDescriptor:pass];
    [enc setRenderPipelineState:_pipeline];
    [enc setFragmentTexture:_whiteTexture atIndex:0];
    [enc setFragmentTexture:_shadowDepthTexture atIndex:1];
    [enc setFragmentSamplerState:_defaultSampler atIndex:0];
    [enc setFragmentSamplerState:_shadowSampler atIndex:1];
    [enc setDepthStencilState:_depthState];

    MTLViewport fullViewport{0.0, 0.0, static_cast<double>(width), static_cast<double>(height), 0.0, 1.0};
    MTLScissorRect fullScissor{0, 0, static_cast<NSUInteger>(width), static_cast<NSUInteger>(height)};
    [enc setViewport:fullViewport];
    [enc setScissorRect:fullScissor];
    [self drawSceneWithEncoder:enc
                viewProjection:viewProjection
                 triangleBuffer:triangleBuffer
                  triangleCount:triangleCount
                    lidarBuffer:lidarBuffer
                     lidarCount:lidarCount
                    radarBuffer:radarBuffer
                     radarCount:radarCount
            lightViewProjection:lightViewProjection
                  shadowStrength:0.25f
              skipSelectedDrone:_onboardCamera];
    [enc endEncoding];

    if (_insetCameraEnabled && !_sim.drones().empty()) {
        const auto& cfg = _sim.sensors();
        double frameW = 680.0;
        double frameH = 382.5;
        double margin = 18.0;
        double fit = std::min((width - margin * 2.0) / frameW, (height - margin * 2.0) / frameH);
        if (fit < 1.0) {
            fit = std::max(0.1, fit);
            frameW *= fit;
            frameH *= fit;
        }
        double frameX = std::max(0.0, width - frameW - margin);
        double frameY = std::max(0.0, height - frameH - margin);
        MTLViewport frameViewport{frameX, frameY, frameW, frameH, 0.0, 1.0};
        MTLScissorRect frameScissor{static_cast<NSUInteger>(frameX),
                                    static_cast<NSUInteger>(frameY),
                                    static_cast<NSUInteger>(frameW),
                                    static_cast<NSUInteger>(frameH)};

        double cameraAspect = static_cast<double>(std::max(1u, cfg.cameraWidth)) /
                              static_cast<double>(std::max(1u, cfg.cameraHeight));
        double contentW = frameW;
        double contentH = contentW / cameraAspect;
        if (contentH > frameH) {
            contentH = frameH;
            contentW = contentH * cameraAspect;
        }
        double contentX = frameX + (frameW - contentW) * 0.5;
        double contentY = frameY + (frameH - contentH) * 0.5;
        MTLViewport contentViewport{contentX, contentY, contentW, contentH, 0.0, 1.0};
        MTLScissorRect contentScissor{static_cast<NSUInteger>(contentX),
                                      static_cast<NSUInteger>(contentY),
                                      static_cast<NSUInteger>(contentW),
                                      static_cast<NSUInteger>(contentH)};

        MTLRenderPassDescriptor* insetPass = [MTLRenderPassDescriptor renderPassDescriptor];
        insetPass.colorAttachments[0].texture = drawable.texture;
        insetPass.colorAttachments[0].loadAction = MTLLoadActionLoad;
        insetPass.colorAttachments[0].storeAction = MTLStoreActionStore;
        insetPass.depthAttachment.texture = pass.depthAttachment.texture;
        insetPass.depthAttachment.loadAction = MTLLoadActionClear;
        insetPass.depthAttachment.storeAction = MTLStoreActionDontCare;
        insetPass.depthAttachment.clearDepth = 1.0;

        id<MTLRenderCommandEncoder> insetEnc = [command renderCommandEncoderWithDescriptor:insetPass];
        [insetEnc setRenderPipelineState:_pipeline];
        [insetEnc setFragmentTexture:_whiteTexture atIndex:0];
        [insetEnc setFragmentTexture:_shadowDepthTexture atIndex:1];
        [insetEnc setFragmentSamplerState:_defaultSampler atIndex:0];
        [insetEnc setFragmentSamplerState:_shadowSampler atIndex:1];
        [insetEnc setViewport:frameViewport];
        [insetEnc setScissorRect:frameScissor];
        [insetEnc setDepthStencilState:_overlayDepthState];
        [self drawOverlayBuffer:_insetBackgroundBuffer
                    vertexCount:_insetBackgroundVertexCount
                  primitiveType:MTLPrimitiveTypeTriangle
                       encoder:insetEnc];

        [insetEnc setViewport:contentViewport];
        [insetEnc setScissorRect:contentScissor];
        matrix_float4x4 insetProjection = makePerspective(70.0f * static_cast<float>(M_PI) / 180.0f,
                                                          static_cast<float>(cameraAspect),
                                                          0.05f,
                                                          1000.0f);
        matrix_float4x4 insetViewProjection = matrix_multiply(insetProjection, [self selectedDroneCameraViewMatrix]);
        [insetEnc setDepthStencilState:_depthState];
        [self drawSceneWithEncoder:insetEnc
                    viewProjection:insetViewProjection
                     triangleBuffer:triangleBuffer
                      triangleCount:triangleCount
                        lidarBuffer:nil
                         lidarCount:0
                        radarBuffer:nil
                         radarCount:0
                lightViewProjection:lightViewProjection
                      shadowStrength:0.42f
                  skipSelectedDrone:YES];

        [insetEnc setDepthStencilState:_overlayDepthState];
        if (lidarBuffer && lidarCount > 0) {
            setUniforms(_uniformBuffer, _uniformCursor, insetEnc, insetViewProjection, identityMatrix(), 2.5f);
            [insetEnc setVertexBuffer:lidarBuffer offset:0 atIndex:0];
            [insetEnc drawPrimitives:MTLPrimitiveTypePoint vertexStart:0 vertexCount:lidarCount];
        }
        if (radarBuffer && radarCount > 0) {
            setUniforms(_uniformBuffer, _uniformCursor, insetEnc, insetViewProjection, identityMatrix(), 4.0f);
            [insetEnc setVertexBuffer:radarBuffer offset:0 atIndex:0];
            [insetEnc drawPrimitives:MTLPrimitiveTypePoint vertexStart:0 vertexCount:radarCount];
        }
        [insetEnc setViewport:frameViewport];
        [insetEnc setScissorRect:frameScissor];
        std::vector<MetalVertex> insetBorderVertices = makeViewportBorderVertices(frameW, frameH, 4.0);
        id<MTLBuffer> insetBorderBuffer = makeMetalBuffer(_device, insetBorderVertices);
        [self drawOverlayBuffer:insetBorderBuffer
                    vertexCount:static_cast<NSUInteger>(insetBorderVertices.size())
                  primitiveType:MTLPrimitiveTypeTriangle
                       encoder:insetEnc];
        [insetEnc endEncoding];
    }

    [command presentDrawable:drawable];
    [command commit];
}

@end

@interface AppDelegate : NSObject <NSApplicationDelegate>
@end

@implementation AppDelegate {
    NSWindow* _window;
    Renderer* _renderer;
}

- (void)applicationDidFinishLaunching:(NSNotification*)notification {
    (void)notification;

    [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];

    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    if (!device) {
        std::cerr << "Metal is not available on this system\n";
        [NSApp terminate:nil];
        return;
    }

    NSRect frame = NSMakeRect(0, 0, 1200, 900);
    _window = [[NSWindow alloc] initWithContentRect:frame
                                          styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable | NSWindowStyleMaskResizable
                                            backing:NSBackingStoreBuffered
                                              defer:NO];
    [_window center];
    [_window setTitle:@"UAM Simulator Modern - Metal 3D"];

    InputMetalView* view = [[InputMetalView alloc] initWithFrame:frame device:device];
    view.preferredFramesPerSecond = 60;
    _renderer = [[Renderer alloc] initWithView:view];
    view.delegate = _renderer;

    [_window setContentView:view];
    [_window makeFirstResponder:view];
    [_window makeKeyAndOrderFront:nil];
    [NSApp activateIgnoringOtherApps:YES];
}

- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)sender {
    (void)sender;
    return YES;
}

@end

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    @autoreleasepool {
        NSApplication* app = [NSApplication sharedApplication];
        AppDelegate* delegate = [[AppDelegate alloc] init];
        [app setDelegate:delegate];
        [app run];
    }
    return 0;
}
