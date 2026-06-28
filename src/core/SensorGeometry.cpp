#include "core/SensorGeometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace uam {
namespace {

void expand(glm::vec3& mn, glm::vec3& mx, const glm::vec3& p) {
    mn = glm::min(mn, p);
    mx = glm::max(mx, p);
}

glm::vec3 centroid(const SensorGeometry::Triangle& tri) {
    return (tri.a + tri.b + tri.c) / 3.0f;
}

void triBounds(const SensorGeometry::Triangle& tri, glm::vec3& mn, glm::vec3& mx) {
    mn = glm::min(tri.a, glm::min(tri.b, tri.c));
    mx = glm::max(tri.a, glm::max(tri.b, tri.c));
}

bool rayBounds(const glm::vec3& ro, const glm::vec3& rd, const glm::vec3& mn, const glm::vec3& mx, float tMax) {
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

bool rayTriangle(const glm::vec3& ro, const glm::vec3& rd, const SensorGeometry::Triangle& tri, float tMax, float& tHit) {
    const float eps = 1e-6f;
    glm::vec3 e1 = tri.b - tri.a;
    glm::vec3 e2 = tri.c - tri.a;
    glm::vec3 h = glm::cross(rd, e2);
    float det = glm::dot(e1, h);
    if (std::abs(det) < eps) return false;

    float invDet = 1.0f / det;
    glm::vec3 s = ro - tri.a;
    float u = invDet * glm::dot(s, h);
    if (u < 0.0f || u > 1.0f) return false;

    glm::vec3 q = glm::cross(s, e1);
    float v = invDet * glm::dot(rd, q);
    if (v < 0.0f || u + v > 1.0f) return false;

    float t = invDet * glm::dot(e2, q);
    if (t <= 0.001f || t > tMax) return false;
    tHit = t;
    return true;
}

} // namespace

void SensorGeometry::clear() {
    triangles_.clear();
    indices_.clear();
    bvh_.clear();
    boundsMin_ = glm::vec3(0.0f);
    boundsMax_ = glm::vec3(0.0f);
    hasBounds_ = false;
}

void SensorGeometry::addMesh(const SceneMesh& mesh, int objectId) {
    if (mesh.vertices.size() < 3) return;
    for (std::size_t i = 0; i + 2 < mesh.vertices.size(); i += 3) {
        Triangle tri{mesh.vertices[i + 0].position,
                     mesh.vertices[i + 1].position,
                     mesh.vertices[i + 2].position,
                     objectId};
        triangles_.push_back(tri);

        if (!hasBounds_) {
            boundsMin_ = tri.a;
            boundsMax_ = tri.a;
            hasBounds_ = true;
        }
        expand(boundsMin_, boundsMax_, tri.a);
        expand(boundsMin_, boundsMax_, tri.b);
        expand(boundsMin_, boundsMax_, tri.c);
    }
}

void SensorGeometry::build() {
    indices_.clear();
    bvh_.clear();
    if (triangles_.empty()) return;

    indices_.resize(triangles_.size());
    std::iota(indices_.begin(), indices_.end(), 0);
    bvh_.reserve(triangles_.size() / 4 + 1);
    buildRecursive(0, indices_.size());
}

int SensorGeometry::buildRecursive(std::size_t start, std::size_t end) {
    BVHNode node;
    node.start = start;
    node.count = end - start;

    glm::vec3 mn(std::numeric_limits<float>::max());
    glm::vec3 mx(std::numeric_limits<float>::lowest());
    glm::vec3 cmn(std::numeric_limits<float>::max());
    glm::vec3 cmx(std::numeric_limits<float>::lowest());

    for (std::size_t i = start; i < end; ++i) {
        const Triangle& tri = triangles_[static_cast<std::size_t>(indices_[i])];
        glm::vec3 tmn, tmx;
        triBounds(tri, tmn, tmx);
        expand(mn, mx, tmn);
        expand(mn, mx, tmx);
        expand(cmn, cmx, centroid(tri));
    }

    node.mn = mn;
    node.mx = mx;
    int nodeIndex = static_cast<int>(bvh_.size());
    bvh_.push_back(node);

    constexpr std::size_t leafSize = 8;
    if (node.count <= leafSize) return nodeIndex;

    glm::vec3 extent = cmx - cmn;
    int axis = 0;
    if (extent.y > extent.x && extent.y >= extent.z) axis = 1;
    else if (extent.z > extent.x && extent.z >= extent.y) axis = 2;

    std::size_t mid = start + node.count / 2;
    if (extent[axis] > 1e-5f) {
        std::nth_element(indices_.begin() + static_cast<std::ptrdiff_t>(start),
                         indices_.begin() + static_cast<std::ptrdiff_t>(mid),
                         indices_.begin() + static_cast<std::ptrdiff_t>(end),
                         [&](int lhs, int rhs) {
                             return centroid(triangles_[static_cast<std::size_t>(lhs)])[axis] <
                                    centroid(triangles_[static_cast<std::size_t>(rhs)])[axis];
                         });
    }

    if (mid == start || mid == end) return nodeIndex;

    int left = buildRecursive(start, mid);
    int right = buildRecursive(mid, end);
    bvh_[static_cast<std::size_t>(nodeIndex)].left = left;
    bvh_[static_cast<std::size_t>(nodeIndex)].right = right;
    bvh_[static_cast<std::size_t>(nodeIndex)].count = 0;
    return nodeIndex;
}

SensorGeometry::RayHit SensorGeometry::raycast(const glm::vec3& origin, const glm::vec3& dir, float maxRange) const {
    RayHit best{false, maxRange, -1};
    if (bvh_.empty()) return best;

    std::vector<int> stack;
    stack.reserve(64);
    stack.push_back(0);
    while (!stack.empty()) {
        int nodeIndex = stack.back();
        stack.pop_back();
        const BVHNode& node = bvh_[static_cast<std::size_t>(nodeIndex)];
        if (!rayBounds(origin, dir, node.mn, node.mx, best.t)) continue;

        if (node.leaf()) {
            for (std::size_t i = node.start; i < node.start + node.count; ++i) {
                const Triangle& tri = triangles_[static_cast<std::size_t>(indices_[i])];
                float t = 0.0f;
                if (rayTriangle(origin, dir, tri, best.t, t) && t < best.t) {
                    best = {true, t, tri.objectId};
                }
            }
        } else {
            if (node.left >= 0) stack.push_back(node.left);
            if (node.right >= 0) stack.push_back(node.right);
        }
    }
    return best;
}

std::vector<LidarHit> SensorGeometry::simulateLidar(const glm::vec3& origin,
                                                    float yawDeg,
                                                    float pitchDeg,
                                                    int beamsH,
                                                    int beamsV,
                                                    float fovHDeg,
                                                    float fovVDeg,
                                                    float maxRange) const {
    std::vector<LidarHit> hits;
    beamsH = std::max(1, beamsH);
    beamsV = std::max(1, beamsV);
    hits.reserve(static_cast<std::size_t>(beamsH * beamsV));
    if (triangles_.empty()) return hits;

    float rayLimit = maxRange > 0.0f ? maxRange : 1000.0f;
    if (maxRange <= 0.0f && hasBounds_) {
        glm::vec3 farCorner = glm::max(glm::abs(boundsMin_ - origin), glm::abs(boundsMax_ - origin));
        rayLimit = glm::length(farCorner) + 100.0f;
    }

    float baseYaw = glm::radians(yawDeg);
    float basePitch = glm::radians(pitchDeg);
    float denomH = static_cast<float>(std::max(1, beamsH - 1));
    float denomV = static_cast<float>(std::max(1, beamsV - 1));
    for (int v = 0; v < beamsV; ++v) {
        float el = ((static_cast<float>(v) / denomV) - 0.5f) * glm::radians(fovVDeg);
        constexpr float halfPi = 1.57079632679f;
        float pitch = glm::clamp(basePitch + el, -halfPi + 1e-4f, halfPi - 1e-4f);
        float cosPitch = std::cos(pitch);

        for (int h = 0; h < beamsH; ++h) {
            float az = ((static_cast<float>(h) / denomH) - 0.5f) * glm::radians(fovHDeg);
            float yaw = baseYaw + az;
            glm::vec3 dir = glm::normalize(glm::vec3(std::cos(yaw) * cosPitch,
                                                     std::sin(pitch),
                                                     std::sin(yaw) * cosPitch));
            RayHit ray = raycast(origin, dir, rayLimit);
            if (ray.hit) hits.push_back({true, ray.t, origin + dir * ray.t, dir, ray.objectId});
            else hits.push_back({false, rayLimit, origin + dir * rayLimit, dir, -1});
        }
    }
    return hits;
}

} // namespace uam
