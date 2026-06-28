// UAM Simulato - Follow-cam fixed (no inversion), 3D LiDARE beams anchored to drone
#include <GL/glew.h>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <vector>
#include <optional>
#include <variant>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <memory>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ===================== Utilities / Math =====================
struct Pose {
    glm::vec3 pos{0.f, 1.0f, 0.f};
    float yaw = 0.f;   // degrees (CCW around +Y when seen from above)
    float pitch = 0.f; // degrees
};

// Forward from yaw/pitch: yaw=0 -> +X, yaw=90 -> +Z. CCW is left turn.
static inline glm::vec3 forwardFrom(const Pose& p){
    float y = glm::radians(p.yaw), pit = glm::radians(p.pitch);
    return glm::normalize(glm::vec3(std::cos(y)*std::cos(pit),
                                    std::sin(pit),
                                    std::sin(y)*std::cos(pit)));
}

#include <fstream>
#include <sstream>

// Hilfsfunktion zum Laden einer Textdatei (für Shader)
static std::string loadShaderSource(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "ERROR: Could not open shader file: " << filepath << std::endl;
        return "";
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    return buffer.str();
}

// Hilfsfunktion zum Kompilieren und Linken der Shader
static GLuint createShaderProgram(const std::string& vertPath, const std::string& fragPath) {
    std::string vertSource = loadShaderSource(vertPath);
    std::string fragSource = loadShaderSource(fragPath);

    if (vertSource.empty() || fragSource.empty()) return 0;

    const char* vertSourceC = vertSource.c_str();
    const char* fragSourceC = fragSource.c_str();
    
    // Vertex Shader
    GLuint vertShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertShader, 1, &vertSourceC, NULL);
    glCompileShader(vertShader);
    
    // Check Vertex Shader
    GLint success;
    glGetShaderiv(vertShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(vertShader, 512, NULL, infoLog);
        std::cerr << "ERROR: Vertex shader compilation failed\n" << infoLog << std::endl;
    }

    // Fragment Shader
    GLuint fragShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragShader, 1, &fragSourceC, NULL);
    glCompileShader(fragShader);

    // Check Fragment Shader
    glGetShaderiv(fragShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(fragShader, 512, NULL, infoLog);
        std::cerr << "ERROR: Fragment shader compilation failed\n" << infoLog << std::endl;
    }

    // Shader Program
    GLuint program = glCreateProgram();
    glAttachShader(program, vertShader);
    glAttachShader(program, fragShader);
    glLinkProgram(program);

    // Check Linking
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(program, 512, NULL, infoLog);
        std::cerr << "ERROR: Shader program linking failed\n" << infoLog << std::endl;
    }

    // Aufräumen
    glDeleteShader(vertShader);
    glDeleteShader(fragShader);

    std::cout << "Shader program loaded successfully!" << std::endl;
    return program;
}

// ===================== Drone =====================
struct Drone {
    Pose p;
    float speed    = 6.f;    // m/s
    float yawRate  = 90.f;   // deg/s
    float climb    = 3.f;    // m/s
    glm::vec3 bodyScale{0.5f, 0.15f, 0.5f}; // for body & shadow
};

// Revert updateDrone back to its original state:
static inline void updateDrone(Drone& d, float dt) {
    glm::vec3 fwd = forwardFrom(d.p);
    glm::vec3 right = glm::normalize(glm::cross(fwd, {0,1,0}));
    
    glm::vec3 move(0);
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) move += fwd;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)) move -= fwd;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) move -= right;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) move += right;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::C)) d.p.pos.y += d.climb * dt;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::V)) d.p.pos.y -= d.climb * dt;

    // Q/E control the drone's rotation directly (Q=LEFT/CCW, E=RIGHT/CW)
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Q)) d.p.yaw -= d.yawRate * dt; 
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::E)) d.p.yaw += d.yawRate * dt;

    if (glm::length(move) > 0.f) d.p.pos += glm::normalize(move) * (d.speed * dt);
}

static inline void drawDroneBox(const Pose& p, const glm::vec3& scale, GLuint shaderProgram, GLuint vao) {
    // 1. Berechne die Model-Matrix für die Drohne
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, p.pos);
    model = glm::rotate(model, glm::radians(-p.yaw),   glm::vec3(0,1,0));
    model = glm::rotate(model, glm::radians(p.pitch), glm::vec3(1,0,0)); // Achtung: original war (1,0,0)
    model = glm::scale(model, scale);

    // 2. Setze die Uniforms
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(model));
    glUniform3f(glGetUniformLocation(shaderProgram, "uObjectColor"), 0.15f, 0.15f, 0.15f); // Feste Farbe

    // 3. Zeichne den Würfel
    glBindVertexArray(vao);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);
    
    // HINWEIS: Die "Arme" (GL_LINES) sind hier nicht enthalten.
    // Um sie hinzuzufügen, müsstest du eine separate VAO/VBO für
    // Linien erstellen oder einen anderen Shader verwenden.
    // Lass sie für den Moment einfach weg.
}

// ===================== Grid =====================
static std::vector<float> generateGridVertices(int halfSize = 10) {
    std::vector<float> vertices;
    float lineColor[3] = {0.7f, 0.7f, 0.7f};
    float centerColor[3] = {0.3f, 0.3f, 0.3f};

    for (int x = -halfSize; x <= halfSize; x++) {
        bool isCenter = (x == 0);
        float r = isCenter ? centerColor[0] : lineColor[0];
        float g = isCenter ? centerColor[1] : lineColor[1];
        float b = isCenter ? centerColor[2] : lineColor[2];

        vertices.insert(vertices.end(), {
            static_cast<float>(x), 0.0f, static_cast<float>(-halfSize), r, g, b,
            static_cast<float>(x), 0.0f, static_cast<float>(halfSize),  r, g, b
        });
    }

    for (int z = -halfSize; z <= halfSize; z++) {
        bool isCenter = (z == 0);
        float r = isCenter ? centerColor[0] : lineColor[0];
        float g = isCenter ? centerColor[1] : lineColor[1];
        float b = isCenter ? centerColor[2] : lineColor[2];

        vertices.insert(vertices.end(), {
            static_cast<float>(-halfSize), 0.0f, static_cast<float>(z), r, g, b,
            static_cast<float>(halfSize),  0.0f, static_cast<float>(z), r, g, b
        });
    }
    return vertices;
}

// ===================== Light gizmo =====================
static void drawLightSource(const glm::vec3& lightPos) {
    glDisable(GL_LIGHTING);
    glPushMatrix();
    glTranslatef(lightPos.x, lightPos.y, lightPos.z);

    glColor3f(1.0f, 1.0f, 0.0f);
    const int segments = 8;
    const float radius = 0.3f;

    for (int i = 0; i < segments; i++) {
        float theta1 = (float)i * 2.0f * M_PI / segments;
        float theta2 = (float)(i + 1) * 2.0f * M_PI / segments;

        glBegin(GL_TRIANGLE_STRIP);
        for (int j = 0; j <= segments/2; j++) {
            float phi = (float)j * M_PI / (segments/2);

            float x1 = radius * std::sin(phi) * std::cos(theta1);
            float y1 = radius * std::cos(phi);
            float z1 = radius * std::sin(phi) * std::sin(theta1);

            float x2 = radius * std::sin(phi) * std::cos(theta2);
            float y2 = radius * std::cos(phi);
            float z2 = radius * std::sin(phi) * std::sin(theta2);

            glVertex3f(x1, y1, z1);
            glVertex3f(x2, y2, z2);
        }
        glEnd();
    }
    glPopMatrix();
}

// ===================== Drone projected shadow (rot-aware) =====================
static inline bool projectPointToGround(const glm::vec3& L, const glm::vec3& P, float yPlane, glm::vec3& out) {
    glm::vec3 dir = P - L;
    if (std::abs(dir.y) < 1e-6f) return false;
    float t = (yPlane - L.y) / dir.y;
    if (t <= 0.f) return false;
    out = L + t * dir;
    return true;
}

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

static void drawDroneProjectedShadow(const Drone& d, const glm::vec3& lightPos) {
    glm::mat4 M(1.0f);
    M = glm::translate(M, d.p.pos);
    M = glm::rotate(M, glm::radians(-d.p.yaw),   glm::vec3(0,1,0));
    M = glm::rotate(M, glm::radians(d.p.pitch), glm::vec3(1,0,0));
    M = glm::scale(M, d.bodyScale);

    glm::vec3 corners[8] = {
        {-0.5f,-0.5f,-0.5f}, {0.5f,-0.5f,-0.5f}, {0.5f, 0.5f,-0.5f}, {-0.5f, 0.5f,-0.5f},
        {-0.5f,-0.5f, 0.5f}, {0.5f,-0.5f, 0.5f}, {0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f}
    };

    std::vector<glm::vec3> proj; proj.reserve(8);
    float groundY = 0.01f;
    for (auto& c : corners) {
        glm::vec4 w = M * glm::vec4(c, 1.0f);
        glm::vec3 worldPt(w.x, w.y, w.z), p;
        if (projectPointToGround(lightPos, worldPt, groundY, p)) proj.push_back(p);
    }
    if (proj.size() < 3) return;

    auto hull = convexHullXZ(proj);
    if (hull.size() < 3) return;

    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glColor4f(0.f, 0.f, 0.f, 0.35f);
    glBegin(GL_TRIANGLE_FAN);
        for (auto& p : hull) glVertex3f(p.x, groundY, p.z);
    glEnd();
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

// ===================== Buildings =====================
class Object3D {
public:
    std::string name;
    glm::vec3 position;
    glm::vec3 dimensions;
    glm::vec3 color;

    Object3D(const YAML::Node& config) {
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

    void draw(GLuint shaderProgram, GLuint vao) {
        // 1. Berechne die Model-Matrix für dieses Objekt
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, position);
        // (Hier könntest du später auch Rotation einfügen)
        model = glm::scale(model, dimensions);

        // 2. Setze die Uniforms, die für dieses Objekt spezifisch sind
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(model));
        glUniform3fv(glGetUniformLocation(shaderProgram, "uObjectColor"), 1, glm::value_ptr(color));
        
        // 3. Zeichne den Würfel
        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, 36); // 36 Vertices
        glBindVertexArray(0);
    }

    void drawShadow(const glm::vec3& lightPos) {
        // simple, cheap shadow quad under building center
        glm::vec3 objectScale = dimensions;
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        glColor4f(0.0f, 0.0f, 0.0f, 0.18f);
        glm::vec3 center = position, p;
        if (projectPointToGround(lightPos, center, 0.01f, p)) {
            glPushMatrix();
            float lightHeight = std::max(0.1f, lightPos.y - 0.01f);
            float shadowScale = 1.0f + (center.y / lightHeight) * 0.5f;
            glTranslatef(p.x, 0.01f, p.z);
            glScalef(objectScale.x * shadowScale, 0.01f, objectScale.z * shadowScale);
            glBegin(GL_QUADS);
            glVertex3f(-0.5f, 0.0f, -0.5f);
            glVertex3f( 0.5f, 0.0f, -0.5f);
            glVertex3f( 0.5f, 0.0f,  0.5f);
            glVertex3f(-0.5f, 0.0f,  0.5f);
            glEnd();
            glPopMatrix();
        }
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }

    // Replace your Object3D::drawProjectedShadow with this
    static void drawProjectedShadow(const Object3D& obj,
                                    const glm::vec3& lightPos,
                                    float groundY = 0.01f){
        // Model matrix: translate · (optional rotate) · scale
        glm::mat4 M(1.0f);
        M = glm::translate(M, obj.position);

        // If you add rotation later, include it here, e.g.:
        // M = glm::rotate(M, glm::radians(obj.yawDeg), glm::vec3(0,1,0));

        M = glm::scale(M, obj.dimensions);

        // Unit cube corners in object space
        static const glm::vec3 corners[8] = {
            {-0.5f,-0.5f,-0.5f}, { 0.5f,-0.5f,-0.5f}, { 0.5f, 0.5f,-0.5f}, {-0.5f, 0.5f,-0.5f},
            {-0.5f,-0.5f, 0.5f}, { 0.5f,-0.5f, 0.5f}, { 0.5f, 0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f}
        };

        std::vector<glm::vec3> proj; proj.reserve(8);
        for (auto& c : corners) {
            glm::vec4 w = M * glm::vec4(c, 1.0f);      // world-space corner
            glm::vec3 p;
            if (projectPointToGround(lightPos, glm::vec3(w), groundY, p)) {
                proj.push_back(p);
            }
        }
        if (proj.size() < 3) return;

        auto hull = convexHullXZ(proj);
        if (hull.size() < 3) return;

        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
        glColor4f(0.f, 0.f, 0.f, 0.35f);

        glBegin(GL_TRIANGLE_FAN);
        for (auto& p : hull) glVertex3f(p.x, groundY, p.z);
        glEnd();

        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }

private:

};

// ===================== Ray / LiDAR =====================
struct AABB { glm::vec3 mn, mx; int id; };

static inline AABB makeAABB(const Object3D& o){
    glm::vec3 h = 0.5f * o.dimensions;
    return { o.position - h, o.position + h, 0 };
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

struct Hit3D { bool ok; float range; glm::vec3 point; int objId; glm::vec3 dir; };

static inline std::vector<Hit3D> simulateLidar3D(const Pose& p,
                                   const std::vector<AABB>& world,
                                   int beamsH, int beamsV,
                                   float fovH_deg, float fovV_deg,
                                   float maxR)
{
    std::vector<Hit3D> hits;
    hits.reserve(beamsH * beamsV);

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

            float best = maxR; int bestId=-1;
            for (const auto& b : world){
                float th; if (rayAABB(p.pos, dir, b, maxR, th)) {
                    if (th < best) { best = th; bestId = b.id; }
                }
            }
            if (bestId>=0) hits.push_back({true, best, p.pos + dir*best, bestId, dir});
            else           hits.push_back({false, maxR, p.pos + dir*maxR, -1, dir});
        }
    }
    return hits;
}

// Draw beams as thin red lines for hits (misses omitted)
static inline void drawLidar3DBeams(const Pose& p, const std::vector<Hit3D>& hits) {
    glLineWidth(1.0f);
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    for (const auto& h : hits) {
        if (!h.ok) continue;
        glVertex3f(p.pos.x, p.pos.y, p.pos.z);
        glVertex3f(h.point.x, h.point.y, h.point.z);
    }
    glEnd();
    glLineWidth(1.0f);
}

// (Optional) draw hit points, if you want a point cloud as well
static inline void drawLidar3DPoints(const std::vector<Hit3D>& hits, float maxR) {
    glPointSize(3.f);
    glBegin(GL_POINTS);
    for (const auto& h : hits) {
        if (!h.ok) continue;
        float t = h.range / maxR;
        glColor3f(1.f - t, 0.f, t);
        glVertex3f(h.point.x, h.point.y, h.point.z);
    }
    glEnd();
    glPointSize(1.f);
}

// ===================== RADAR (monostatic, simple) =====================
struct RadarParams {
    int   beamsH   = 60;
    int   beamsV   = 8;
    float fovH     = 90.f;   // deg
    float fovV     = 20.f;   // deg
    float maxR     = 150.f;  // m
    float minR     = 2.0f;   // blind zone
    float snr0     = 40.f;   // nominal at 1 m (dB-ish)
    float snrMin   = 8.f;    // threshold
};

struct RadarDet {
    bool ok;
    float range;     // meters
    float vr;        // radial velocity (m/s), + closing
    float az;        // radians
    float el;        // radians
    glm::vec3 point; // world pos
    int objId;
};

static inline std::vector<RadarDet> simulateRadar3D(
        const Pose& sensorPose,
        const glm::vec3& sensorVel, // drone velocity in world
        const std::vector<AABB>& world,
        const RadarParams& R)
{
    std::vector<RadarDet> dets;
    dets.reserve(R.beamsH * R.beamsV);

    float yaw = glm::radians(sensorPose.yaw), pit = glm::radians(sensorPose.pitch);
    glm::vec3 F = glm::normalize(glm::vec3(std::cos(yaw)*std::cos(pit), std::sin(pit), std::sin(yaw)*std::cos(pit)));
    glm::vec3 Rv = glm::normalize(glm::cross(F, glm::vec3(0,1,0)));
    glm::vec3 U  = glm::normalize(glm::cross(Rv, F));

    for (int v = 0; v < R.beamsV; ++v) {
        float el = ((v / (float)(R.beamsV-1)) - 0.5f) * glm::radians(R.fovV);
        for (int h = 0; h < R.beamsH; ++h) {
            float az = ((h / (float)(R.beamsH-1)) - 0.5f) * glm::radians(R.fovH);

            glm::vec3 dir = glm::normalize(F + std::tan(az)*Rv + std::tan(el)*U);

            float best = R.maxR; int bestId = -1;
            for (const auto& box : world) {
                float th;
                if (rayAABB(sensorPose.pos, dir, box, R.maxR, th)) {
                    if (th < best && th > R.minR) { best = th; bestId = box.id; }
                }
            }

            if (bestId < 0) {
                dets.push_back({false, R.maxR, 0.f, az, el, sensorPose.pos + dir*R.maxR, -1});
                continue;
            }

            // SNR ~ snr0 - 40 log10(R) (1/R^4 power law)
            float snr = R.snr0 - 40.f * std::log10(std::max(best, 1e-2f));
            if (snr < R.snrMin) {
                dets.push_back({false, R.maxR, 0.f, az, el, sensorPose.pos + dir*R.maxR, -1});
                continue;
            }

            // Radial velocity: projection of platform velocity onto LOS, + closing
            // float vr = glm::dot(sensorVel, dir) * (-1.0f); // flip to make +closing
            float vr = glm::dot(sensorVel, dir); // flip to make +closing

            dets.push_back({true, best, vr, az, el, sensorPose.pos + dir*best, bestId});
        }
    }
    return dets;
}

static inline void drawRadarBeams(const Pose& p, const std::vector<RadarDet>& dets) {
    // beams
    glLineWidth(1.0f);
    glColor3f(0.0f, 0.9f, 0.9f); // teal
    glBegin(GL_LINES);
    for (const auto& d : dets) {
        if (!d.ok) continue;
        glVertex3f(p.pos.x, p.pos.y, p.pos.z);
        glVertex3f(d.point.x, d.point.y, d.point.z);
    }
    glEnd();
    glLineWidth(1.0f);
}

static inline void drawRadarPoints(const std::vector<RadarDet>& dets) {
    // points colored by vr (red closing -> blue receding)
    glPointSize(3.f);
    glBegin(GL_POINTS);
    for (const auto& d : dets) {
        if (!d.ok) continue;
        float t = glm::clamp((d.vr + 10.f) / 20.f, 0.f, 1.f); // map -10..+10 m/s
        glColor3f(t, t +0.f, 1.f - t);
        glVertex3f(d.point.x, d.point.y, d.point.z);
    }
    glEnd();
    glPointSize(1.f);
}

// Helferfunktion zum Rendern aller würfelbasierten Objekte
static void renderScene(GLuint shaderProgram, GLuint vao, const std::vector<std::unique_ptr<Object3D>>& objects, const Drone& drone) {
    // Rendere Gebäude
    for (const auto& object : objects) {
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, object->position);
        model = glm::scale(model, object->dimensions);
        
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(model));
        
        // (Optional: Farbe/andere Uniforms, falls der Shader sie benötigt)
        if (glGetUniformLocation(shaderProgram, "uObjectColor") != -1) {
             glUniform3fv(glGetUniformLocation(shaderProgram, "uObjectColor"), 1, glm::value_ptr(object->color));
        }

        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, 36);
    }
    
    // Rendere Drohne
    glm::mat4 droneModel = glm::mat4(1.0f);
    droneModel = glm::translate(droneModel, drone.p.pos);
    droneModel = glm::rotate(droneModel, glm::radians(-drone.p.yaw),   glm::vec3(0,1,0));
    droneModel = glm::rotate(droneModel, glm::radians(drone.p.pitch), glm::vec3(1,0,0));
    droneModel = glm::scale(droneModel, drone.bodyScale);
    
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, glm::value_ptr(droneModel));
    
    if (glGetUniformLocation(shaderProgram, "uObjectColor") != -1) {
        glUniform3f(glGetUniformLocation(shaderProgram, "uObjectColor"), 0.15f, 0.15f, 0.15f);
    }
    
    glBindVertexArray(vao);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);
}

// ===================== main =====================
int main() {
    std::cout << "=== UAM Simulator - FollowCam fix + LiDAR anchored to drone ===\n";

    // Window
    sf::ContextSettings settings;
    settings.depthBits = 24;
    settings.stencilBits = 8;
    settings.antiAliasingLevel = 4;
    settings.majorVersion = 4;
    settings.minorVersion = 1;
    settings.attributeFlags = sf::ContextSettings::Core;

    sf::RenderWindow window(sf::VideoMode({1200, 900}),
                            "UAM Simulator",
                            sf::Style::Default,
                            sf::State::Windowed,
                            settings);

    if (!window.setActive(true))
        std::cerr << "Warning: Could not activate OpenGL context!\n";

    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
   if (GLEW_OK != err) {
        std::cerr << "Error: " << glewGetErrorString(err) << std::endl;
        return -1;
    }
    std::cout << "Using GLEW Version: " << glewGetString(GLEW_VERSION) << std::endl;
    // ===========================

    window.setVerticalSyncEnabled(true);
    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << "\n";
    std::cout << "Renderer: " << glGetString(GL_RENDERER) << "\n";

    GLuint gridShaderProgram = createShaderProgram("grid.vert", "grid.frag");
    if(gridShaderProgram == 0) {
        std::cerr << "Failed to create grid shader program. Exiting.\n";
        return -1;
    }

    // ----- Gitter VBO/VAO Setup -----
    std::vector<float> gridVertices = generateGridVertices(10);
    int gridVertexCount = gridVertices.size() / 6; // (pos(3) + color(3))

    GLuint gridVAO, gridVBO;

    // 1. VAO erstellen und binden
    glGenVertexArrays(1, &gridVAO);
    glBindVertexArray(gridVAO);

    // 2. VBO erstellen, binden und mit Daten füllen
    glGenBuffers(1, &gridVBO);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
    glBufferData(GL_ARRAY_BUFFER, 
                 gridVertices.size() * sizeof(float), 
                 gridVertices.data(), 
                 GL_STATIC_DRAW);

    // 3. Vertex Attribute Pointer setzen (sagen OpenGL, wie die Daten im VBO zu lesen sind)
    
    // layout (location = 0) in vec3 aPos;
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // layout (location = 1) in vec3 aColor;
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // 4. Bindung aufheben (gute Praxis)
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    GLuint objectShaderProgram = createShaderProgram("object.vert", "object.frag");
    if (objectShaderProgram == 0) return -1;

    // ----- Einheits-Würfel VBO/VAO Setup -----
    // 36 Vertices (6 pro Seite * 6 Seiten)
    // Jede Zeile: 3x Position, 3x Normale
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

    GLuint cubeVAO, cubeVBO;
    glGenVertexArrays(1, &cubeVAO);
    glBindVertexArray(cubeVAO);

    glGenBuffers(1, &cubeVBO);
    glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);

    // Stride (Schrittweite) ist jetzt 6 floats
    GLsizei stride = 6 * sizeof(float);

    // layout (location = 0) in vec3 aPos;
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
    glEnableVertexAttribArray(0);

    // layout (location = 1) in vec3 aNormal;
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0); // VAO freigeben 

    //  SHADOW
    GLuint shadowShaderProgram = createShaderProgram("shadow.vert", "shadow.frag");
    if (shadowShaderProgram == 0) return -1;

    // --- Shadow Map FBO Setup ---
    const unsigned int SHADOW_WIDTH = 4096, SHADOW_HEIGHT = 4096; // Hohe Auflösung für Qualität
    GLuint depthMapFBO;
    GLuint depthMap;

    // 1. Framebuffer Objekt (FBO) erstellen
    glGenFramebuffers(1, &depthMapFBO);

    // 2. Depth Texture (Shadow Map) erstellen
    glGenTextures(1, &depthMap);
    glBindTexture(GL_TEXTURE_2D, depthMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, 
                SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);

    // Filtereinstellungen
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    // Clamp-to-Border verhindert, dass Koordinaten außerhalb des Lichts Schattendaten wiederholen
    float borderColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        
    // 3. Depth Texture an FBO anhängen
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthMap, 0);

    // Wir rendern KEINE Farben, nur Tiefe
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cerr << "ERROR::FRAMEBUFFER:: Shadow Map FBO is not complete!" << std::endl;
        
    glBindFramebuffer(GL_FRAMEBUFFER, 0); // Zurück zum Standard-Framebuffer

    // GL state
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);



    // Load objects
    std::vector<std::unique_ptr<Object3D>> objects;
    try {
        YAML::Node config = YAML::LoadFile("config.yaml");
        for (const auto& objectConfig : config["objects"]) {
            objects.push_back(std::make_unique<Object3D>(objectConfig));
        }
        std::cout << "Loaded " << objects.size() << " objects\n";
    } catch (const YAML::Exception& ) {
        std::cerr << "Error loading config.yaml, creating default objects...\n";
        YAML::Node o1; o1["name"]="RedCube";   o1["position"]=std::vector<float>{0,1,0};  o1["dimensions"]=std::vector<float>{1,1,1};   o1["color"]=std::vector<float>{1,0,0};
        YAML::Node o2; o2["name"]="GreenCube"; o2["position"]=std::vector<float>{3,1,2};  o2["dimensions"]=std::vector<float>{1,2,1};   o2["color"]=std::vector<float>{0,1,0};
        YAML::Node o3; o3["name"]="BlueCube";  o3["position"]=std::vector<float>{-2,0.5f,1}; o3["dimensions"]=std::vector<float>{1,0.5f,1}; o3["color"]=std::vector<float>{0,0,1};
        objects.push_back(std::make_unique<Object3D>(o1));
        objects.push_back(std::make_unique<Object3D>(o2));
        objects.push_back(std::make_unique<Object3D>(o3));
    }

    // Static AABBs for buildings
    std::vector<AABB> worldAABBs;
    worldAABBs.reserve(objects.size());
    for (size_t i=0;i<objects.size(); ++i) {
        auto box = makeAABB(*objects[i]);
        box.id = (int)i;
        worldAABBs.push_back(box);
    }

    // Grid
    // std::vector<float> gridVertices = generateGridVertices(10);

    // Light & shadow
    glm::vec3 lightPosition(5.0f, 8.0f, 3.0f);
    bool showLightSource = true;
    bool enableShadows = true;

    // Camera state (free-cam values used only when followDrone=false)
    glm::vec3 cameraPos  = glm::vec3(0.0f, 3.0f, 8.0f);
    glm::vec3 cameraUp   = glm::vec3(0.0f, 1.0f, 0.0f);
    float yaw   = -90.0f;
    float pitch = -10.0f;
    const float yawSpeed = 60.0f;
    bool enableMouseLook = false;
    bool followDrone = true;
    sf::Vector2i windowCenter(window.getSize().x / 2, window.getSize().y / 2);

    auto updateCameraFront = [&](float yaw_, float pitch_) -> glm::vec3 {
        glm::vec3 front;
        front.x = std::cos(glm::radians(yaw_)) * std::cos(glm::radians(pitch_));
        front.y = std::sin(glm::radians(pitch_));
        front.z = std::sin(glm::radians(yaw_)) * std::cos(glm::radians(pitch_));
        return glm::normalize(front);
    };
    glm::vec3 cameraFront = updateCameraFront(yaw, pitch);
    glm::vec3 cameraRight = glm::normalize(glm::cross(cameraFront, cameraUp));
    float velocity = 5.0f;

    // Drone: start looking along -Z like your original camera feel
    Drone drone;
    drone.p.yaw = -90.f;

    // Radar + velocity tracking
    bool enableRadar = false;
    RadarParams radarCfg;
    glm::vec3   prevDronePos = drone.p.pos;
    bool        prevPosValid = false;
    glm::vec3   droneVel     = glm::vec3(0);

    sf::Clock deltaClock;
    sf::Clock fpsCounter;
    int frameCount = 0;

    // Lidar
    bool enableLidar = false;

    // Events
    const auto onClose = [&](const sf::Event::Closed&) { window.close(); };

    const auto onKeyPressed = [&](const sf::Event::KeyPressed& keyPressed) {
        using sc = sf::Keyboard::Scancode;
        if (keyPressed.scancode == sc::M) {
            enableMouseLook = !enableMouseLook;
            window.setMouseCursorVisible(!enableMouseLook);
            if (enableMouseLook) sf::Mouse::setPosition(windowCenter, window);
            std::cout << "Mouse look: " << (enableMouseLook ? "ON" : "OFF") << "\n";
        }
        else if (keyPressed.scancode == sc::B) {
            showLightSource = !showLightSource;
            std::cout << "Light gizmo: " << (showLightSource ? "ON" : "OFF") << "\n";
        }
        else if (keyPressed.scancode == sc::H) {
            enableShadows = !enableShadows;
            std::cout << "Shadows: " << (enableShadows ? "ON" : "OFF") << "\n";
        }
        else if (keyPressed.scancode == sc::F) {
            followDrone = !followDrone;
            std::cout << "Follow drone: " << (followDrone ? "ON" : "OFF") << "\n";
        }
        else if (keyPressed.scancode == sc::Escape) {
            window.close();
        }
        else if (keyPressed.scancode == sc::L) {
            enableLidar = !enableLidar;
            std::cout << "Lidar: " << (enableLidar ? "ON" : "OFF") << "\n";
        }
        else if (keyPressed.scancode == sc::R) {
            enableRadar = !enableRadar;
            std::cout << "Radar: " << (enableRadar ? "ON" : "OFF") << "\n";
        }

        // Light control
        float lightSpeed = 0.5f;
        if (keyPressed.scancode == sc::Up)      lightPosition.z -= lightSpeed;
        if (keyPressed.scancode == sc::Down)    lightPosition.z += lightSpeed;
        if (keyPressed.scancode == sc::Left)    lightPosition.x -= lightSpeed;
        if (keyPressed.scancode == sc::Right)   lightPosition.x += lightSpeed;
        if (keyPressed.scancode == sc::PageUp)  lightPosition.y += lightSpeed;
        if (keyPressed.scancode == sc::PageDown)lightPosition.y -= lightSpeed;
    };

    const auto onMouseMoved = [&](const sf::Event::MouseMoved& mouseMoved) {
        if (enableMouseLook && !followDrone) {
            float xoffset = static_cast<float>(mouseMoved.position.x - windowCenter.x);
            float yoffset = static_cast<float>(windowCenter.y - mouseMoved.position.y);
            sf::Mouse::setPosition(windowCenter, window);

            float sensitivity = 0.1f;
            yaw   += xoffset * sensitivity;
            pitch += yoffset * sensitivity;

            if (pitch > 89.0f)  pitch = 89.0f;
            if (pitch < -89.0f) pitch = -89.0f;

            cameraFront = updateCameraFront(yaw, pitch);
            cameraRight = glm::normalize(glm::cross(cameraFront, cameraUp));
        }
    };

    std::cout << "\n=== Controls ===\n";
    std::cout << "WASD: Drone move   Q/E: Yaw (Q=left/CCW)   C/V: Up/Down\n";
    std::cout << "F: Toggle follow camera   M: Toggle mouse look (free cam)\n";
    std::cout << "Arrows + PgUp/PgDn: Move light   B/H: Gizmo/Shadows   ESC: Exit\n";

    // LiDAR config
    const int   beamsH   = 360;
    const int   beamsV   = 16;
    const float fovH_deg = 360.f;
    const float fovV_deg = 45.f;
    const float lidarMax = 50.f;

    // Main loop
    float dtSmooth = 1.f/60.f;
    while (window.isOpen()) {
        float dt = deltaClock.restart().asSeconds();
        dt = std::min(dt, 0.1f);
        dtSmooth = 0.9f*dtSmooth + 0.1f*dt;

        window.handleEvents(
            onClose,
            [](const sf::Event::Resized&){},
            onKeyPressed,
            [](const sf::Event::KeyReleased&){},
            [](const sf::Event::MouseButtonPressed&){},
            [](const sf::Event::MouseButtonReleased&){},
            onMouseMoved,
            [](const sf::Event::MouseWheelScrolled&){}
        );

        

        // Update drone pose first (so camera & LiDAR use the new pose)
        
        // Camera
        if (!followDrone) {
            // free-cam mode
            glm::vec3 movement(0.0f);
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W)) movement += cameraFront;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S)) movement -= cameraFront;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) movement -= cameraRight;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) movement += cameraRight;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::C)) movement += cameraUp;
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::V)) movement -= cameraUp;
            
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::E)) {
                yaw += yawSpeed * dtSmooth; // keep free-cam E = right
                cameraFront = updateCameraFront(yaw, pitch);
                cameraRight = glm::normalize(glm::cross(cameraFront, cameraUp));
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Q)) {
                yaw -= yawSpeed * dtSmooth; // free-cam Q = left
                cameraFront = updateCameraFront(yaw, pitch);
                cameraRight = glm::normalize(glm::cross(cameraFront, cameraUp));
            }
            
            if (glm::length(movement) > 0.0f) {
                movement = glm::normalize(movement) * velocity * dtSmooth;
                cameraPos += movement;
            }
        } else {
            updateDrone(drone, dtSmooth);
            
            // Drone velocity (for RADAR Doppler)
            if (!prevPosValid) { prevPosValid = true; prevDronePos = drone.p.pos; }
            droneVel     = (drone.p.pos - prevDronePos) / std::max(dtSmooth, 1e-4f);
            prevDronePos = drone.p.pos;

            // FOLLOW MODE — deterministic, no smoothing, no inversion
            // Use drone yaw to place camera directly behind it and look forward
            const float followBack = 4.0f;
            const float followUp   = 2.0f;
            const float followLead = 2.0f;

            // Horizontal forward from *drone yaw* only (ignore drone pitch for camera placement)
            float y = glm::radians(drone.p.yaw);
            glm::vec3 F_cam(std::cos(y), 0.0f, std::sin(y)); F_cam = glm::normalize(F_cam);
            glm::vec3 U(0,1,0);

            cameraPos   = drone.p.pos - F_cam * followBack + U * followUp;
            glm::vec3 target = drone.p.pos + F_cam * followLead;
            
            cameraFront = glm::normalize(target - cameraPos);
            cameraRight = glm::normalize(glm::cross(cameraFront, U));
        }

        // Render
        glClearColor(0.8f, 0.9f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        float aspectRatio = static_cast<float>(window.getSize().x) / static_cast<float>(window.getSize().y);
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), aspectRatio, 0.5f, 80.0f);
        glLoadMatrixf(glm::value_ptr(projection));

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
        glLoadMatrixf(glm::value_ptr(view));

        // Grid
        // Stattdessen haben wir bereits die Matrizen von glm!
        // float aspectRatio = static_cast<float>(window.getSize().x) / static_cast<float>(window.getSize().y);
        // glm::mat4 projection = glm::perspective(glm::radians(45.0f), aspectRatio, 0.5f, 80.0f);
        // glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);

        // --- Light Space Matrizen berechnen (einmal pro Frame)
        float near_plane = 1.0f, far_plane = 70.0f; 
        glm::mat4 lightProjection = glm::ortho(-20.0f, 20.0f, -20.0f, 20.0f, near_plane, far_plane); // Orthogonal für ein gerichtetes/paralleles Licht
        glm::mat4 lightView = glm::lookAt(lightPosition, glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        glm::mat4 lightSpaceMatrix = lightProjection * lightView;


        // --- 1. Schatten Pass (Tiefe rendern) ---
        glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
        glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
        glClear(GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        // glCullFace(GL_FRONT); // Optional: Kann Shadow Acne reduzieren, aber erfordert saubere Modelle

        glUseProgram(shadowShaderProgram);
        glUniformMatrix4fv(glGetUniformLocation(shadowShaderProgram, "uLightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(lightSpaceMatrix));

        renderScene(shadowShaderProgram, cubeVAO, objects, drone);

        // glCullFace(GL_BACK); // Zurücksetzen
        glBindFramebuffer(GL_FRAMEBUFFER, 0); // Zurück zum Fenster-Framebuffer


        // --- 2. Haupt Pass (Szene rendern) ---
        glViewport(0, 0, window.getSize().x, window.getSize().y);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);

        // === Gitter zeichnen (Modern) ===
        glDisable(GL_LIGHTING); // <-- Diese alten Aufrufe sind jetzt wirkungslos, aber ok
        
        glUseProgram(gridShaderProgram); // 1. Shader aktivieren
        
        // 2. Uniforms setzen (Matrizen an den Shader senden)
        glUniformMatrix4fv(glGetUniformLocation(gridShaderProgram, "uProjection"), 1, GL_FALSE, glm::value_ptr(projection));
        glUniformMatrix4fv(glGetUniformLocation(gridShaderProgram, "uView"), 1, GL_FALSE, glm::value_ptr(view));

        // 3. VAO binden und zeichnen
        glBindVertexArray(gridVAO);
        glDrawArrays(GL_LINES, 0, gridVertexCount); // Zeichne die Linien!
        // glBindVertexArray(0); // 4. VAO lösen
        
        glUseProgram(0); // 5. Shader deaktivieren

        // Shadows
        if (enableShadows) {
            for (const auto& object : objects)
                Object3D::drawProjectedShadow(*object, lightPosition, 0.01f);
            drawDroneProjectedShadow(drone, lightPosition);
        }

        glUseProgram(objectShaderProgram); // 1. Objekt-Shader aktivieren
        
        // Setze die Uniforms, die für ALLE Objekte gleich sind
        glUniformMatrix4fv(glGetUniformLocation(objectShaderProgram, "uProjection"), 1, GL_FALSE, glm::value_ptr(projection));
        glUniformMatrix4fv(glGetUniformLocation(objectShaderProgram, "uView"), 1, GL_FALSE, glm::value_ptr(view));
        glUniform3fv(glGetUniformLocation(objectShaderProgram, "uLightPos"), 1, glm::value_ptr(lightPosition));

        // Schatten Uniforms senden
        glUniformMatrix4fv(glGetUniformLocation(objectShaderProgram, "uLightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(lightSpaceMatrix));
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, depthMap);
        glUniform1i(glGetUniformLocation(objectShaderProgram, "uShadowMap"), 0); // Bindet GL_TEXTURE0 an uShadowMap

        // Rendere alle Objekte
        renderScene(objectShaderProgram, cubeVAO, objects, drone);

        glUseProgram(0);
        glDisable(GL_DEPTH_TEST);

        // Buildings
        // for (const auto& object : objects) {
        //     object->draw(objectShaderProgram, cubeVAO);
        // }

        // Drone
        drawDroneBox(drone.p, drone.bodyScale, objectShaderProgram, cubeVAO);

        // Light
        // if (showLightSource) drawLightSource(lightPosition);

        // LiDAR (anchored to the DRONE pose)
        auto hits = simulateLidar3D(drone.p, worldAABBs, beamsH, beamsV, fovH_deg, fovV_deg, lidarMax);
        
        if(enableLidar)
        // drawLidar3DBeams(drone.p, hits);
        drawLidar3DPoints(hits, lidarMax); // optional

        // RADAR (anchored to drone)
        auto radarDets = simulateRadar3D(drone.p, droneVel, worldAABBs, radarCfg);
        
        if(enableRadar)
        // drawRadarBeams(drone.p, radarDets);
        drawRadarPoints(radarDets);

        window.display();

        // FPS
        frameCount++;
        if (fpsCounter.getElapsedTime().asSeconds() >= 1.0f) {
            std::cout << "FPS: " << frameCount << " | Light: ("
                      << lightPosition.x << ", " << lightPosition.y << ", " << lightPosition.z << ")\n";
            frameCount = 0;
            fpsCounter.restart();
        }
    }

    // Am Ende von main(), vor return 0:
    glDeleteVertexArrays(1, &gridVAO);
    glDeleteBuffers(1, &gridVBO);
    glDeleteProgram(gridShaderProgram);

    glDeleteVertexArrays(1, &cubeVAO);
    glDeleteBuffers(1, &cubeVBO);
    glDeleteProgram(objectShaderProgram);

    std::cout << "Exiting successfully!\n";
    return 0;
}

