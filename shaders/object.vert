#version 410 core

layout (location = 0) in vec3 aPos;    // Vertex-Position (aus dem VBO)
layout (location = 1) in vec3 aNormal; // Normalenvektor (aus dem VBO)
layout (location = 2) in vec2 aTex;    // UV

// Outputs an den Fragment Shader
out vec3 vFragPos;  // Position im Weltraum
out vec3 vNormal;   // Normale im Weltraum
out vec2 vTex;

// Uniforms (Daten vom C++ Code)
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;

// NEUE UNIFORM
uniform mat4 uLightSpaceMatrix;

// NEUER OUTPUT
out vec4 vLightSpacePos; // Position des Vertex im Licht-Space

void main()
{
    // Berechne die finale Bildschirm-Position
    gl_Position = uProjection * uView * uModel * vec4(aPos, 1.0);
    
    // Berechne Position und Normale im Weltraum für die Beleuchtung
    // (wird vom Fragment Shader verwendet)
    vFragPos = vec3(uModel * vec4(aPos, 1.0));
    
    // Normalen korrekt transformieren (besonders bei Skalierung)
    vNormal = mat3(transpose(inverse(uModel))) * aNormal;

    vTex = aTex;

    // Position des Vertex im Light-Space
    vLightSpacePos = uLightSpaceMatrix * uModel * vec4(aPos, 1.0);
}
