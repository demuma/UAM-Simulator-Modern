#version 410 core

layout(location = 0) in vec3 aPos;

uniform mat4 uProjection;
uniform mat4 uView;
uniform mat4 uLightSpaceMatrix;

out vec4 vLightSpacePos;
out vec3 vNormal;

void main() {
	vec4 worldPos = vec4(aPos, 1.0);
	gl_Position = uProjection * uView * worldPos;
	vLightSpacePos = uLightSpaceMatrix * worldPos;
	vNormal = vec3(0.0, 1.0, 0.0);
}
