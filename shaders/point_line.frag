// point_line.frag
#version 410 core
out vec4 FragColor;
uniform vec3 uColor;
uniform bool uIsPoint;

void main()
{   
    // Make points circular; skip for lines/triangles
    if (uIsPoint) {
        vec2 c = gl_PointCoord - vec2(0.5);
        if (dot(c, c) > 0.25) discard;
    }
    FragColor = vec4(uColor, 1.0);
}
