#version 410 core

// Inputs vom Vertex Shader
in vec3 vFragPos;
in vec3 vNormal;
in vec2 vTex;

// Output-Farbe
out vec4 FragColor;

// Uniforms (Daten vom C++ Code)
uniform vec3 uObjectColor; // Die Grundfarbe des Objekts
uniform vec3 uLightPos;    // Die Position der Lichtquelle
uniform sampler2D uTexture;
uniform int uUseTexture;

// NEUE UNIFORMS FÜR SCHATTEN
uniform sampler2D uShadowMap;
in vec4 vLightSpacePos;

float calculateShadow(vec3 norm, vec3 lightDir)
{
    // 1. Position des Fragments im Light Space berechnen und auf [0, 1] normalisieren
    vec3 projCoords = vLightSpacePos.xyz / vLightSpacePos.w;
    projCoords = projCoords * 0.5 + 0.5;
    
    // 2. Fragment außerhalb des Licht-Frustums rendert keinen Schatten
    if(projCoords.z > 1.0) return 0.0;

    // 3. Tiefenwert aus der Shadow Map abrufen
    float closestDepth = texture(uShadowMap, projCoords.xy).r; 
    
    // 4. Aktuelle Fragmenttiefe
    float currentDepth = projCoords.z;
    
    // 5. Biasing (Schattenakne vermeiden)
    // Wir nutzen ein Normalenbasiertes Bias für bessere Ergebnisse
    float bias = max(0.05 * (1.0 - dot(norm, lightDir)), 0.005);
    
    // 6. Schatten-Check (Wir verwenden PCF für weichere Ränder)
    float shadow = 0.0;
    vec2 texelSize = 1.0 / textureSize(uShadowMap, 0);
    for (int x = -2; x <= 2; ++x)
    {
        for (int y = -2; y <= 2; ++y)
        {
            float pcfDepth = texture(uShadowMap, projCoords.xy + vec2(x, y) * texelSize).r; 
            shadow += currentDepth - bias > pcfDepth ? 1.0 : 0.0;        
        }    
    }
    shadow /= 25.0;
    
    return shadow;
}

void main()
{
    // 1. BELEUCHTUNG BERECHNEN (wie zuvor)
    vec3 norm = normalize(vNormal);
    vec3 lightDir = normalize(uLightPos - vFragPos);
    
    float ambientStrength = 0.3;
    vec3 baseColor = (uUseTexture == 1) ? texture(uTexture, vTex).rgb : uObjectColor;
    vec3 ambient = ambientStrength * baseColor;
    
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * baseColor;

    // 2. SCHATTEN BERECHNEN
    float shadowFactor = calculateShadow(norm, lightDir);
    
    // 3. ENDFARBE
    // Der diffuse Anteil wird durch den Schattenfaktor gedämpft. 
    // Der Umgebungslicht-Anteil (ambient) bleibt immer sichtbar, damit der Schatten nicht tiefschwarz ist.
    vec3 lighting = ambient + (1.0 - shadowFactor) * diffuse;

    FragColor = vec4(lighting, 1.0);
}
