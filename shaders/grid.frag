// // #version 410 core

// // // Input vom Vertex Shader
// // in vec3 vColor;

// // // Output (die endgültige Pixelfarbe)
// // out vec4 FragColor;

// // void main()
// // {
// //     FragColor = vec4(vColor, 1.0);
// // }

// #version 410 core

// // Input vom Vertex Shader
// in vec3 vColor;
// // NEUE INPUTS
// in vec4 vLightSpacePos;

// // Output (die endgültige Pixelfarbe)
// out vec4 FragColor;

// // NEUE UNIFORMS FÜR SCHATTEN
// uniform sampler2D uShadowMap;
// uniform vec3 uLightPos; // Benötigt für Normalenbasiertes Bias

// // Die Shadow-Kalkulationsfunktion (vom object.frag übernommen)
// float calculateShadow()
// {
//     // 1. Position des Fragments im Light Space berechnen und auf [0, 1] normalisieren
//     vec3 projCoords = vLightSpacePos.xyz / vLightSpacePos.w;
//     projCoords = projCoords * 0.5 + 0.5;
//     
//     // 2. Fragment außerhalb des Licht-Frustums rendert keinen Schatten
//     // Da wir nur den Boden zeichnen, ist dies weniger kritisch, aber gut zu behalten.
//     if(projCoords.z > 1.0) return 0.0;

//     // 3. Tiefenwert aus der Shadow Map abrufen
//     // float closestDepth = texture(uShadowMap, projCoords.xy).r; // Not needed for PCF start

//     // 4. Aktuelle Fragmenttiefe
//     float currentDepth = projCoords.z;
//     
//     // 5. Biasing (Schattenakne vermeiden)
//     // Das Gitter hat keine Normalen. Wir nehmen eine feste Normalen (0, 1, 0) an,
//     // da es auf der XZ-Ebene liegt, und simulieren den Normalen-Bias.
//     // lightDir muss in diesem Shader *nicht* berechnet werden, wir verwenden einen konstanten Bias.
//     
//     // float bias = max(0.05 * (1.0 - dot(norm, lightDir)), 0.005); // Original
//     // Da die Normale (0,1,0) ist und lightDir variiert, vereinfachen wir:
//     float bias = 0.005; // Fester kleiner Bias
//     
//     // 6. Schatten-Check (Wir verwenden PCF für weichere Ränder - wie in object.frag)
//     float shadow = 0.0;
//     vec2 texelSize = 1.0 / textureSize(uShadowMap, 0);
//     for (int x = -1; x <= 1; ++x)
//     {
//         for (int y = -1; y <= 1; ++y)
//         {
//             float pcfDepth = texture(uShadowMap, projCoords.xy + vec2(x, y) * texelSize).r; 
//             shadow += currentDepth - bias > pcfDepth ? 1.0 : 0.0;        
//         }    
//     }
//     shadow /= 9.0;
//     
//     // Wir invertieren den Wert, um den Schatten-Faktor (0=Schatten, 1=Kein Schatten) zu erhalten
//     return 1.0 - shadow; 
// }


// void main()
// {
//     // Der Schattenfaktor ist 1.0 (voll beleuchtet) oder < 1.0 (im Schatten).
//     float shadowFactor = calculateShadow();
//     
//     // Wir multiplizieren die Gitterfarbe mit einem Schatten-Faktor,
//     // z.B. 0.3 (volle Dunkelheit) bis 1.0 (volle Beleuchtung).
//     // Wenn shadowFactor = 0 (voller Schatten), wollen wir 0.3.
//     // Wenn shadowFactor = 1 (kein Schatten), wollen wir 1.0.
//     // factor = 0.7 * shadowFactor + 0.3
//     float finalDarkeningFactor = 0.7 * shadowFactor + 0.3; 

//     // Beleuchtung (Ambient ist nicht enthalten, da das Gitter normalerweise nicht beleuchtet wird)
//     // Wir behalten die Gitterfarbe und dunkeln sie ab.
//     FragColor = vec4(vColor * finalDarkeningFactor, 1.0);
// }

#version 410 core

// Inputs vom Vertex Shader
in vec4 vLightSpacePos;
in vec3 vNormal;

// Output
out vec4 FragColor;

// Uniforms
uniform sampler2D uShadowMap;
uniform vec3 uLightPos; 
uniform vec3 uGroundColor; // Die Grundfarbe des Bodens (von C++ übergeben)

// --- Shadow-Kalkulationsfunktion (PCF mit Normal-basiertem Bias) ---
float calculateShadow(vec3 norm, vec3 lightDir)
{
    // 1. Position des Fragments im Light Space auf [0, 1] normalisieren
    vec3 projCoords = vLightSpacePos.xyz / vLightSpacePos.w;
    projCoords = projCoords * 0.5 + 0.5;
    
    if(projCoords.z > 1.0) return 0.0; // Außerhalb des Frustums

    // 2. Aktuelle Fragmenttiefe
    float currentDepth = projCoords.z;
    
    // 3. Normalen-basierter Biasing (wie in object.frag)
    float bias = max(0.05 * (1.0 - dot(norm, lightDir)), 0.005);
    
    // 4. PCF (Percentage Closer Filtering)
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
    
    // Gibt den Beleuchtungsfaktor zurück (1.0 = kein Schatten, 0.0 = voller Schatten)
    return 1.0 - shadow; 
}


void main()
{
    vec3 norm = normalize(vNormal); 
    // Achtung: Das Gitter hat keine Model-Matrix. vLightSpacePos.xyz ist bereits die Weltposition.
    vec3 lightDir = normalize(uLightPos - vLightSpacePos.xyz); 

    float shadowFactor = calculateShadow(norm, lightDir);
    
    // Finaler Abdunkelungsfaktor (von 0.3 im Schatten bis 1.0 beleuchtet)
    float finalDarkeningFactor = 0.7 * shadowFactor + 0.3; 

    // Finale Farbe: Bodenfarbe multipliziert mit dem Abdunkelungsfaktor
    FragColor = vec4(uGroundColor * finalDarkeningFactor, 1.0);
}
