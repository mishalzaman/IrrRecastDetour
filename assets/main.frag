#version 120

varying vec3 fragNormal;
varying vec3 fragPos;
varying vec2 fragTexCoord;

uniform vec3 mLightPos;
uniform float mAmbientStrength;
uniform float mSpecularStrength;
uniform sampler2D mTexture;

void main()
{
    // 1. Texture Sample
    vec4 texSample = texture2D(mTexture, fragTexCoord);
    vec3 objectColor = texSample.rgb;

    // 2. Setup Vectors
    vec3 N = normalize(fragNormal);
    vec3 L = normalize(mLightPos - fragPos);
    vec3 V = normalize(-fragPos);

    // 3. Light Properties
    vec3 lightColor = vec3(1.0, 1.0, 1.0);

    // 4. Ambient
    // Slight increase suggested for softer setups to reduce contrast
    float ambientStrength = mAmbientStrength; 
    vec3 ambient = ambientStrength * lightColor * objectColor;

    // 5. Soft Diffuse (Half-Lambert)
    // Instead of max(dot(N, L), 0.0), we remap the range [-1, 1] to [0, 1]
    float NdotL = dot(N, L);
    float halfLambert = NdotL * 0.5 + 0.5;
    
    // Squaring the result creates a pleasing falloff, 
    // preventing the object from looking too flat.
    float diff = halfLambert * halfLambert; 
    vec3 diffuse = diff * lightColor * objectColor;

    // 6. Soft Specular (Blinn-Phong)
    // Uses the Halfway vector (H) between Light and View
    vec3 H = normalize(L + V);
    float NdotH = max(dot(N, H), 0.0);
    
    float specularStrength = mSpecularStrength;
    // Lower shininess (e.g., 16.0 -> 8.0) makes the highlight larger and softer
    // Higher shininess (e.g., 32.0+) makes it smaller and sharper
    float spec = pow(NdotH, 16.0); 
    vec3 specular = specularStrength * spec * lightColor;

    // 7. Combine
    vec3 color = ambient + diffuse + specular;

    // Optional: Gamma Correction (Makes lighting falloff much smoother)
    // color = pow(color, vec3(1.0 / 2.2));

    gl_FragColor = vec4(color, 1.0);
}