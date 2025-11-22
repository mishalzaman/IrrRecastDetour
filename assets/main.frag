#version 120

varying vec3 fragNormal;
varying vec3 fragPos;
varying vec2 fragTexCoord;

uniform vec3 mLightPos;
uniform sampler2D mTexture;

// Simple directional Ambient Occlusion
float approximateAO(vec3 normal) {
    float downFacing = dot(normal, vec3(0.0, -1.0, 0.0));
    downFacing = max(downFacing, 0.0);
    return 1.0 - (downFacing * 0.4);
}

void main()
{
    // 1. Texture Sample
    vec4 texSample = texture2D(mTexture, fragTexCoord);
    vec3 objectColor = texSample.rgb;

    // 2. Setup Vectors
    vec3 N = normalize(fragNormal);
    vec3 L = normalize(mLightPos - fragPos);
    vec3 V = normalize(-fragPos);
    
    // 3. Light Configuration
    vec3 lightColor = vec3(1.0, 1.0, 1.0);
    float lightPower = 60.0; // Very high intensity

    // 4. Modified Linear Attenuation (Very slow falloff)
    float distance = length(mLightPos - fragPos);
    // The '0.1' coefficient makes the light reach much further before dimming
    float attenuation = 1.0 / (1.0 + distance * 0.1); 
    vec3 radiance = lightColor * attenuation * lightPower;

    // 5. Diffuse
    float diff = max(dot(N, L), 0.0);
    vec3 diffuse = diff * radiance * objectColor;

    // 6. Specular
    vec3 H = normalize(L + V);
    float spec = pow(max(dot(N, H), 0.0), 128.0);
    vec3 specular = spec * radiance; 

    // 7. Ambient (High Visibility)
    float ao = approximateAO(N);
    // Base brightness of 0.6 ensures nothing is ever fully dark
    vec3 ambient = vec3(0.6) * objectColor * ao; 

    // 8. Combine
    vec3 lighting = ambient + diffuse + specular;

    // 9. Tone Mapping & Gamma
    vec3 color = lighting / (lighting + vec3(1.0));
    color = pow(color, vec3(1.0 / 2.2));
    
    gl_FragColor = vec4(color, 1.0);
}