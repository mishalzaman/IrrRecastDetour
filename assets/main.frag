#version 120

varying vec3 fragNormal;
varying vec3 fragPos;
varying vec2 fragTexCoord;

uniform vec3 mLightPos;
uniform vec3 mLightColor;
uniform vec3 mBaseColor;
uniform sampler2D mTexture;

void main()
{
    vec3 N = normalize(fragNormal);
    vec3 L = normalize(mLightPos - fragPos);
    vec3 V = normalize(-fragPos); // View direction (camera is at origin in view space)
    
    // Wrapped diffuse for softer shadows (Half-Lambert technique)
    float NdotL = dot(N, L);
    float wrappedDiff = (NdotL + 1.0) * 0.5; // Wrap lighting into shadow areas
    wrappedDiff = wrappedDiff * wrappedDiff; // Square for smoother falloff
    
    // Rim lighting for modern look
    float rimPower = 3.0;
    float rimIntensity = 0.4;
    float rim = 1.0 - max(dot(V, N), 0.0);
    rim = pow(rim, rimPower) * rimIntensity;
    
    // Fresnel effect for subtle edge highlights
    float fresnel = pow(1.0 - max(dot(V, N), 0.0), 4.0) * 0.2;
    
    // Sample texture
    vec4 texColor = texture2D(mTexture, fragTexCoord);
    
    // Increased ambient for softer overall look
    vec3 ambient = 0.5 * mBaseColor;
    
    // Softer diffuse with less contrast
    vec3 diffuse = wrappedDiff * mLightColor * mBaseColor * 0.7;
    
    // Combine all lighting components
    vec3 rimLight = rim * mLightColor;
    vec3 fresnelLight = fresnel * vec3(1.0);
    
    vec3 finalColor = (ambient + diffuse + rimLight + fresnelLight) * texColor.rgb;
    
    // Simple tone mapping for better color distribution
    finalColor = finalColor / (finalColor + vec3(1.0));
    
    // Slight gamma correction for more natural look
    finalColor = pow(finalColor, vec3(1.0 / 1.8));
    
    gl_FragColor = vec4(finalColor, 1.0);
}