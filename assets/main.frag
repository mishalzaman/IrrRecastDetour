#version 460 core

in vec3 fragNormal;
in vec3 fragPos;
in vec2 fragTexCoord; // [NEW] Receive from vertex shader

uniform vec3 mLightPos;
uniform vec3 mLightColor;
uniform vec3 mBaseColor;
uniform sampler2D mTexture; // [NEW] The texture unit

out vec4 FragColor;

void main()
{
    vec3 N = normalize(fragNormal);
    vec3 L = normalize(mLightPos - fragPos);
    
    float diff = max(dot(N, L), 0.0);
    
    // [NEW] Sample the texture color using the coordinates
    vec4 texColor = texture(mTexture, fragTexCoord);

    // Lighting calculation
    vec3 ambient = 0.3 * mBaseColor;
    vec3 diffuse = diff * mLightColor * mBaseColor;
    
    // [NEW] Combine lighting with the texture
    // We multiply (ambient + diffuse) by texColor.rgb
    vec3 result = (ambient + diffuse) * texColor.rgb;
    
    FragColor = vec4(result, 1.0);
}