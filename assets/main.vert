#version 460 core

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inTexCoord; // This comes from the mesh

uniform mat4 mWorldViewProj;
uniform mat4 mWorld;

out vec3 fragNormal;
out vec3 fragPos;
out vec2 fragTexCoord; // [NEW] Output to fragment shader

void main()
{
    vec4 worldPos = mWorld * vec4(inPosition, 1.0);
    fragPos = worldPos.xyz;
    fragNormal = mat3(mWorld) * inNormal;
    
    // [NEW] Pass the coordinate along
    fragTexCoord = inPosition; 
    
    gl_Position = mWorldViewProj * vec4(inPosition, 1.0);
}