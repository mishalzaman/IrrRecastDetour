#version 120

uniform mat4 mWorldViewProj;
uniform mat4 mWorld;

varying vec3 fragNormal;
varying vec3 fragPos;
varying vec2 fragTexCoord;

void main()
{
    // gl_Vertex, gl_Normal, and gl_MultiTexCoord0 come from Irrlicht automatically
    vec4 worldPos = mWorld * gl_Vertex;
    fragPos = worldPos.xyz;
    fragNormal = mat3(mWorld) * gl_Normal;
    fragTexCoord = gl_MultiTexCoord0.xy;
    
    gl_Position = mWorldViewProj * gl_Vertex;
}