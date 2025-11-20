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
    float diff = max(dot(N, L), 0.0);
    
    vec4 texColor = texture2D(mTexture, fragTexCoord);
    
    vec3 ambient = 0.3 * mBaseColor;
    vec3 diffuse = diff * mLightColor * mBaseColor;
    
    vec3 result = (ambient + diffuse) * texColor.rgb;
    gl_FragColor = vec4(result, 1.0);
}