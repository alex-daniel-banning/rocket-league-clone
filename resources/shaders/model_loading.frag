#version 330 core
out vec4 FragColor;

in VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
    vec4 FragPosLightSpace;
} fs_in;

uniform sampler2D texture_diffuse1;
uniform sampler2D shadowMap;
uniform bool useTexture;
uniform vec3 diffuseColor;

uniform vec3 lightColor;
uniform vec3 lightPos;
uniform vec3 viewPos;

// uniform float ambientStrength = 0.01;

float ShadowCalculations(vec4 fragPosLightSpace, vec3 normal, vec3 lightDir)
{
    // perform perspective divide
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    projCoords = projCoords * 0.5 + 0.5;

    float closestDepth = texture(shadowMap, projCoords.xy).r;
    float currentDepth = projCoords.z;
    float bias = max(0.05 * (1.0 - dot(normal, lightDir)), 0.005);
    //float shadow = currentDepth - bias > closestDepth ? 1.0 : 0.0;
    float shadow = 0.0;
    vec2 texelSize = 1.0 / textureSize(shadowMap, 0);
    for (int x = -1; x <= 1; ++x)
    {
        for (int y = -1; y <= 1; ++y)
        {
            float pcfDepth = texture(shadowMap, projCoords.xy + vec2(x, y) * texelSize).r;
            shadow += currentDepth - bias > pcfDepth ? 1.0 : 0.0;
        }
    }
    shadow /= 9;
    return shadow;
}

void main()
{    
    // vec3 ambient = ambientStrength * lightColor;
    vec3 ambient = 0.5 * lightColor;
    vec3 normal = normalize(fs_in.Normal);

    vec3 baseColor;
    float alpha;
    if (useTexture) {
        vec4 textureColor = (texture(texture_diffuse1, fs_in.TexCoords));
        baseColor = textureColor.rgb;
        alpha = textureColor.a;
    } else {
        baseColor = diffuseColor;
        alpha = 1.0;
    }


    vec3 lightDir = normalize(lightPos - fs_in.FragPos);
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    float shadow = ShadowCalculations(fs_in.FragPosLightSpace, normal, lightDir);

    vec3 result = (ambient + (1.0 - shadow) * diffuse) * baseColor;
    FragColor = vec4(result, alpha);
}
