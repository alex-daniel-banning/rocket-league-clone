#version 330 core
out vec4 FragColor;

in vec2 TexCoords;
in vec3 Normal;
in vec3 FragPos;

uniform sampler2D texture_diffuse1;
uniform bool useTexture;
uniform vec3 lightColor;
uniform vec3 lightPos;

void main()
{    
    float ambientStrength = 0.01;
    vec3 ambient = ambientStrength * lightColor;

    vec3 baseColor;
    float alpha;
    if (useTexture) {
        vec4 textureColor = (texture(texture_diffuse1, TexCoords));
        baseColor = textureColor.rgb;
        alpha = textureColor.a;
    } else {
        baseColor = vec3(0.2, 0.4, 0.8);
        alpha = 1.0;
    }


    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    vec3 result = (ambient + diffuse) * baseColor;
    FragColor = vec4(result, alpha);
}
