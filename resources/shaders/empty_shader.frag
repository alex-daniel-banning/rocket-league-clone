//#version 330 core
//
//void main()
//{
//    // nothing
//}
#version 330 core
in vec4 LightSpacePos;
out vec4 FragColor;

void main()
{
    // map clip-space xy from [-1,1] to [0,1]
    vec3 color = LightSpacePos.xyz / LightSpacePos.w; // NDC
    color = color * 0.5 + 0.5;
    FragColor = vec4(color, 1.0);
}

