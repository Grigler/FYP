#version 400

in vec3 bodyCent;

out vec4 col;

void main()
{
   col = vec4(bodyCent.x, bodyCent.y, bodyCent.z, 1.0f);
}