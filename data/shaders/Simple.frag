#version 400

in vec3 bodyCent;
in vec3 vCol;

out vec4 col;

void main()
{
   col = vec4(vCol, 1.0f);
}