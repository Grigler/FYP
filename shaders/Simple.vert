#version 400

struct Body
{
  vec3 pos;
  float x,y,z,w;
  
  //Mat3 invInertiaTensor;
  float x1,x2,x3;
  float y1,y2,y3;
  float z1,z2,z3;
  
  float mass;
  
  vec3 linearVel;
  vec3 angularVel;
  
  float linearDrag;
  float angularDrag;
  
  //AABB bv;
  vec3 bvOrigMin;
  vec3 bvOrigMax;
  vec3 bvMin;
  vec3 bvMax;
  //int bodyID;
  
  //TODO - Complex collider type instead of sphere
  float sphereRadius;
  
  vec3 accumForce;
  vec3 accumTorque;
};

layout (location = 0) in vec3 b; //Stride by 1 per circle
layout (location = 1) in vec3 pos;

uniform mat4 VP;

out vec3 bodyCent;

void main()
{
  mat4 M = mat4(vec4(0.0f),vec4(0.0f),vec4(0.0f),vec4(0.0f));
  M[0][0] = 1.0f;
  M[1][1] = 1.0f;
  M[2][2] = 1.0f;
  M[3][3] = 1.0f;
  
  M[3][0] = b.x;
  M[3][1] = b.y;
  M[3][2] = b.z;
  
  bodyCent = b;
  
  mat4 MVP = VP * M;
  
  gl_Position = MVP * vec4(pos, 1.0f);
}