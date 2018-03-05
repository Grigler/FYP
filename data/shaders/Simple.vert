#version 400

struct Body
{
  vec3 pos;
  vec4 orien;
  
  mat3 invInertiaTensor;
  
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

//Stride by 1 per sphere
layout (location = 0) in vec3 bPos;
layout (location = 1) in vec4 bQuat;

layout (location = 2) in vec3 vPos;

uniform mat4 VP;

out vec3 bodyCent;

mat4 RotFromQuat(vec4 q)
{
  float qxx = (q.x * q.x);
  float qyy = (q.y * q.y);
  float qzz = (q.z * q.z);
  float qxz = (q.x * q.z);
  float qxy = (q.x * q.y);
  float qyz = (q.y * q.z);
  float qwx = (q.w * q.x);
  float qwy = (q.w * q.y);
  float qwz = (q.w * q.z);
  
  mat4 mat;
  mat[0][0] = 1 - 2 * ( qyy + qzz );
  mat[1][0] =     2 * ( qxy - qwz );
  mat[2][0] =     2 * ( qxz + qwy );
  mat[3][0] = 0.0f;
  
  mat[0][1] =     2 * ( qxy + qwz );
  mat[1][1] = 1 - 2 * ( qxx + qzz );
  mat[2][1] =     2 * ( qyz - qwx );
  mat[3][1] = 0.0f;
  
  mat[0][2] =     2 * ( qxz - qwy );
  mat[1][2] =     2 * ( qyz + qwx );
  mat[2][2] = 1 - 2 * ( qxx + qyy );
  mat[3][2] = 0.0f;
  
  mat[3][0] = mat[3][1] = mat[3][2] = 0;
  mat[3][3] = 1;

  return mat;
}

void main()
{
  mat4 r = RotFromQuat(bQuat);
  
  mat4 p = mat4(vec4(0.0f),vec4(0.0f),vec4(0.0f),vec4(0.0f));
  p[0][0] = 1.0f;
  p[1][1] = 1.0f;
  p[2][2] = 1.0f;
  p[3][3] = 1.0f;
  
  p[3][0] = bPos.x;
  p[3][1] = bPos.y;
  p[3][2] = bPos.z;
  
  bodyCent = bPos;
  
  mat4 s = mat4(1.0f);
  
  mat4 M = p * r * s;
  
  mat4 MVP = VP * M;
  
  gl_Position = MVP * vec4(vPos, 1.0f);
}