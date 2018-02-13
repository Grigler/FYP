#ifndef __FYP_BODY_STRUCT__
#define __FYP_BODY_STRUCT__

#include <glm/vec3.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/mat3x3.hpp>

//MUST BE KEPT IN SYNC WITH KERNEL INTERPRETATION

struct Body
{
  cl_float3 pos;

  //glm::quat rot;
  //quat
  cl_float x, y, z, w;

  //glm::mat3 invInertiaTensor;
  //invInertiaTensor
  cl_float x1, x2, x3;
  cl_float y1, y2, y3;
  cl_float z1, z2, z3;

  cl_float mass;

  cl_float3 linearVel;
  cl_float3 angularVel;

  cl_float linearDrag;
  cl_float angularDrag;
  
  //AABB region
  cl_float3 bvLocalMin;
  cl_float3 bvLocalMax;
  cl_float3 bvMin;
  cl_float3 bvMax;

  //TODO - complex collider type
  cl_float sphereRadius;

  cl_float3 accumulatedForce;
  cl_float3 accumulatedTorque;
  //43 - Floats as of here - 172 bytes per body (without padding)

  Body()
  {
    static cl_float i = 0;
    i += 2.5f;

    cl_float3 empty;
    empty.x = 0.0f;
    empty.y = 0.0f;
    empty.z = 0.0f;

    cl_float p = (float)(rand() % 2000 - 1000) / 10.0f;
    pos.x = 0;
    pos.y = i;
    pos.z = 1000.0f;


    x = 0.0f; y = 0.0f; z = 0.0f; w = 1.0f;

    x1 = 1.0f; x2 = 0.0f; x3 = 0.0f;
    y1 = 0.0f; y2 = 1.0f; y3 = 0.0f;
    z1 = 0.0f; z2 = 0.0f; z3 = 1.0f;

    mass = 2.5f;

    linearVel = empty;
    angularVel = empty;

    linearDrag = 1.0f;
    angularDrag = 1.0f;

    bvLocalMin.x = -1.0f;
    bvLocalMin.y = -1.0f;
    bvLocalMin.z = -1.0f;
    bvLocalMax.x = 1.0f;
    bvLocalMax.y = 1.0f;
    bvLocalMax.z = 1.0f;
    bvMin.x = pos.x -1.0f;
    bvMin.y = pos.y -1.0f;
    bvMin.z = pos.z -1.0f;
    bvMax.x = pos.x +1.0f;
    bvMax.y = pos.y +1.0f;
    bvMax.z = pos.z +1.0f;

    sphereRadius = 1.0f;

    accumulatedForce = empty;
    accumulatedTorque = empty;

    //staging collision
    if (i == 2.5f)
    {
      pos.x = -2.0f;
      pos.y = 0.0f;
      pos.z = 0.0f;
      linearVel.x = 4.0f;
    }
    if (i == 5.0f)
    {
      pos.x = 2.0f;
      pos.y = 0.0f;
      pos.z = 0.0f;
      linearVel.x = -4.0f;
    }
  }
  //TODO - API functions to update bodies and such
};

#endif