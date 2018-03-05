#ifndef __FYP_BODY_STRUCT__
#define __FYP_BODY_STRUCT__


//MUST BE KEPT IN SYNC WITH KERNEL INTERPRETATION
struct Mat3
{
  cl_float3 row[3];
};
struct Quat
{
  cl_float4 val;
};

struct Body
{
  cl_float3 pos;

  //glm::quat rot;
  //quat
  //cl_float x, y, z, w;
  Quat orien;

  //invInertiaTensor
  //cl_float x1, x2, x3;
  //cl_float y1, y2, y3;
  //cl_float z1, z2, z3;
  Mat3 invInertiaTensor;

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
	static int amnt = 0;
	amnt++;

    cl_float3 empty;
    empty.x = 0.0f;
    empty.y = 0.0f;
    empty.z = 0.0f;

    cl_float p = (float)(rand() % 2000 - 1000) / 10.0f;
    pos.x = rand()%10 - 5;
	  pos.y = amnt * 2.0f;
    pos.z = 0.0f;

    orien.val.x = 0.0f;
    orien.val.y = 0.0f;
    orien.val.z = 0.0f;
    orien.val.w = 1.0f;

    mass = rand()%50 + 1;

    linearVel = empty;
    angularVel = empty;

    linearDrag = 0.2f;
    angularDrag = 10.0f;

    bvLocalMin.x = -1.0f;
    bvLocalMin.y = -1.0f;
    bvLocalMin.z = -1.0f;
    bvLocalMax.x = 1.0f;
    bvLocalMax.y = 1.0f;
    bvLocalMax.z = 1.0f;

    sphereRadius = 1.0f;

    accumulatedForce = empty;
    accumulatedTorque = empty;

    //staging collision
    if (i == 2.5f)
    {
      pos.x = -4.0f;
      pos.y = 2.0f;
      pos.z = 0.0f;
      linearVel.x = 8.0f;
      mass = 20.0f;
    }
    if (i == 5.0f)
    {
      pos.x = 0.0f;
      pos.y = 2.0f;
      pos.z = 0.0f;
      linearVel.x = 0.0f;
      //angularVel.x = 24.0f;
      //angularVel.y = 24.0f;
      angularVel.z = 2.0f;
      mass = 20.0f;
    }
    if (i == 7.5f)
    {
      pos.x = 4.0f;
      pos.y = 2.0f;
      pos.z = 0.0f;
      linearVel.x = -8.0f;
      mass = 20.0f;
    }

    bvMin.x = pos.x - 1.0f;
    bvMin.y = pos.y - 1.0f;
    bvMin.z = pos.z - 1.0f;
    bvMax.x = pos.x + 1.0f;
    bvMax.y = pos.y + 1.0f;
    bvMax.z = pos.z + 1.0f;
  }
  //TODO - API functions to update bodies and such
};

#endif