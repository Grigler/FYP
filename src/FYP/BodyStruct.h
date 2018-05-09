#ifndef __FYP_BODY_STRUCT__
#define __FYP_BODY_STRUCT__


//MUST BE KEPT IN SYNC WITH KERNEL INTERPRETATION
struct Mat3
{
  cl_float3 row[3];
};
static Mat3 GetInvInertiaForSphere(float _rad, float _invMass)
{
  Mat3 ret;
  float diagVal = _invMass == 0.0f ? 0.0f : (1.0f/_invMass)*0.4f * _rad*_rad;
  ret.row[0].x = diagVal;
  ret.row[1].y = diagVal;
  ret.row[2].z = diagVal;

  ret.row[0].y = 0.0f;
  ret.row[0].z = 0.0f;
  ret.row[1].x = 0.0f;
  ret.row[1].z = 0.0f;
  ret.row[2].x = 0.0f;
  ret.row[2].y = 0.0f;

  return ret;
}
static Mat3 GetInvInertiaForBox(float _hx, float _hy, float _hz, float _invMass)
{
  Mat3 ret;
  _hx *= 2.0f; _hy *= 2.0f; _hz *= 2.0f;
  float hSqr = _hy * _hy;
  float wSqr = _hx * _hx;
  float dSqr = _hz * _hz;
  float sMass = _invMass == 0.0f ? 0.0f : (1.0f/_invMass)*(1.0f / 12.0f);

  ret.row[0].x = sMass*(hSqr+dSqr);
  ret.row[1].y = sMass*(wSqr+dSqr);
  ret.row[2].z = sMass*(wSqr+hSqr);

  ret.row[0].y = 0.0f;
  ret.row[0].z = 0.0f;
  ret.row[1].x = 0.0f;
  ret.row[1].z = 0.0f;
  ret.row[2].x = 0.0f;
  ret.row[2].y = 0.0f;

  return ret;
}
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
  Mat3 worldInvInertiaTensor;

  cl_float invMass;

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
  cl_bool isSphere;

  cl_float sphereRadius;
  
  Quat obbOrien;
  cl_float3 obbHalfExtents;

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
    pos.x = 0.0f;// rand() % 20 - 10;
    pos.y = (amnt+10.0f) * 2.5f;// +rand() % 20 - 10;
    pos.z = -5.0f;// rand() % 20 - 10;

    Quat emptyQ;
    emptyQ.val.x = 0.0f;
    emptyQ.val.y = 0.0f;
    emptyQ.val.z = 0.0f;
    emptyQ.val.w = 1.0f;

    orien = emptyQ;

    invMass = 1.0f / (rand() % 50 + 1);

    linearVel = empty;
    angularVel = empty;

    linearDrag = 0.4f;
    angularDrag = 0.0f;

    bvLocalMin.x = -1.0f;
    bvLocalMin.y = -1.0f;
    bvLocalMin.z = -1.0f;
    bvLocalMax.x = 1.0f;
    bvLocalMax.y = 1.0f;
    bvLocalMax.z = 1.0f;

    isSphere = true;
    sphereRadius = 1.0f;
    obbOrien = emptyQ;
    obbHalfExtents.x = 0;
    obbHalfExtents.y = 0;
    obbHalfExtents.z = 0;

    accumulatedForce = empty;
    accumulatedTorque = empty;

    invInertiaTensor = GetInvInertiaForSphere(sphereRadius, invMass);
    worldInvInertiaTensor = invInertiaTensor;

    static float offset = 0.0f;
    //if (i == 10.0f)
    //{
    //  i = 2.5f;
    //  offset -= 2.5f;
    //}


    //staging collision
    if (i >= 5.0f)
    {
      //pos.x = 0.0f;
      pos.x = (rand() % 50 - 25) / 10.0f;
      pos.y = 1.0f * i;
      pos.z = 0;
      pos.z = (rand() % 50 - 25) / 10.0f;
      //linearVel.x = 8.0f;
      //linearVel.y = 2.0f;
      //angularVel.z = (rand()%40-20)/10.0f;
      isSphere = true;
      invMass = 1.0f / (rand()%90+10);
      invInertiaTensor =
        GetInvInertiaForSphere(1.0f, invMass);
    }
    if (i == 2.5f)
    {
      pos.x = 0.0f;
      pos.y = -7.5f;
      pos.z = 0.0f;
      linearVel.x = 0.0f;
      //sphereRadius = 1.5f;
      //angularVel.x = 100.0f;
      //angularVel.y = 0.001f;
      //angularVel.z = 0.5f;
      obbHalfExtents.x = 250.0f;
      obbHalfExtents.y = 7.5f;
      obbHalfExtents.z = 250.0f;
      bvLocalMin.x = -250.0f;
      bvLocalMin.y = -7.5f;
      bvLocalMin.z = -250.0f;
      bvLocalMax.x = 250.0f;
      bvLocalMax.y = 7.5f;
      bvLocalMax.z = 250.0f;
      isSphere = false;
      invMass = 0.0f;
      invInertiaTensor =
        GetInvInertiaForBox(obbHalfExtents.x, obbHalfExtents.y, obbHalfExtents.z, invMass);
    }
    if (false && i == 7.5f)
    {
      pos.x = 4.0f;
      pos.y = 4.0f;
      pos.z = 0.0f;
      linearVel.x = -8.0f;
      isSphere = true;
      angularVel.z = 2.0f;
      invMass = 1.0f / 20.0f;
      invInertiaTensor =
        GetInvInertiaForSphere(1.0f, invMass);
    }

    
    bvMin.x = pos.x + bvLocalMin.x;
    bvMin.y = pos.y + bvLocalMin.y;
    bvMin.z = pos.z + bvLocalMin.z;
    bvMax.x = pos.x + bvLocalMax.x;
    bvMax.y = pos.y + bvLocalMax.y;
    bvMax.z = pos.z + bvLocalMax.z;

    
  }

  //TODO - API functions to update bodies and such
};

#endif