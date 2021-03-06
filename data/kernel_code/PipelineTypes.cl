#define MAX_BODIES 4096

//Quat and Mat3 utility functions interpreted from 
//Bullet3 Source and partially from GLM source
typedef struct
{
  float4 val;
} Quat;
Quat quatNorm(Quat q)
{
  Quat ret;
  ret.val = fast_normalize(q.val);
  return ret;
}
Quat quatMult(Quat l, Quat r)
{
 Quat ret;
 ret.val = cross(l.val, r.val);
 ret.val += l.val.w*r.val + r.val.w*l.val;
 ret.val.w = l.val.w*r.val.w - dot(l.val, r.val);
 return ret;
}
float3 quatMultV3(float3 v, Quat q)
{
  float3 u = (float3)(q.val.x, q.val.y, q.val.z);
  float s = q.val.w;
 
  float3 ret = 2.0f * dot(u, v) * u
          + (s*s - dot(u, u)) * v
          + 2.0f * s * cross(u, v);
          
  return ret;
}
float4 quatMultV4(float3 v, Quat q)
{
  float3 u = (float3)(q.val.x, q.val.y, q.val.z);
  float s = q.val.w;
 
  float3 r = 2.0f * dot(u, v) * u
          + (s*s - dot(u, u)) * v
          + 2.0f * s * cross(u, v);
  float4 ret = (float4)(r.x,r.y,r.z,1.0f);
  
  return ret;
}

typedef struct
{
  float3 row[3];
} Mat3;

Mat3 quatToMat(Quat q)
{
  float qxx = (q.val.x * q.val.x);
  float qyy = (q.val.y * q.val.y);
  float qzz = (q.val.z * q.val.z);
  float qxz = (q.val.x * q.val.z);
  float qxy = (q.val.x * q.val.y);
  float qyz = (q.val.y * q.val.z);
  float qwx = (q.val.w * q.val.x);
  float qwy = (q.val.w * q.val.y);
  float qwz = (q.val.w * q.val.z);
  
  Mat3 ret;
  ret.row[0].x = 1 - 2 * ( qyy + qzz );
  ret.row[1].x =     2 * ( qxy - qwz );
  ret.row[2].x =     2 * ( qxz + qwy );
  ret.row[0].y =     2 * ( qxy + qwz );
  ret.row[1].y  = 1 - 2 * ( qxx + qzz );
  ret.row[2].y  =     2 * ( qyz - qwx );
  ret.row[0].z =     2 * ( qxz - qwy );
  ret.row[1].z =     2 * ( qyz + qwx );
  ret.row[2].z = 1 - 2 * ( qxx + qyy );
  
  return ret;
}

//interpreted from bullet3 source code
float3 mtMul1(Mat3 m, float3 v)
{
  float3 ret;
  
  ret.x = dot(m.row[0], v);
  ret.y = dot(m.row[1], v);
  ret.z = dot(m.row[2], v);
  
  return ret;
}
float3 mtMul3(float3 v, Mat3 m)
{
  float3 cX = (float3)(m.row[0].x, m.row[1].x, m.row[2].x);
  float3 cY = (float3)(m.row[0].y, m.row[1].y, m.row[2].y);
  float3 cZ = (float3)(m.row[0].z, m.row[1].z, m.row[2].z);
  
  float3 ret;
  
  ret.x = dot(v, cX);
  ret.y = dot(v, cY);
  ret.z = dot(v, cZ);  
  
  return ret;
}
Mat3 mtMulM(Mat3 l, Mat3 r)
{
  float3 rCol0;
  rCol0.x = r.row[0].x; rCol0.y = r.row[1].x; rCol0.z = r.row[2].x;
  float3 rCol1;
  rCol1.x = r.row[0].y; rCol1.y = r.row[1].y; rCol1.z = r.row[2].y;
  float3 rCol2;
  rCol2.x = r.row[0].z; rCol2.y = r.row[1].z; rCol2.z = r.row[2].z;
  
  Mat3 ret;
  ret.row[0].x = dot(l.row[0], rCol0);
  ret.row[0].y = dot(l.row[0], rCol1);
  ret.row[0].z = dot(l.row[0], rCol2);
  
  ret.row[1].x = dot(l.row[1], rCol0);
  ret.row[1].y = dot(l.row[1], rCol1);
  ret.row[1].z = dot(l.row[1], rCol1);
  
  ret.row[2].x = dot(l.row[2], rCol0);
  ret.row[2].y = dot(l.row[2], rCol1);
  ret.row[2].z = dot(l.row[2], rCol2);
  
  return ret;
}
Mat3 mtTranspose(Mat3 m)
{
  Mat3 ret;
  ret.row[0].y = m.row[1].x;
  ret.row[0].z = m.row[2].x;
  
  ret.row[1].x = m.row[0].y;
  ret.row[1].z = m.row[2].y;
  
  ret.row[2].x = m.row[0].z;
  ret.row[2].y = m.row[1].z;
  
  ret.row[0].x = m.row[0].x;
  ret.row[1].y = m.row[1].y;
  ret.row[2].z = m.row[2].z;
  
  return ret;  
}
void mtPrint(Mat3 m)
{
  printf("Matrix\n");
  for(int i = 0; i < 3; i++)
  {
    printf("%f\t%f\t%f\n", m.row[i].x, m.row[i].y, m.row[i].z);
  }
}

typedef struct 
{
  float3 pos;
  Quat orien;
  //float x,y,z,w;
  
  Mat3 invInertiaTensor;
  Mat3 worldInvInertiaTensor;
  //float x1,x2,x3;
  //float y1,y2,y3;
  //float z1,z2,z3;
  
  float invMass;
  
  float3 linearVel;
  float3 angularVel;
  
  float linearDrag;
  float angularDrag;
  
  //AABB bv;
  float3 bvOrigMin;
  float3 bvOrigMax;
  float3 bvMin;
  float3 bvMax;
  //int bodyID;
  
  //TODO - Complex collider type instead of sphere
  bool isSphere;
  
  float sphereRadius;
  
  Quat obbOrien;
  float3 obbHalfExtents;
  
  float3 accumForce;
  float3 accumTorque;
  
} Body;

void DebugPrintF3(float3 v)
{
  printf("%f, %f, %f\n", v.x, v.y, v.z);
}

void UpdateBV(__global Body *b, int gid)
{
  float3 rOrigMin = quatMultV3(b->bvOrigMin, b->orien);
  rOrigMin += b->pos;
  
  float3 rOrigMax = quatMultV3(b->bvOrigMax, b->orien);
  rOrigMax += b->pos;

  float3 newMin = min(rOrigMin, rOrigMax);
  float3 newMax = max(rOrigMin, rOrigMax);
  
  if(true || !(newMax.x <= -100.0f))
  {
    //printf("%i newMin %f, %f, %f\n", gid, newMin.x, newMin.y, newMin.z);
    //printf("%i newMax %f, %f, %f\n", gid, newMax.x, newMax.y, newMax.z);
    //printf("\n");
  }
  
  b->bvMin = newMin;
  b->bvMax = newMax;
}

void UpdateInvInertiaTensor(__global Body *b)
{
  Mat3 r = quatToMat(b->orien);
  Mat3 rT = mtTranspose(r);
  
  b->worldInvInertiaTensor = mtMulM(r, mtMulM(b->invInertiaTensor, rT));
}

__kernel void Integrate(__global Body *bodies, float dt)
{
  int gid = get_global_id(0);
  
  if(bodies[gid].invMass == 0.0f)
  {
    return;
  }
  
  //Linear
  //float mass = max(bodies[gid].mass, 0.01f); <-- Maybe still clamp?
  float invMass = bodies[gid].invMass;
  float3 linearAccel = bodies[gid].accumForce * invMass;
  //Gravity
  linearAccel += (float3)(0.0f, -9.81f, 0.0f);
  bodies[gid].linearVel += linearAccel * (dt);
  
  //Angular
  bodies[gid].angularVel += 
    mtMul1(bodies[gid].worldInvInertiaTensor, bodies[gid].accumTorque) * dt;
  
  //Simplified drag applications - estimation, not strictly accurate
  bodies[gid].linearVel *= (1.0f - dt*bodies[gid].linearDrag);
  bodies[gid].angularVel *= (1.0f - dt*bodies[gid].angularDrag);
  
  //Adjust state
  bodies[gid].pos += bodies[gid].linearVel * (dt);
  
  Quat orien = bodies[gid].orien;
  //Ian millington
  bodies[gid].orien.val += quatMultV4(bodies[gid].angularVel, orien) * dt*0.5f;
  //Normalising
  bodies[gid].orien = quatNorm(bodies[gid].orien);
  
  UpdateInvInertiaTensor(&bodies[gid]);
  
  //Resetting accumulators
  bodies[gid].accumForce = (float3)(0.0f);
  bodies[gid].accumTorque = (float3)(0.0f);
  
  UpdateBV(&bodies[gid], gid);
}

typedef struct
{
  int leftIndx;
  int rightIndx;
} IDPair;



typedef struct
{
  float3 origMin;
  float3 origMax;
  
  float3 min;
  float3 max;
  
  int bodyID;
} AABB;

AABB GetBVFrom(Body b)
{
  AABB ret;
  ret.origMin = b.bvOrigMin;
  ret.origMax = b.bvOrigMax;
  ret.min = b.bvMin;
  ret.max = b.bvMax;
  return ret;
}

__kernel void BroadPhase(__global Body *bodies, int numBodies, __global IDPair *pairs, volatile __global int *lastPairIndx, volatile __global int *pairsFound)
{
  int gid = get_global_id(0);
  
  AABB thisBV = GetBVFrom(bodies[gid]);

  int numFound = 0;
  
  for(int i = gid + 1; i < numBodies; i++)
  {
    AABB againstBV = GetBVFrom(bodies[i]);
  
    bool xResult = thisBV.max.x >= againstBV.min.x || thisBV.min.x <= againstBV.max.x;
    bool yResult = thisBV.max.y >= againstBV.min.y || thisBV.min.y <= againstBV.max.y;
    bool zResult = thisBV.max.z >= againstBV.min.z || thisBV.min.z <= againstBV.max.z;
    
    bool testResult = xResult & yResult & zResult;
    
    if(testResult)
    {
      IDPair newPair;
      newPair.leftIndx = gid;
      newPair.rightIndx = i;
      
      int indx = atomic_inc(lastPairIndx);
      pairs[indx] = newPair;
      
      numFound++;
    }
  }

  //Adding number found to total - for easy workload assignment for narrowphase
  atomic_add(pairsFound, numFound);
}

typedef struct
{
  float appliedImpulse;
  
  //float3 relativeVel;
  float3 normal;
  float3 worldPos;
  float depth;
  
  //1 / (J * M^-1 * J^T)l+r
  float jacDiagABInv;
  
  //Keeping it in the constraint so it could be changed
  //for more generic constraints later
  //float lowerLimit; //0 for contact
  //float upperLimit;  //< infinite for contact
  
  //indx values for bodies in bodies buffer
  int leftIndx;
  
  int rightIndx;
  
} Constraint;

//Calculates the effective mass of a normal-constraint (J*M^-1*J^T)
float GetJacM(Body l, Body r, float3 contactPos, float3 contactNorm)
{
  //Bullet's
  float jmj0 = l.invMass;
  float jmj1 = 0.0f;
  if(l.invMass != 0.0f)
  {
    float3 armL = cross(contactPos - l.pos, contactNorm);
    jmj1 = dot(mtMul3(armL, l.worldInvInertiaTensor), armL);
  }
  
  float jmj2 = r.invMass;
  float jmj3 = 0.0f;
  if(r.invMass != 0.0f)
  {
    float3 armR = cross(contactPos - r.pos, -contactNorm);
    jmj3 = dot(mtMul3(armR, r.worldInvInertiaTensor), armR);
  }
  
  
  //printf("jmj1: %f\njmj3: %f\n", jmj1, jmj3);
  // -(1/ (J*M^-1*J^T))
  float ret = -(1.0f / (jmj0+jmj1+jmj2+jmj3));
  //printf("jacM: %f\n", ret);
  return ret;
}

bool SphereSphere(Body l, Body r)
{
  //TODO - Switch to faster square check
  float radSum = l.sphereRadius + r.sphereRadius;
  float dist = distance(l.pos, r.pos);
  return dist < radSum;
}
bool OBBSphere(Body o, Body s, float3 *closestPoint)
{
  float3 dir = s.pos - o.pos;
  float3 result = o.pos;
  
  float3 up = quatMultV3((float3)(0,1,0),o.obbOrien);
  //printf("Up: %f, %f, %f\n", up.x, up.y, up.z);
  float3 right = quatMultV3((float3)(1,0,0),o.obbOrien);
  //printf("Ri: %f, %f, %f\n", right.x, right.y, right.z);
  float3 forward = quatMultV3((float3)(0,0,1),o.obbOrien);
  //printf("Fo: %f, %f, %f\n", forward.x, forward.y, forward.z);
  
  float dist = dot(dir, up);
  if(dist > o.obbHalfExtents.y) dist = o.obbHalfExtents.y;
  if(dist < -o.obbHalfExtents.y) dist = -o.obbHalfExtents.y;
  //printf("Up Dist: %f\n", dist);
  result += (dist * up);
  //printf("Dir: %f, %f, %f\nDist a: %f\n  Result a: %f\n", dir.x, dir.y, dir.z, dist, result);
  
  dist = dot(dir, right);
  if(dist > o.obbHalfExtents.x) dist = o.obbHalfExtents.x;
  if(dist < -o.obbHalfExtents.x) dist = -o.obbHalfExtents.x;
  result += (dist * right);
  //printf("Dist b: %f\n  Result b: %f\n", dist, result);
  
  dist = dot(dir, forward);
  if(dist > o.obbHalfExtents.z) dist = o.obbHalfExtents.z;
  if(dist < -o.obbHalfExtents.z) dist = -o.obbHalfExtents.z;
  result += (dist * forward);
  //printf("Dist c: %f\n  Result c: %f\n", dist, result);
  
  *closestPoint = result;
  float3 testV = result - s.pos;
  bool ret = dot(testV, testV) <= s.sphereRadius*s.sphereRadius;
  if(ret)
  {
    //printf("Result: %f, %f, %f\n", result.x, result.y, result.z);
    //("testV:  %f, %f, %f\n", testV.x, testV.y, testV.z);
    //*closestPoint = result + testV;
    //printf("point:  %f, %f, %f\n\n", closestPoint->x, closestPoint->y, closestPoint->z);
  }
  return ret;
}
__kernel void NarrowPhase(__global IDPair *pairs, __global Body *bodies, __global Constraint *constraints, volatile __global int *lastConstraintIndx)
{
  int gid = get_global_id(0);
  
  Body leftBody = bodies[pairs[gid].leftIndx];
  Body rightBody = bodies[pairs[gid].rightIndx];
  
  if(leftBody.isSphere && rightBody .isSphere)
  {
    if(SphereSphere(leftBody, rightBody))
    {
      float3 midLine = leftBody.pos - rightBody.pos;
      float dist = distance(leftBody.pos, rightBody.pos);
      
      float3 contactNorm = midLine / dist;
      //printf("Norm: %f, %f, %f\n", contactNorm.x, contactNorm.y, contactNorm.z);
      float3 contactPoint = leftBody.pos + (midLine*0.5f);
      float penetrationDepth = (leftBody.sphereRadius + rightBody.sphereRadius) - dist;
     
      //Converting into constraint format
      Constraint c;
      c.appliedImpulse = 0.0f;
      c.normal = contactNorm;
      //printf("Norm: %f, %f, %f\n", contactNorm.x, contactNorm.y, contactNorm.z);
      c.worldPos = contactPoint;
      c.depth = dot(penetrationDepth, contactNorm);
      //printf("depth: %f\n", c.depth);

      c.leftIndx = pairs[gid].leftIndx;
      
      c.rightIndx = pairs[gid].rightIndx;
      
      c.jacDiagABInv = GetJacM(leftBody, rightBody, contactPoint, contactNorm);
      
      //Write into constraint buffer
      int indx = atomic_inc(lastConstraintIndx);
      constraints[indx] = c;
    }
  }
  else
  {
    Body sphere;
    int sIndx;
    Body obb;
    int oIndx;
    //OBB
    if(leftBody.isSphere)
    {
      sphere = leftBody;
      sIndx = pairs[gid].leftIndx;
      obb = rightBody;
      oIndx = pairs[gid].rightIndx;
    }
    else
    {
      sphere = rightBody;
      sIndx = pairs[gid].rightIndx;
      obb = leftBody;
      oIndx = pairs[gid].leftIndx;
    }
    
    float3 contactPoint;
    if(OBBSphere(obb, sphere, &contactPoint))
    {
      Constraint c;
      c.appliedImpulse = 0.0f;
      c.worldPos = contactPoint;
      c.normal = fast_normalize(sphere.pos - c.worldPos);
      //printf("\nNorm: %f, %f, %f\n", c.normal.x, c.normal.y, c.normal.z);
      //printf("CONTACT POINT: %f, %f, %f\n", c.worldPos.x, c.worldPos.y, c.worldPos.z);
      //printf("SPHERE POSITI: %f, %f, %f\n\n", sphere.pos.x, sphere.pos.y, sphere.pos.z);
      
      c.depth = (distance(sphere.pos, contactPoint) - sphere.sphereRadius);
      
      c.leftIndx = sIndx;
      
      c.rightIndx = oIndx;
      
      c.jacDiagABInv = GetJacM(sphere, obb, c.worldPos, c.normal);
      //Write into constraint buffer
      int indx = atomic_inc(lastConstraintIndx);
      constraints[indx] = c;
    }
  }
}

void PGSSolver(__global Body *bodies, __global Constraint *constraints, float dt)
{
   int gid = get_global_id(0);
  
  __global Constraint * __private c = &constraints[gid];
  __global Body * __private l = &bodies[constraints[gid].leftIndx];
  __global Body * __private r = &bodies[constraints[gid].rightIndx];

  
  //Store private representations of vels, impulses and positions (reduces atomic ops)
  float3 posL = l->pos, posR = r->pos;
  float3 cPosL = c->worldPos + l->pos, cPosR = c->worldPos + r->pos;
  float3 linL = l->linearVel, linR = r->linearVel;
  float3 angL = l->angularVel, angR = r->angularVel;
  float invMassL = l->invMass, invMassR = r->invMass;
  
  for(int iteration = 0; iteration < 4; iteration++)
  {
    //printf("Iteration: %i\n", iteration);
    
    //float3 rL = c->worldPos - posL;
    //float3 rR = c->worldPos - posR;
    //float3 angL = cross(rL, c->normal);
    //float3 angR = -cross(rR, c->normal);
    float relVel = dot(c->normal, linL) + -dot(c->normal, linR); //And ang
    //printf("\tRelVel: %f\n", relVel);
    
    float jacRelVel = relVel * c->jacDiagABInv;
    //printf("\tJac: %f\n", c->jacDiagABInv);
    //printf("\tJacRelVel: %f\n", jacRelVel);
    
    //b value
    float beta = 0.1f; //Roughly a percentage of depth to remove per-iteration
    float b = (beta/dt)*c->depth;// + relVel; ?
    //printf("\tb: %f\n", b);
    
    float lamda = (jacRelVel);
    
    //Clamping to constraints for 'project' gauss-seidel
    lamda = max(lamda, 0.0f);
    
    //angL
    //angR
    //Calculating impulse forces from lambda+b
    float3 linImpulseL = (lamda+b) * c->normal * invMassL;
    float3 linImpulseR = (lamda+b) * -c->normal * invMassR;
    
    float3 cL = linImpulseL;
    float3 cR = linImpulseR;
    
    linL += cL;
    linR += cR;
    //ang+=
    //ang+=
    //barrier(CLK_GLOBAL_MEM_FENCE);
  }
  
  
  //printf("\nFinalLinL: %f, %f, %f\n", linL.x, linL.y, linL.z);
  //printf("FinalLinR: %f, %f, %f\n", linR.x, linR.y, linR.z);

  l->linearVel = linL;
  r->linearVel = linR;

  //ang+=
  //ang+=
  
  //bodies[constraints[gid].leftIndx] = *l;
  //bodies[constraint[gid].rightIndx] = *r;
}

void SequentialImpulseSolver(__global Body *bodies, __global Constraint *constraints, float dt)
{
   int gid = get_global_id(0);
  
  __global Constraint * __private c = &constraints[gid];
  __global Body * __private l = &bodies[constraints[gid].leftIndx];
  __global Body * __private r = &bodies[constraints[gid].rightIndx];

  //Store private representations of vels, impulses and positions (reduces atomic ops)
  float3 posL = l->pos, posR = r->pos;
  float3 cPosL = c->worldPos + l->pos, cPosR = c->worldPos + r->pos;
  float3 linL = l->linearVel, linR = r->linearVel;
  float3 angL = l->angularVel, angR = r->angularVel;
  float invMassL = l->invMass, invMassR = r->invMass;
  //printf("lMass: %f\nrMass: %f\n\n", invMassL, invMassR);
  Mat3 invInertL = l->worldInvInertiaTensor, invInertR = r->worldInvInertiaTensor;
  
  float3 rL = c->worldPos - posL;
  float3 rR = c->worldPos - posR;
  float3 aL = -cross(rL, c->normal);
  float3 aR = cross(rR, c->normal);

  for(int iteration = 0; iteration < 4; iteration++)
  {
    barrier(CLK_GLOBAL_MEM_FENCE);
    
    //printf("\nIteration: %i\n", iteration);
    
    linL = l->linearVel;
    linR = r->linearVel;
    angL = l->angularVel;
    angR = r->angularVel;
    
    //Lin
    float relVel = dot(c->normal, linL) + dot(-c->normal, linR);
    //printf("\tRelLin: %f\n", relVel);
    //printf("\t\tLl: %f\n\t\tRl: %f\n", dot(c->normal, linL), dot(-c->normal, linR));
    //Ang
    relVel += dot(aL, angL) + dot(aR, angR);
    float i = dot(aL, angL);
    float j = dot(aR, angR);
    //printf("\t\tLa: %f\n\t\tRa: %f\n", i, j);
    //printf("\trelvel: %f\n", relVel);
    
    //b value for linear projection
    float beta = 0.8f; //Roughly a percentage of depth to remove per-iteration
    float b = (beta/dt)*c->depth;
    //float b = beta * dt * c->depth;
    //printf("\tdt: %f\n", dt);
    //printf("\tdepth: %f\n", c->depth);
    //printf("\tb: %f\n", b);

    //printf("\tjacDiagABInv: %f\n", c->jacDiagABInv);
    float lambda = (c->jacDiagABInv)*(relVel + b);
    //printf("\tlambda: %f\n", lambda);
    
    //Using lambdaAccumulator
    float preClamp = c->appliedImpulse;
    //printf("\tpreAccum: %f\n", preClamp);
    c->appliedImpulse += lambda;
    //printf("\tlambdaAccum: %f\n", c->appliedImpulse);
    c->appliedImpulse = max(c->appliedImpulse, 0.0f);
    //printf("\tlambdaAccum: %f\n", c->appliedImpulse);
    //Re-adjusting lambda to apply based on clamping
    lambda = c->appliedImpulse-preClamp;
    //printf("\tlambda: %f\n", lambda);
    
    float3 linImpulseL = lambda * c->normal * invMassL;
    float3 linImpulseR = lambda * -c->normal * invMassR;
    float3 angImpulseL = 0.0f;
    float3 angImpulseR = 0.0f;
    if(invMassL != 0.0f) angImpulseL = lambda * mtMul1(invInertL, aL);
    if(invMassR != 0.0f) angImpulseR = lambda * mtMul1(invInertR, aR);
    
    //Applying linear impulse
    //printf("\tlinVLpre: %f, %f, %f\n", l->linearVel.x,l->linearVel.y,l->linearVel.z);
    l->linearVel += linImpulseL;
    r->linearVel += linImpulseR;
    //printf("\tlinVLpost: %f, %f, %f\n", l->linearVel.x,l->linearVel.y,l->linearVel.z);
    //printf("\tlinR: %f, %f, %f\n", r->linearVel.x,r->linearVel.y,r->linearVel.z);
    //angular impulse
    l->angularVel += angImpulseL;
    r->angularVel += angImpulseR;
    
    //printf("\tlinIL: %f, %f, %f\n\tlinIR: %f, %f, %f\n",
    //  linImpulseL.x, linImpulseL.y, linImpulseL.z,
    //  linImpulseR.x, linImpulseR.y, linImpulseR.z);
    //printf("\tangL: %f, %f, %f\n\tangR: %f, %f, %f\n",
    //  angImpulseL.x, angImpulseL.y, angImpulseL.z,
    //  angImpulseR.x, angImpulseR.y, angImpulseR.z);      

    //Waiting for all other threads to finish their addition
    //to the relevant body's velocity (could be done local thread if batched)
    barrier(CLK_GLOBAL_MEM_FENCE);
  }
}

//Solver Define Flags
//#define USE_PGS 1
#define USE_SI 1

//Takes constraints information - not contacts (contacts are in constraint form)
__kernel void ConstraintSolver(__global Body *bodies, __global Constraint *constraints, float dt)
{
  //Solves contacts in a constraint format using an LCP solver (PGS)
  
  #ifdef USE_PGS
    PGSSolver(bodies, constraints, dt);  
  #elif USE_SI
    SequentialImpulseSolver(bodies, constraints, dt);
  #endif
  
  
}


