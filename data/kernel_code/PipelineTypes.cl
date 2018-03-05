//Quat and Mat3 utility functions interpreted from Bullet3 Source
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

typedef struct 
{
  float3 pos;
  Quat orien;
  //float x,y,z,w;
  
  Mat3 invInertiaTensor;
  //float x1,x2,x3;
  //float y1,y2,y3;
  //float z1,z2,z3;
  
  float mass;
  
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
  float sphereRadius;
  
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

__kernel void Integrate(__global Body *bodies, float dt)
{
  int gid = get_global_id(0);
  
  //Linear Vel  
  //TODO - handle 0/infinite mass
  float mass = max(bodies[gid].mass, 0.5f);
  float3 linearAccel = bodies[gid].accumForce / mass;
  //Gravity
  linearAccel += (float3)(0.0f, -9.81f, 0.0f);
  bodies[gid].linearVel += linearAccel * (dt);
  
  //Angular
  bodies[gid].angularVel += 
    mtMul1(bodies[gid].invInertiaTensor, bodies[gid].accumTorque) * dt;

  //Simplified drag applications - estimation, not strictly accurate
  bodies[gid].linearVel *= (1.0f - (dt) * bodies[gid].linearDrag);
  bodies[gid].angularVel *= (1.0f - (dt) * bodies[gid].angularDrag);
  
  //Linear
  bodies[gid].pos += bodies[gid].linearVel * (dt);
  
  //Angular
  Quat oldQuat = bodies[gid].orien;
  bodies[gid].orien.val = oldQuat.val +
      quatMultV4((dt/2.0f)*bodies[gid].angularVel,oldQuat);
  //Normalising
  bodies[gid].orien = quatNorm(bodies[gid].orien);
  float4 q = bodies[gid].orien.val;
  
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

#define MAX_BODIES 2048

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

bool SphereSphere(Body l, Body r)
{
  float radSum = l.sphereRadius + r.sphereRadius;
  float dist = distance(l.pos, r.pos);
  return dist < radSum;
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
  float lowerLimit; //0 for contact
  float upperLimit;  //< infinite for contact
  
  //indx values for bodies in bodies buffer
  int leftIndx;
  float3 leftDeltaLinVel;
  float3 leftDeltaAngVel;
  float3 leftTorqueArm;
  
  int rightIndx;
  float3 rightDeltaLinVel;
  float3 rightDeltaAngVel;
  float3 rightTorqueArm;
  
} Constraint;

float GetJacM(Body l, Body r, float3 contactPos, float3 contactNorm)
{
  float3 rnL = cross((contactPos - l.pos), contactNorm);
  float3 rnR = cross((contactPos - r.pos), contactNorm);
  
  //J*M^-1 === 1/m * norm <--for linear
  float3 jmL = (1.0f/l.mass) * contactNorm;
  float3 jmR = (1.0f/r.mass) * -contactNorm;
  
  //J^T * (JM^-1) === dot(norm, JM^-1)
  float vL = dot(jmL, contactNorm);
  float vR = dot(jmR, -contactNorm);
  
  float combinedInverseMass = (1.0f/l.mass)+(1.0f/r.mass);
  
  return -1.0f / (combinedInverseMass + vL + vR);
}

__kernel void NarrowPhase(__global IDPair *pairs, __global Body *bodies, __global Constraint *constraints, volatile __global int *lastConstraintIndx)
{
  int gid = get_global_id(0);
  
  Body leftBody = bodies[pairs[gid].leftIndx];
  Body rightBody = bodies[pairs[gid].rightIndx];
  
  //Assuming spheres for simplest collision atm 
  if(SphereSphere(leftBody, rightBody))
  {
    float3 midLine = leftBody.pos - rightBody.pos;
    float dist = distance(leftBody.pos, rightBody.pos);
    
    float3 contactNorm = midLine * (1.0f / dist);
    float3 contactPoint = leftBody.pos - (midLine*0.5f);
    float penetrationDepth = (leftBody.sphereRadius + rightBody.sphereRadius) - dist;
   
    //Converting into constraint format
    Constraint c;
    c.appliedImpulse = 0.0f;
    c.normal = contactNorm;
    c.worldPos = contactPoint;
    c.depth = penetrationDepth;
    
    //Still don't know wtf this is
    c.jacDiagABInv = 0.0f;
    
    c.lowerLimit = 0.0f;
    c.upperLimit = 9999999.9f;;
    
    c.leftIndx = pairs[gid].leftIndx;
    c.leftDeltaLinVel = (float3)(0.0f);
    c.leftDeltaAngVel = (float3)(0.0f);
    
    c.rightIndx = pairs[gid].rightIndx;
    c.rightDeltaLinVel = (float3)(0.0f);
    c.rightDeltaAngVel = (float3)(0.0f);
    
    c.jacDiagABInv = GetJacM(leftBody, rightBody, contactPoint, contactNorm);
    //c.jacDiagABInv += GetJacM(rightBody, contactPoint, contactNorm);
    //c.jacDiagABInv = c.jacDiagABInv;
    
    //Write into constraint buffer
    int indx = atomic_inc(lastConstraintIndx);
    constraints[indx] = c;
    
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
  float invMassL = 1.0f / l->mass, invMassR = 1.0f / r->mass;
  
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
    float beta = 0.8f; //Roughly a percentage of depth to remove per-iteration
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
  float invMassL = 1.0f / l->mass, invMassR = 1.0f / r->mass;
  
  for(int iteration = 0; iteration < 4; iteration++)
  {
    //printf("Iteration: %i\n", iteration);
    
    //float3 rL = c->worldPos - posL;
    //float3 rR = c->worldPos - posR;
    //float3 angL = cross(rL, c->normal);
    //float3 angR = -cross(rR, c->normal);
    
    linL = l->linearVel;
    linR = r->linearVel;
    float relVel = dot(c->normal, linL) + -dot(c->normal, linR); //And ang
    //printf("\tRelVel: %f\n", relVel);
    
    float jacRelVel = relVel * c->jacDiagABInv;
    //printf("\tJac: %f\n", c->jacDiagABInv);
    //printf("\tJacRelVel: %f\n", jacRelVel);
    
    //b value
    float beta = 0.8f; //Roughly a percentage of depth to remove per-iteration
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
    
    l->linearVel += linImpulseL;
    r->linearVel += linImpulseR;
        
    //linL += cL;
    //linR += cR;
    //ang+=
    //ang+=
    
    //Waiting for all other threads to finish their addition
    //to the relevant body's velocity (could be done local thread if batched)
    barrier(CLK_GLOBAL_MEM_FENCE);
  }
  
  
  //printf("\nFinalLinL: %f, %f, %f\n", linL.x, linL.y, linL.z);
  //printf("FinalLinR: %f, %f, %f\n", linR.x, linR.y, linR.z);

  //l->linearVel = linL;
  //r->linearVel = linR;

  //ang+=
  //ang+=
  
  //bodies[constraints[gid].leftIndx] = *l;
  //bodies[constraint[gid].rightIndx] = *r;
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


