//TODO - Implement
typedef struct
{
  float x, y, z, w;
} Quat;
typedef struct
{
  //TODO
  float x1,x2,x3;
  float y1,y2,y3;
  float z1,z2,z3;
} Mat3;

float3 VecMultMat(float3 v, Mat3 m)
{
  //TODO
  float3 ret;
  
  return ret;
}

typedef struct 
{
  float3 pos;
  float x,y,z,w;
  
  //Mat3 invInertiaTensor;
  float x1,x2,x3;
  float y1,y2,y3;
  float z1,z2,z3;
  
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

//Applies an impulse force to body b
void ApplyBodyImpulse(__global Body* b, float n, float3 loc)
{
  //TODO
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
  
  //Angular - TODO
  //angularVel += invInertiaTensor * torque * dt

  //Simplified drag applications - estimation, not strictly accurate
  bodies[gid].linearVel *= (1.0f - (dt) * bodies[gid].linearDrag);
  bodies[gid].angularVel *= (1.0f - (dt) * bodies[gid].angularDrag);
  
  //Applying velocities to adjust position and orientation
  if(bodies[gid].mass == 5.0f)
  {
    printf("\nPost integration vel: %f, %f, %f\n", 
    bodies[gid].linearVel.x, bodies[gid].linearVel.y, bodies[gid].linearVel.z);
  }
  
  bodies[gid].pos += bodies[gid].linearVel * (dt);
  //TODO - if integration is done first, should update BV
  //TODO - adjust rotation by angular velocity, need to lookup some Quat stuff
  
  //Resetting accumulators
  bodies[gid].accumForce = (float3)(0.0f);
  bodies[gid].accumTorque = (float3)(0.0f);
}

typedef struct
{
  int leftIndx;
  int rightIndx;
} IDPair;

#define MAX_BODIES 256

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
  //float3 arm = contactPos - b.pos;
  //float3 c = cross(arm, contactNorm);
  //v = cross(c*invInertiaTensor, arm);
  //float3 v;
  float lInvM = 1.0f / l.mass;
  float rInvM = 1.0f / r.mass;
  return -1.0f / (lInvM + rInvM);// + dot(contactNorm, v);
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
  
  //DEBUG
  //l->linearVel = -l->linearVel;//(float3)(0.0f, 100.0f, 0.0f);
  //r->pos = -r->linearVel;//(float3)(0.0f, 100.0f, 0.0f);
  //return;
  
  //Store private representations of vels, impulses and positions (reduces atomic ops)
  float3 posL = l->pos, posR = r->pos;
  float3 cPosL = c->worldPos + l->pos, cPosR = c->worldPos + r->pos;
  float3 linL = l->linearVel, linR = r->linearVel;
  float3 angL = l->angularVel, angR = r->angularVel;
  float invMassL = 1.0f / l->mass, invMassR = 1.0f / r->mass;
  
  for(int iteration = 0; iteration < 4; iteration++)
  {
    printf("Iteration: %i\n", iteration);
    
    //float iterationInitImpulse = c->impulse;
    
    //m = J M^-1 J^T
    //c->impulse = c->impulse - (1/m)(J * c->relativeVel);
    //c->impulse = max(0, c->impulse);
    //float3 v = c->relativeVel + (M^-1 * J^T)(c->impulse - dt);
    
    //float3 rL = c->worldPos - posL;
    //float3 rR = c->worldPos - posR;
    //float3 angL = cross(rL, c->normal);
    //float3 angR = -cross(rR, c->normal);
    float relVel = dot(c->normal, linL) - dot(c->normal, linR); //And ang
    printf("\tRelVel: %f\n", relVel);
    
    float jacRelVel = relVel * c->jacDiagABInv;
    printf("\tJacRelVel: %f\n", jacRelVel);
    
    //b value
    float beta = 0.0f;
    float b = -(beta/dt)*c->depth;// + relVel;
    printf("\tb: %f\n", b);
    
    float lamda = (jacRelVel);
    //Clamping to constraints for 'project' gauss-seidel
    lamda = max(lamda, 0.0f);
    
    printf("\tlamda: %f\n", lamda);
    //angL
    //angR
 
    //Calculating impulse forces from lambda+b
    float3 linImpulseL = (lamda+b) * -c->normal * invMassL;// * dt;
    float3 linImpulseR = (lamda+b) * c->normal * invMassR;// * dt;
    
    float3 cL = linImpulseL;// b;
    float3 cR = linImpulseR;// * b;
    
    linL += cL;
    linR += cR;
    //ang+=
    //ang+=
    
    printf("\timpL: %f, %f, %f\n", linImpulseL.x, linImpulseL.y, linImpulseL.z);
    printf("\timpR: %f, %f, %f\n", linImpulseR.x, linImpulseR.y, linImpulseR.z);
    
    //printf("\tvelL: %f, %f, %f\n", linL.x, linL.y, linL.z);
    //printf("\tvelR: %f, %f, %f\n", linR.x, linR.y, linR.z);
  }
  
  
  printf("\nFinalLinL: %f, %f, %f\n", linL.x, linL.y, linL.z);
  printf("FinalLinR: %f, %f, %f\n", linR.x, linR.y, linR.z);

  l->linearVel = linL;
  r->linearVel = linR;

  //ang+=
  //ang+=
  
  //bodies[constraints[gid].leftIndx] = *l;
  //bodies[constraint[gid].rightIndx] = *r;
}

void SolverImpulseAdd(__global float3 *deltaLinear, __global float3 *deltaAngular, float3 linearComp, float3 angularComp, float magnitude)
{
  *deltaLinear += linearComp*magnitude;//*linearFactor?
  *deltaAngular += angularComp*magnitude;//*angularFactor?
}

void SequentialImpulseSolver(__global Body *bodies, __global Constraint *constraints, float dt)
{
  int gid = get_global_id(0);
  
  __global Constraint *c = &constraints[gid];
  
  for(int iteration = 0; iteration < 4; iteration++)
  {
    float deltaImpulse = 0.0f; //c->rhs - c->appliedImpulse*c->cfm; // <-- as far as I can tell, this is for position easing
    float deltaVelLeftDotn = dot(c->normal, c->leftDeltaLinVel) + dot(c->leftTorqueArm, c->leftDeltaAngVel);
    float deltaVelRightDotn = -dot(c->normal, c->rightDeltaLinVel) + dot(c->rightTorqueArm, c->rightDeltaAngVel); 
    
    deltaImpulse -= deltaVelLeftDotn * c->jacDiagABInv;
    deltaImpulse -= deltaVelRightDotn * c->jacDiagABInv;
    
    float sum = c->appliedImpulse + deltaImpulse;
    if(sum < c->lowerLimit)
    {
      deltaImpulse = c->lowerLimit - c->appliedImpulse;
      c->appliedImpulse = c->lowerLimit;
    }
    else if(sum > c->upperLimit)
    {
      deltaImpulse = c->upperLimit - c->appliedImpulse;
      c->appliedImpulse = c->upperLimit;
    }
    else
    {
      c->appliedImpulse = sum;
    }
    
    //Applying delta impulse for this iteration
    //ApplyBodyImpulse(&bodies[c->leftIndx], deltaImpulse, c->normal);
    //ApplyBodyImpulse(&bodies[c->rightIndx], deltaImpulse, c->normal);
    Body b = bodies[c->leftIndx];
    float3 angularCompDEBUG = (float3)(0.0f);
    SolverImpulseAdd(&c->leftDeltaLinVel, &c->leftDeltaAngVel, c->normal*(1.0f/b.mass), angularCompDEBUG, deltaImpulse);
    b = bodies[c->rightIndx];
    SolverImpulseAdd(&c->rightDeltaLinVel, &c->rightDeltaAngVel, -c->normal*(1.0f/b.mass), angularCompDEBUG, deltaImpulse);
    //Results in a lot of atomic operations
  }
}

//Solver Define Flags
#define USE_PGS
//#define USE_SI

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


