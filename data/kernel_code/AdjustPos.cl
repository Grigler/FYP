typedef struct
{
  float3 pos;
  float4 orientation;
  float mass;
  
  float3 linearVel;
  float3 angularVel;
} Body;

__kernel void Adjust(__global Body *input, float dt)
{
  int gid = get_global_id(0);
  
  input[gid].linearVel += (float3)(0.0f, -9.81f, 0.0f)*dt;
  input[gid].pos += input[gid].linearVel * dt;
  
}

typedef struct
{
  float3 min;
  float3 max;
  int indx;
} BV;

typedef struct
{
  int leftIndx;
  int rightIndx;  
} BVPair;


//This could be better done by batching each bv-pair check into a thread
__kernel void BroadPhase(__global BV *input, __global BVPair *output, int inputNum)
{
  int gid = get_global_id(0);
  __private BV thisBV = input[gid];
  __private BV againstBV;
  
  for(int i = gid+1; i < inputNum; i++)
  {
    againstBV = input[i];
    
    //AABB check for thisBV and againstBV
    bool xResult = thisBV.max.x <= againstBV.min.x || thisBV.min.x >= againstBV.max.x;
    bool yResult = thisBV.max.y <= againstBV.min.y || thisBV.min.y >= againstBV.max.y;
    bool zResult = thisBV.max.z <= againstBV.min.z || thisBV.min.z >= againstBV.max.z;
    
    //bool xResult = true;
    //bool yResult = true;
    //bool zResult = true;
    
    if(xResult & yResult & zResult)
    {
      output[gid+i].leftIndx = thisBV.indx;
      output[gid+i].rightIndx = againstBV.indx;
    }
    else
    {
      output[gid+i].leftIndx = -1;
      output[gid+i].rightIndx = -1;
    }
  }
}

/*
typedef struct
{
  float3 minL, maxL;
  float3 minR, maxR;
  int indx;
} TestPair;

__kernel void BroadPhase(__global TestPair *pair, __global int *output)
{
  int gid = get_global_id(0);
  
  __private TestPair thisPair = pair[gid];
  
  __private bool xResult = thisPair.maxL.x <= thisPair.minR.x || thisPair.minL.x >= thisPair.maxR.x;
  __private bool yResult = thisPair.maxL.y <= thisPair.minR.y || thisPair.minL.y >= thisPair.maxR.y;
  __private bool zResult = thisPair.maxL.z <= thisPair.minR.z || thisPair.minL.z >= thisPair.maxR.z;
  
  if(xResult && yResult && zResult)
  {
    output[gid] = thisPair.indx;
  }
  else
  {
    //Using -1 as a flag for no collision
    output[gid] = -1;
  }
}
*/
  __kernel void ConstraintSolver(__global float *placeHolder)
  {
    int gid = get_global_id(0);
    placeHolder[gid] = 2.0f;
  }