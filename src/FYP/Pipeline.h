#ifndef __FYP_PIPELINE__
#define __FYP_PIPELINE__

#include <vector>
#include <memory>

#include <clew/clew.h>
#include <GL/glew.h>

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include "BodyStruct.h"

namespace FYP
{
  //Arbitrary value, should be from hardware limits
#define MAX_BODIES 256
#define FIXED_TIME 0.008f

  class Context;
  //class Body;
  //struct Body;

  class Pipeline
  {
  public:
    //Initialises contexts and system stuff
    static void Init();

    static void Update(float _dt);

    static void RegisterOutputVBOBuffer(GLuint _vboID);
    static void CopyPosToVBOBuffer(GLuint _vboID);

  private:
    static std::shared_ptr<Context> context;

    static cl_mem glVBO;

    static float dt;

    static void InitKernels();

    static void BufferBodies();

    static void BroadPhase();
    static cl_kernel broadPhaseKernel;
    static cl_mem idPairsMem;
    static cl_mem pairIndxMem;
    static cl_mem pairsFoundMem;
    struct idPairs
    {
      cl_int left;
      cl_int right;
    };

    static void NarrowPhase();
    static cl_kernel narrowPhaseKernel;

    static void ConstraintSolving();
    static cl_kernel constraintSolverKernel;
    static cl_mem constraintsMem;
    static cl_mem lastConstraintIndxMem;

    struct Constraint
    {
      cl_float appliedImpulse;

      cl_float3 normal;
      cl_float3 worldPos;

      cl_float jacDiagABInv;

      cl_float lowerLimit;
      cl_float upperLimit;

      cl_int leftIndx;
      cl_float3 leftDeltaLinVel;
      cl_float3 leftDeltaAngVel;
      cl_float3 leftTorqueArm;

      cl_int rightIndx;
      cl_float3 rightDeltaLinVel;
      cl_float3 rightDeltaAngVel;
      cl_float3 rightTorqueArm;
    };

    static void Integrate();
    static cl_kernel integrationKernel;

    //static std::list< std::shared_ptr<Body> > bodies;
    static std::vector<Body> bodies;
    static cl_mem bodiesMem;

  };
}

#endif