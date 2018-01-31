#ifndef __FYP_PIPELINE__
#define __FYP_PIPELINE__

#include <vector>
#include <memory>

#include <clew/clew.h>

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include "BodyStruct.h"

namespace FYP
{
  //Arbitrary value, should be from hardware limits
#define MAX_BODIES 2048
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

  private:
    static std::shared_ptr<Context> context;

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

    static void Integrate();
    static cl_kernel integrationKernel;

    struct BodyStruct
    {
      glm::vec3 pos;
      glm::quat orien;
      float mass;
      glm::mat3 inertiaTensor;

      float linearDrag;
      float angularDrag;

      //AABB struct
      glm::vec3 origMin;
      glm::vec3 origMax;
      glm::vec3 min;
      glm::vec3 max;
      int bodyID;

      glm::vec3 linearVel;
      glm::vec3 angularVel;
    };
    //static std::list< std::shared_ptr<Body> > bodies;
    static std::vector<Body> bodies;
    static cl_mem bodiesMem;

  };
}

#endif