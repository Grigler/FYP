#ifndef __FYP_PIPELINE__
#define __FYP_PIPELINE__

#include <list>
#include <memory>

#include <clew/clew.h>

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

namespace FYP
{
  //Arbitrary value, should be from hardware limits
#define MAX_BODIES 2048
#define FIXED_TIME 0.08f

  class Context;
  class Body;

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
    static cl_mem bvMem;
    struct AABBStruct
    {
      glm::vec3 min;
      glm::vec3 max;
    };
    static cl_mem bvPairs;
    struct BVPair
    {
      int leftID;
      int rightID;
    };


    static void NarrowPhase();

    static void ConstraintSolving();
    static cl_kernel constraintSolverKernel;

    static void Integrate();
    static cl_kernel integrationKernel;


    struct BodyStruct
    {
      glm::vec3 pos;
      glm::quat orien;
      float mass;
      //glm::mat3 inertiaTensor;

      glm::vec3 linearVel;
      glm::vec3 angularVel;
    };
    static std::list< std::shared_ptr<Body> > bodies;
    static cl_mem bodiesMem;
    static size_t bodiesMemSize;


  };
}

#endif