#ifndef __FYP_PIPELINE__
#define __FYP_PIPELINE__

#include <list>
#include <memory>

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

    static void BroadPhase();
    

	static void NarrowPhase();

    static void ConstraintSolving();

    static void Integrate();
	static cl_kernel integrationKernal;

    static std::list< std::shared_ptr<Body> > bodies;
	static cl_mem bodies;


  };
}

#endif