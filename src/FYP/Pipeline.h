#ifndef __FYP_PIPELINE__
#define __FYP_PIPELINE__

namespace FYP
{
  //Arbitrary value, should be from hardware limits
#define MAX_BODIES 256
#define FIXED_TIME 0.08f

  class Pipeline
  {
  public:
    //Initialises contexts and system stuff
    static void Init();

    static void Update(float _dt);

    static void BroadPhase();
    static void NarrowPhase();
    static void ConstraintSolving();
    static void Integrate();

  private:
    static float dt;

    static void InitKernels();

  };
}

#endif