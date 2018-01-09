#include "Pipeline.h"

using namespace FYP;

void Pipeline::Init()
{

  InitKernels();
}
void Pipeline::InitKernels()
{

}

void Pipeline::Update(float _dt)
{
  dt += _dt;

  if (dt >= FIXED_TIME)
  {
    BroadPhase();
    NarrowPhase();
    ConstraintSolving();
    Integrate();

    dt = 0.0f;
  }
}

void Pipeline::BroadPhase()
{
  //AABB broadphase giving a list of bodies to pass into narrowphase
}

void Pipeline::NarrowPhase()
{

}

void Pipeline::ConstraintSolving()
{

}

void Pipeline::Integrate()
{

}