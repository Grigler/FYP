#include "Pipeline.h"

#include "Context.h"
#include "util.h"

#include "Body.h"

#include <iostream>

using namespace FYP;

cl_kernel Pipeline::integrationKernal;

std::list< std::shared_ptr<Body> > Pipeline::bodies;
cl_mem Pipeline::bodiesMem;
size_t Pipeline::bodiesMemSize;

std::shared_ptr<Context> Pipeline::context = std::make_shared<Context>();
float Pipeline::dt = 0.0f;

void Pipeline::Init()
{
  context->Create(CL_DEVICE_TYPE_ALL);

  //DEBUG
  for(int i = 0; i < MAX_BODIES; i++)
    bodies.push_back(std::make_shared<Body>());

  InitKernels();
}
void Pipeline::InitKernels()
{
  cl_int ret;

  bodiesMem = clCreateBuffer(*context->GetId(), CL_MEM_READ_WRITE, sizeof(BodyStruct)*MAX_BODIES, NULL, &ret);

  std::string src = Util::ReadFromFile("../kernel_code/AdjustPos.txt");
  const char *srcArr[] = { src.c_str() };
  const size_t sizeArr[] = { src.size() };

  cl_program program = clCreateProgramWithSource(*context->GetId(), 1,
    srcArr, sizeArr, &ret);
  
  printf("> Program Building...\n");
  ret = clBuildProgram(program, 1, &context->devices[0], NULL, NULL, NULL);
  printf("> Program Built\n");

  printf("> Creating Integration Kernel\n");
  integrationKernal = clCreateKernel(program, "Adjust", &ret);
  printf("> Created Integration Kenrel\n");
}

void Pipeline::BufferBodies()
{
  static BodyStruct bs[MAX_BODIES];
  int it = 0;
  for (auto i = bodies.begin(); i != bodies.end(); i++)
  {
    std::shared_ptr<Body> b = (*i);
    bs[it].pos = b->pos;
    bs[it].orien = b->orientation;
    bs[it].mass = b->mass;

    it++;
  }
  bodiesMemSize = it;
  clEnqueueWriteBuffer(context->commandQueue[0], bodiesMem, CL_TRUE, 0, sizeof(BodyStruct)*it, bs,
    0, NULL, NULL);
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
  struct AABB
  {
    glm::vec3 min;
    glm::vec3 max;
  };
  static AABB bv[MAX_BODIES];

  int it = 0;
  for (auto i = bodies.begin(); i != bodies.end(); i++)
  {
    std::shared_ptr<Body> b = (*i);

    bv[it].min = b->bvMin;
    bv[it].max = b->bvMax;

    it++;
  }


}

void Pipeline::NarrowPhase()
{

}

void Pipeline::ConstraintSolving()
{

}

void Pipeline::Integrate()
{
  BufferBodies();

  clSetKernelArg(integrationKernal, 0, sizeof(bodiesMem), &bodiesMem);

  clSetKernelArg(integrationKernal, 1, sizeof(float), &dt);

  clEnqueueNDRangeKernel(context->commandQueue[0], integrationKernal, 1, NULL, 
    &bodiesMemSize, NULL, NULL, NULL, NULL);

  clFinish(context->commandQueue[0]);


  static BodyStruct bs[MAX_BODIES];
  clEnqueueReadBuffer(context->commandQueue[0], bodiesMem, CL_TRUE, 0,
    sizeof(BodyStruct)*bodiesMemSize, bs, NULL, NULL, NULL);

  size_t it = 0;
  for (auto i = bodies.begin(); i != bodies.end(); i++)
  {
    std::shared_ptr<Body> b = (*i);
    b->pos = bs[it].pos;
  }
}