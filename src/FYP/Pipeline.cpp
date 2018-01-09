#include "Pipeline.h"

#include "Context.h"
#include "util.h"

#include "Body.h"

#include <iostream>

using namespace FYP;

std::list< std::shared_ptr<Body> > Pipeline::bodies;

std::shared_ptr<Context> Pipeline::context = std::make_shared<Context>();
float Pipeline::dt = 0.0f;

void Pipeline::Init()
{
  context->Create(CL_DEVICE_TYPE_ALL);

  //DEBUG
  bodies.push_back(std::make_shared<Body>());

  InitKernels();
}
void Pipeline::InitKernels()
{
  struct BodyStruct
  {
    glm::vec3 pos;
    glm::quat orien;
    float mass;
    //glm::mat3 inertiaTensor;
  };

  cl_int ret;
  cl_mem bodiesIn = clCreateBuffer(*context->GetId(), CL_MEM_READ_WRITE, sizeof(BodyStruct)*MAX_BODIES, NULL, &ret);
  //cl_mem bodiesOut = clCreateBuffer(*context->GetId(), CL_MEM_READ_WRITE, sizeof(BodyStruct)*MAX_BODIES, NULL, &ret);

  BodyStruct bs[MAX_BODIES];
  int it = 0;
  for (auto i = bodies.begin(); i != bodies.end(); i++)
  {
    std::shared_ptr<Body> b = (*i);
    bs[it].pos = b->pos;
    bs[it].orien = b->orientation;
    bs[it].mass = b->mass;
    
    it++;
  }

  ret = clEnqueueWriteBuffer(context->commandQueue[0], bodiesIn, CL_TRUE, 0, sizeof(BodyStruct)*it, bs,
    0, NULL, NULL);

  std::string src = Util::ReadFromFile("../kernel_code/AdjustPos.txt");
  const char *srcArr[] = { src.c_str() };
  const size_t sizeArr[] = { src.size() };

  cl_program program = clCreateProgramWithSource(*context->GetId(), 1,
    srcArr, sizeArr, &ret);
  ret = clBuildProgram(program, 1, &context->devices[0], NULL, NULL, NULL);

  cl_kernel kernel = clCreateKernel(program, "Adjust", &ret);

  while (true)
  {
    clSetKernelArg(kernel, 0, sizeof(bodiesIn), &bodiesIn);
    ret = clEnqueueTask(context->commandQueue[0], kernel, 0, NULL, NULL);
    clFlush(context->commandQueue[0]);
  }
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