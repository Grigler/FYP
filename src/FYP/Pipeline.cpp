#include "Pipeline.h"

#include "Context.h"
#include "util.h"

//#include "Body.h"
#include "BodyStruct.h"

#include <iostream>
#include <vector>

using namespace FYP;

cl_kernel Pipeline::integrationKernel;
cl_kernel Pipeline::broadPhaseKernel;
cl_kernel Pipeline::narrowPhaseKernel;
cl_kernel Pipeline::constraintSolverKernel;

cl_mem Pipeline::idPairsMem;
cl_mem Pipeline::pairIndxMem;
cl_mem Pipeline::pairsFoundMem;

cl_mem Pipeline::constraintsMem;
cl_mem Pipeline::lastConstraintIndxMem;

std::vector<Body> Pipeline::bodies;
cl_mem Pipeline::bodiesMem;

std::shared_ptr<Context> Pipeline::context = std::make_shared<Context>();
float Pipeline::dt = 0.0f;

void Pipeline::Init()
{
  context->Create(CL_DEVICE_TYPE_ALL);

  //DEBUG
  for (int i = 0; i < MAX_BODIES; i++)
  {
    Body b = Body();
    bodies.push_back(b);
  }

  InitKernels();
}
void Pipeline::InitKernels()
{
  cl_int ret;

  printf("\n> Buffer Creation\n");
  bodiesMem = clCreateBuffer(*context->GetId(), CL_MEM_READ_WRITE, sizeof(Body)*MAX_BODIES, NULL, &ret);
  
  idPairsMem = clCreateBuffer(*context->GetId(), CL_MEM_READ_WRITE,
    sizeof(idPairs)*MAX_BODIES*MAX_BODIES, NULL, &ret);
  pairIndxMem = clCreateBuffer(*context->GetId(), CL_MEM_READ_WRITE, sizeof(int), NULL, &ret);
  pairsFoundMem = clCreateBuffer(*context->GetId(), CL_MEM_READ_WRITE, sizeof(int), NULL, &ret);
  
  constraintsMem = clCreateBuffer(*context->GetId(), CL_MEM_READ_WRITE, sizeof(Constraint)*MAX_BODIES*(MAX_BODIES/8),
    NULL, &ret);
  lastConstraintIndxMem = clCreateBuffer(*context->GetId(), CL_MEM_READ_WRITE, sizeof(int), NULL, &ret);
  printf("> Buffers Created\n");

  //std::string src = Util::ReadFromFile("../kernel_code/AdjustPos.txt");
  std::string src = Util::ReadFromFile("../kernel_code/PipelineTypes.txt");
  const char *srcArr[] = { src.c_str() };
  const size_t sizeArr[] = { src.size() };

  cl_program program = clCreateProgramWithSource(*context->GetId(), 1,
    srcArr, sizeArr, &ret);
  
  printf("\n> Program Building...\n");
  ret = clBuildProgram(program, 1, &context->devices[0], NULL, NULL, NULL);
  printf("> Program Built\n\n");

  printf("> Creating Integration Kernel\n");
  integrationKernel = clCreateKernel(program, "Integrate", &ret);
  printf("> Created  Integration Kenrel\n\n");

  printf("> Creating Broadphase Kernel\n");
  broadPhaseKernel = clCreateKernel(program, "BroadPhase", &ret);
  printf("> Created  Broadphase Kernel\n\n");

  printf("> Creating Narrowphase Kernel\n");
  narrowPhaseKernel = clCreateKernel(program, "NarrowPhase", &ret);
  printf("> Created  Narrowphase Kernel\n\n");

  printf("> Creating Constraint Solver Kernel\n");
  constraintSolverKernel = clCreateKernel(program, "ConstraintSolver", &ret);
  printf("> Created  Constraint Solver Kernel\n\n");
}

void Pipeline::BufferBodies()
{
  clEnqueueWriteBuffer(context->commandQueue[0], bodiesMem, CL_TRUE,
    0, sizeof(Body)*bodies.size(), &bodies.at(0), 0, NULL, NULL);
}

void Pipeline::Update(float _dt)
{
  dt += _dt;

  if (dt >= FIXED_TIME)
  {
    //printf("\tPhysics Update\n");
    Integrate();
    BroadPhase();
    NarrowPhase();
    ConstraintSolving();

    dt = 0.0f;
  }
}

void Pipeline::BroadPhase()
{
  clSetKernelArg(broadPhaseKernel, 0, sizeof(bodiesMem), &bodiesMem);
  int numBodies = bodies.size();
  clSetKernelArg(broadPhaseKernel, 1, sizeof(int), &numBodies);
  clSetKernelArg(broadPhaseKernel, 2, sizeof(idPairsMem), &idPairsMem);
  clSetKernelArg(broadPhaseKernel, 3, sizeof(pairIndxMem), &pairIndxMem);
  clSetKernelArg(broadPhaseKernel, 4, sizeof(pairsFoundMem), &pairsFoundMem);

  cl_int zero = 0;
  //Writing zero to both int buffers
  clEnqueueWriteBuffer(context->commandQueue[0], pairIndxMem, CL_FALSE,
    0, sizeof(int), &zero, 0, NULL, NULL);
  clEnqueueWriteBuffer(context->commandQueue[0], pairsFoundMem, CL_TRUE,
    0, sizeof(int), &zero, 0, NULL, NULL);

  size_t sizeHold = bodies.size();
  clEnqueueNDRangeKernel(context->commandQueue[0], broadPhaseKernel, 1, NULL,
    &sizeHold, NULL, NULL, NULL, NULL);

  clFinish(context->commandQueue[0]);
}
void Pipeline::NarrowPhase()
{
  //Reading pair data from broadphase
  clSetKernelArg(narrowPhaseKernel, 0, sizeof(idPairsMem), &idPairsMem);
  clSetKernelArg(narrowPhaseKernel, 1, sizeof(bodiesMem), &bodiesMem);
  clSetKernelArg(narrowPhaseKernel, 2, sizeof(constraintsMem), &constraintsMem);
  clSetKernelArg(narrowPhaseKernel, 3, sizeof(lastConstraintIndxMem), &lastConstraintIndxMem);

  int pairCount = 0;
  clEnqueueWriteBuffer(context->commandQueue[0], lastConstraintIndxMem, CL_TRUE,
    0, sizeof(int), &pairCount, 0, NULL, NULL); //Using pairCount towrite 0 to indx for mem
  clEnqueueReadBuffer(context->commandQueue[0], pairsFoundMem, CL_TRUE,
    0, sizeof(int), &pairCount, 0, NULL, NULL);

  //Might not be needed - avoiding any ptr type casting from int to size_t
  size_t workAmnt = pairCount;
  clEnqueueNDRangeKernel(context->commandQueue[0], narrowPhaseKernel, 1, NULL,
    &workAmnt, NULL, NULL, NULL, NULL);

  clFinish(context->commandQueue[0]);
}

void Pipeline::ConstraintSolving()
{
  clSetKernelArg(constraintSolverKernel, 0, sizeof(bodiesMem), &bodiesMem);
  clSetKernelArg(constraintSolverKernel, 1, sizeof(constraintsMem), &constraintsMem);
  clSetKernelArg(constraintSolverKernel, 2, sizeof(float), &dt);

  int constraintCount = 0;
  clEnqueueReadBuffer(context->commandQueue[0], lastConstraintIndxMem, CL_TRUE,
    0, sizeof(int), &constraintCount, 0, NULL, NULL);

  //Might not be needed - avoiding any ptr type casting from int to size_t
  size_t workAmnt = constraintCount;
  if (workAmnt > 0)
    clEnqueueNDRangeKernel(context->commandQueue[0], constraintSolverKernel, 1, NULL,
      &workAmnt, NULL, NULL, NULL, NULL);

  clFinish(context->commandQueue[0]);
}

void Pipeline::Integrate()
{
  BufferBodies();

  clSetKernelArg(integrationKernel, 0, sizeof(bodiesMem), &bodiesMem);

  clSetKernelArg(integrationKernel, 1, sizeof(float), &dt);

  size_t sizeHold = bodies.size();

  clEnqueueNDRangeKernel(context->commandQueue[0], integrationKernel, 1, NULL,
    &sizeHold, NULL, NULL, NULL, NULL);

  //TODO - Double check the commandQueue doesn't implicitly handle this
  //Would be needed so that nothing is read back into the host code before
  //that kernel is done running
  clFinish(context->commandQueue[0]);

  //static BodyStruct bs[MAX_BODIES];
  clEnqueueReadBuffer(context->commandQueue[0], bodiesMem, CL_TRUE, 0,
    sizeof(Body)*sizeHold, &bodies[0], NULL, NULL, NULL);
}