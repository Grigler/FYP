#include "Pipeline.h"

#include "Context.h"
#include "util.h"

#include "Body.h"

#include <iostream>
#include <vector>

using namespace FYP;

cl_kernel Pipeline::integrationKernel;
cl_kernel Pipeline::constraintSolverKernel;
cl_kernel Pipeline::broadPhaseKernel;

cl_mem Pipeline::bvMem;
cl_mem Pipeline::bvPairs;

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
  
  printf("\n> Program Building...\n");
  ret = clBuildProgram(program, 1, &context->devices[0], NULL, NULL, NULL);
  printf("> Program Built\n\n");

  printf("> Creating Integration Kernel\n");
  integrationKernel = clCreateKernel(program, "Adjust", &ret);
  printf("> Created  Integration Kenrel\n\n");

  printf("> Creating Broadphase Kernel\n");
  broadPhaseKernel = clCreateKernel(program, "BroadPhase", &ret);
  printf("\t> Creating Buffers\n");
  bvMem = clCreateBuffer(*context->GetId(), CL_MEM_READ_WRITE, sizeof(AABBStruct)*MAX_BODIES, NULL, &ret);
  bvPairs = clCreateBuffer(*context->GetId(), CL_MEM_READ_WRITE, sizeof(BVPair)*MAX_BODIES*MAX_BODIES, NULL, &ret);
  printf("> Created  Broadphase Kernel\n\n");

  printf("> Creating Constraint Solver Kernel\n");
  constraintSolverKernel = clCreateKernel(program, "ConstraintSolver", &ret);
  printf("> Created  Constraint Solver Kernel\n\n");
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
    ConstraintSolving();
    Integrate();

    dt = 0.0f;
  }
}

void Pipeline::BroadPhase()
{
  //AABB broadphase giving a list of bodies to pass into narrowphase
  
  static AABBStruct bv[MAX_BODIES];

  int it = 0;
  for (auto i = bodies.begin(); i != bodies.end(); i++)
  {
    std::shared_ptr<Body> b = (*i);

    bv[it].min = b->bvMin;
    bv[it].max = b->bvMax;
    bv[it].indx = it;

    it++;
  }

  //Writing BV data to buffer with blocking write
  clEnqueueWriteBuffer(context->commandQueue[0], bvMem, CL_TRUE,
    0, sizeof(AABBStruct)*it, bv, NULL, NULL, NULL);
  //Setting full list of BVs as input
  clSetKernelArg(broadPhaseKernel, 0, sizeof(bvMem), &bvMem);

  //Output pairs
  clSetKernelArg(broadPhaseKernel, 1, sizeof(bvPairs), &bvPairs);

  //Setting number of bvs
  clSetKernelArg(broadPhaseKernel, 2, sizeof(int), &it);

  clEnqueueNDRangeKernel(context->commandQueue[0], broadPhaseKernel, 1,
    NULL, (size_t*)&it, NULL, NULL, NULL, NULL);
  //Blocking wait
  clFinish(context->commandQueue[0]);

  static BVPair pairData[MAX_BODIES*MAX_BODIES];

  //Reading collision passes
  clEnqueueReadBuffer(context->commandQueue[0], bvPairs, CL_TRUE,
    0, sizeof(BVPair)*it*it, pairData, NULL, NULL, NULL);



  NarrowPhase();
}

/*
void Pipeline::BroadPhase()
{
  

  //TODO replace with array
  std::vector<BroadTestPair> pairs;

  int indx = 0;
  //Create array of BroadTestPair
  for (auto i = bodies.begin(); i != bodies.end(); i++)
  {
    for (auto j = i; j != bodies.end(); j++)
    {
      BroadTestPair thisPair;
      thisPair.minL = (*i)->bvMin;
      thisPair.maxL = (*i)->bvMax;

      thisPair.minR = (*j)->bvMin;
      thisPair.maxR = (*j)->bvMax;

      thisPair.indx = indx;
      indx++;

      //TODO replace with array
      pairs.push_back(thisPair);
    }

  }

  std::vector<int> outputIDs;
  outputIDs.reserve(pairs.size());

  //Buffer data - blocking write
  clEnqueueWriteBuffer(context->commandQueue[0], bvMem, CL_TRUE,
    0, sizeof(BroadTestPair)*indx, &pairs[0], NULL, NULL, NULL);

  //Setting kernel args
  clSetKernelArg(broadPhaseKernel, 0, sizeof(bvMem), &bvMem);
  clSetKernelArg(broadPhaseKernel, 1, sizeof(bvPairs), &bvPairs);

  //run kernel
  clEnqueueNDRangeKernel(context->commandQueue[0], broadPhaseKernel, 1,
    NULL, (size_t*)&indx, NULL, NULL, NULL, NULL);

  //Read back colliding pairs


  //Send data to narrowphase

}
*/
void Pipeline::NarrowPhase()
{
  //Reading pair data from broadphase
  
}

void Pipeline::ConstraintSolving()
{

}

void Pipeline::Integrate()
{
  BufferBodies();

  clSetKernelArg(integrationKernel, 0, sizeof(bodiesMem), &bodiesMem);

  clSetKernelArg(integrationKernel, 1, sizeof(float), &dt);

  clEnqueueNDRangeKernel(context->commandQueue[0], integrationKernel, 1, NULL,
    &bodiesMemSize, NULL, NULL, NULL, NULL);

  //Would be needed so that nothing is read back into the host code before
  //that kernel is done running
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