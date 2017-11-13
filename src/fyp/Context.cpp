#include "Context.h"

#include <iostream>

using namespace FYP;

void Context::Create(cl_device_type _devices)
{
  //Platform
  cl_uint numPlatforms;
  clGetPlatformIDs(1, &platform, &numPlatforms);
  printf("Platforms available: %i\n", numPlatforms);
  //Device using _devices bitfield
  clGetDeviceIDs(platform, _devices, 10, devices, &numDevices);
  printf("Number of devices found: %i\n", numDevices);
  //Creating context
  context = new cl_context();
  cl_int errCode;
  *context = clCreateContext(0, 1, devices, NULL, NULL, &errCode);
  if (errCode == CL_SUCCESS)
  {
    printf("Context created\n");
  }
  else
  {
    printf("Context failed with code: %i\n", errCode);
  }
  //Creating commandQueues for each device
  for (size_t i = 0; i < numDevices; i++)
  {
    commandQueue[i] = clCreateCommandQueue(*context, devices[i], NULL, &errCode);
    if (errCode == NULL)
    {
      printf("Command Queue %i Created\n", i);
    }
    else
    {
      printf("Command Queue %i failed to create\n", i);
    }
  }

}