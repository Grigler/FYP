#include "Context.h"

#include "util.h"

#include <iostream>

int main()
{
  if(clewInit())
  {
    return 1;
  }

  /*
  //Getting possible platforms available
  cl_platform_id platforms[5];
  cl_uint numPlatforms;
  clGetPlatformIDs(5, platforms, &numPlatforms);
  printf("Platforms: %i\n", numPlatforms);

  //Getting possible devices into cl_device_id*
  cl_device_id devices;
  cl_uint availableDeviceCount;
  clGetDeviceIDs(platforms[0], CL_DEVICE_TYPE_GPU, 1, &devices, &availableDeviceCount);
  printf("Available Devices on platform 1: %i\n", availableDeviceCount);

  //Creating cl_context using desired cl_device_id - error callback registered here
  cl_context *context = new cl_context();
  cl_int errCode;
  *context = clCreateContext(0, 1, &devices, NULL, NULL, &errCode);
  if (context > 0)
  {
    printf("Context Created on device\n");
  }
  else
  {
    printf("Context Failed to be created on device\n");
  }

  //Creating commandQueue
  cl_command_queue commandQueue;
  commandQueue = clCreateCommandQueue(*context, devices, NULL, &errCode);
  if (errCode == NULL)
  {
    printf("Command Queue create for context\n");
  }
  else
  {
    printf("Command queue failed to create\n");
  }
  */

  fyp::Context context;
  context.Create(CL_DEVICE_TYPE_ALL);
  
  std::string test = fyp::util::ReadFromFile("kernel code/test.txt");

  std::cout << test << std::endl;

  //Arbritrary key hold
  getchar();

  return 0;
}