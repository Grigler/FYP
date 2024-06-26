#include "Context.h"

//TODO - make GL API independent
#include <GL/glut.h>

#include <iostream>

using namespace FYP;

cl_context *Context::context = NULL;

void Context::Create(cl_device_type _devices)
{
  clewInit();
  
  //Platform
  cl_uint numPlatforms = 0;
  clGetPlatformIDs(1, &platform, &numPlatforms);
  printf("> Platforms available: %i\n", numPlatforms);

  //Device using _devices bitfield
  clGetDeviceIDs(platform, _devices, 10, devices, &numDevices);
  printf("> Number of devices found: %i\n", numDevices);

  //Creating context
  context = new cl_context();
  cl_int errCode;
  
  //Looking for GL context to create for sharing
  cl_context_properties p[] =
  {
    CL_GL_CONTEXT_KHR, (cl_context_properties)wglGetCurrentContext(),
    CL_WGL_HDC_KHR, (cl_context_properties)wglGetCurrentDC(), 0
  };
  
  *context = clCreateContext(p, 1, devices, NULL, NULL, &errCode);
  if (errCode == CL_SUCCESS)
  {
    printf("> Context created\n");
  }
  else
  {
    printf("! ERR: Context failed with code: %i\n", errCode);
  }

  //Creating commandQueues for each device
  for (size_t i = 0; i < numDevices; i++)
  {
    commandQueue[i] = clCreateCommandQueue(*context, devices[i], NULL, &errCode);
    if (errCode == NULL)
    {
      printf("> Command Queue %i Created\n", i);
    }
    else
    {
      printf("! ERR: Command Queue %i failed to create\n", i);
    }
  }

}
void Context::Release()
{
  clReleaseCommandQueue(commandQueue[0]);
}

cl_device_id *Context::GetFirstDevice(cl_device_type _ofType)
{
  return &devices[0];
  for (size_t i = 0; i < numDevices; i++)
  {
    if (clGetDeviceInfo(devices[i], CL_DEVICE_TYPE, NULL, NULL, NULL) == _ofType)
    {
      return &devices[i];
    }
  }

  return NULL;
}