#ifndef __FYP_CONTEXT__
#define __FYP_CONTEXT__

#include "clew/clew.h"

namespace fyp
{
  class Context
  {
  public:
    void Create(cl_device_type _devices);

    cl_context *GetId() { return context; }

  private:
    cl_platform_id platform;

    cl_device_id devices[10];
    cl_uint numDevices;
    
    cl_context *context;
    cl_command_queue commandQueue[2];
  };
}

#endif