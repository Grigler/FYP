#ifndef __FYP_CONTEXT__
#define __FYP_CONTEXT__

#include <clew/clew.h>

namespace FYP
{
#define CL_DEVICE_MAX 1

  class Context
  {
  public:
    //Creates global context and a command queue for each device that is found
    void Create(cl_device_type _devices);

    static cl_context *GetId() { return context; }

    //Returns ptr to the first instance of paramater type device
    //CL_DEVICE_TYPE_ALL by default
    cl_device_id *GetFirstDevice(cl_device_type _ofType = CL_DEVICE_TYPE_ALL);

  //private:
    cl_int errCode;

    cl_platform_id platform;

    cl_device_id devices[CL_DEVICE_MAX];
    cl_uint numDevices;
    
    static cl_context *context;
    cl_command_queue commandQueue[CL_DEVICE_MAX];
  };
}

#endif