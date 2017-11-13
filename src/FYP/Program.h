#ifndef __FYP_PROGRAM__
#define __FYP_PROGRAM__

#include <clew/clew.h>

#include <string>

namespace FYP
{
  class Program
  {
  public:
    void CreateFromPath(std::string _path);

  private:
    cl_program program;

  };
}

#endif