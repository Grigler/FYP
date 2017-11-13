#include "Program.h"

#include "Context.h"
#include "util.h"

using namespace FYP;

void Program::CreateFromPath(std::string _path)
{
  std::string text = Util::ReadFromFile(_path.c_str());

  cl_int err;
  clCreateProgramWithSource(*Context::GetId(), 1,
    (const char **)text.c_str(), (size_t*)text.size(), &err);
  

}