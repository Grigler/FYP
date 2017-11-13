#ifndef __FYP_UTIL__
#define __FYP_UTIL__

#include <iostream>
#include <sstream>
#include <fstream>

namespace FYP
{
namespace Util
{ 

  //Returns string containing txt from _path or NULL on failure
  std::string ReadFromFile(const char *_path)
  {
    std::ifstream file(_path);
    if (file.is_open())
    {
      std::stringstream ss;
      ss << file.rdbuf();
      file.close();

      return std::string(ss.str());
    }
    else
    {
      std::cout << "File " << _path << " failed to open" << std::endl;
      return NULL;
    }
  }

}
}

#endif