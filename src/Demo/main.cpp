#include <FYP.h>

#include <iostream>

int main()
{
  if(clewInit())
  {
    return 1;
  }

  FYP::Context context;
  context.Create(CL_DEVICE_TYPE_ALL);
 
  std::string test = FYP::Util::ReadFromFile("../kernel_code/test.txt");
  std::cout << test << std::endl;

  //Arbritrary key hold
  getchar();

  return 0;
}