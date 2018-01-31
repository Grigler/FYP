#include <FYP.h>

#include <iostream>

int main()
{
  /*
  if(clewInit())
  {
    return 1;
  }

  FYP::Context context;
  context.Create(CL_DEVICE_TYPE_ALL);
 
  std::string test = FYP::Util::ReadFromFile("../kernel_code/test.txt");
  std::cout << test << std::endl;
  */

  FYP::Pipeline::Init();

  bool isRunning = true;

  while (isRunning)
  {
    FYP::Pipeline::Update(0.008f);
    printf("Post-Update print\n");
    _sleep(8);
  }

  //Arbritrary key hold
  getchar();

  return 0;
}