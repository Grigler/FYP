#include <FYP.h>

#include <iostream>
#include <chrono>

int main()
{
  FYP::Pipeline::Init();

  bool isRunning = true;

  std::chrono::steady_clock cl;

  while (isRunning)
  {
    auto t0 = cl.now();
    
    FYP::Pipeline::Update(0.008f);

    auto t1 = cl.now();
    std::cout << "T: " <<
      std::chrono::duration_cast<std::chrono::milliseconds>((t1 - t0)).count() <<
      " ms" << std::endl;

    //Super basic sleeping
    _sleep(8);
  }

  //Arbritrary key hold
  getchar();

  return 0;
}