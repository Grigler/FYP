#include <iostream>
#include <chrono>

#include "RenderContext.h"

int main(int argc, char **argv)
{
  /*
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
  */

  RenderContext rc;

  rc.InitWindow(argc, argv, "Test", 300, 300, 1280, 720);
  rc.StartMainLoop();

  return 0;
}