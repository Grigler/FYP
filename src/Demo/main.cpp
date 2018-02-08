#include <iostream>
#include <chrono>

#include "RenderContext.h"

int main(int argc, char **argv)
{
  RenderContext rc;

  rc.InitWindow(argc, argv, "Test", 300, 300, 1280, 720);

  rc.InitVBO();
  rc.BuildShaders();

  rc.StartMainLoop();

  return 0;
}