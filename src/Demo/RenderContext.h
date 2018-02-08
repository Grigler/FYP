#ifndef __DEMO_RENDER_CONTEXT__
#define __DEMO_RENDER_CONTEXT__

#include <GL/glew.h>
#include <GL/freeglut.h>

class RenderContext
{
public:
  void InitWindow(int _argc, char **_argv,
    const char *_name, int _x, int _y, int _w, int _h);

  void StartMainLoop();

private:
  static float deltaTime;

  static void Idle();
  static void Display();

};

#endif