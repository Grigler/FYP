#ifndef __DEMO_RENDER_CONTEXT__
#define __DEMO_RENDER_CONTEXT__

#include <GL/glew.h>
#include <GL/freeglut.h>

class RenderContext
{
public:
  void InitWindow(int _argc, char **_argv,
    const char *_name, int _x, int _y, int _w, int _h);
  void ShutDown();

  void InitVBO();

  void BuildShaders();

  void StartMainLoop();

private:
  static float deltaTime;

  static GLuint sphereVAO;
  static int sphereVerts;

  static GLuint floorVAO;
  static int floorVerts;

  static GLuint rbID;

  static GLuint programID;
  static GLuint VPID;
  static GLuint colID;

  static void Idle();
  static void Display();
  static void Keyboard(unsigned char _key, int _x, int _y);
};

#endif