#include "RenderContext.h"

#include <FYP.h>

float RenderContext::deltaTime = 0.0f;

void RenderContext::InitWindow(int _argc, char **_argv, 
  const char *_name, int _x, int _y, int _w, int _h)
{
  //Init and window creation
  glutInit(&_argc, _argv);
  glutInitWindowSize(_w, _h);
  glutInitWindowPosition(_x, _y);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
  glutInitContextVersion(4, 5);
  glutInitContextFlags(GLUT_CORE_PROFILE);
  glutCreateWindow(_name);
  glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

  //glutCallbacks
  glutIdleFunc(Idle);
  glutDisplayFunc(Display);

  //glew stuff and backface culling params
  glewExperimental = GL_TRUE;
  glewInit();
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glClearColor(0.f, 0.f, 0.f, 1.f);

  //Init pipeline
  FYP::Pipeline::Init();
}

void RenderContext::StartMainLoop()
{
  printf("> Entering Loop\n");
  glutMainLoop();
  printf("> Exiting Loop\n");
}

void RenderContext::Idle()
{
  //Timer updating
  static float lastT = glutGet(GLUT_ELAPSED_TIME); //only run 1st time
  float t = glutGet(GLUT_ELAPSED_TIME);
  deltaTime += (t - lastT) / 1000.0f;
  lastT = glutGet(GLUT_ELAPSED_TIME);

  //Pipeline handles fixed update - no logic needed
  FYP::Pipeline::Update(deltaTime);

  //Setting vSync to ~60fps
  if (deltaTime >= 0.016f)
  {
    printf("Display\n");
    deltaTime = 0.0f;
    glutPostRedisplay();
  }
}

void RenderContext::Display()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //Push position data into a given VBO

  //Draw Position Data

  glutSwapBuffers();
}