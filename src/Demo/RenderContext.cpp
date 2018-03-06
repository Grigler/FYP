#include "RenderContext.h"

#include "tiny_obj_loader.h"

#include <FYP.h>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

float RenderContext::deltaTime = 0.0f;

GLuint RenderContext::VAO;

GLuint RenderContext::bodyID;
int RenderContext::sphereVerts = 0;

GLuint RenderContext::programID;
GLuint RenderContext::VPID;

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

void RenderContext::InitVBO()
{
  glGenVertexArrays(1, &VAO);
  glBindVertexArray(VAO);

  glCreateBuffers(1, &bodyID);
  glBindBuffer(GL_ARRAY_BUFFER, bodyID);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Body), 0); //Position
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Body), (GLvoid*)(sizeof(GLfloat)*4)); //Orientation
  glVertexAttribDivisor(0, 1); //Sets to 1 body per draw instance
  glVertexAttribDivisor(1, 1);
  glEnableVertexAttribArray(0);
  glEnableVertexAttribArray(1);

  glBufferData(GL_ARRAY_BUFFER, sizeof(Body)*MAX_BODIES, NULL, GL_STATIC_DRAW);

  FYP::Pipeline::RegisterOutputVBOBuffer(bodyID);

  //Load sphere
  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapesVec;
  std::vector<tinyobj::material_t> matVec;
  std::string err;

  bool r = tinyobj::LoadObj(&attrib, &shapesVec, &matVec, &err, "../data/models/sphere.obj", NULL, true);

  std::vector<glm::vec3> vertData;
  std::vector<glm::vec3> normData;
  std::vector<glm::vec2> uvData;

  for (size_t s = 0; s < shapesVec.size(); s++)
  {
    size_t indexOffset = 0;
    //Each face in this object
    for (int f = 0; f < shapesVec.at(s).mesh.num_face_vertices.size(); f++)
    {
      int fv = shapesVec.at(s).mesh.num_face_vertices.at(f);
      for (size_t v = 0; v < fv; v++)
      {
        tinyobj::index_t i = shapesVec.at(s).mesh.indices.at(indexOffset + v);
        glm::vec3 vert;
        vert.x = attrib.vertices.at(3 * i.vertex_index + 0);
        vert.y = attrib.vertices.at(3 * i.vertex_index + 1);
        vert.z = attrib.vertices.at(3 * i.vertex_index + 2);
        vertData.push_back(vert);

        if (attrib.normals.size() > 0)
        {
          vert.x = attrib.normals.at(3 * i.normal_index + 0);
          vert.y = attrib.normals.at(3 * i.normal_index + 1);
          vert.z = attrib.normals.at(3 * i.normal_index + 2);
          normData.push_back(vert);
        }

        if (attrib.texcoords.size() > 0)
        {
          vert.x = attrib.texcoords.at(2 * i.texcoord_index + 0);
          vert.y = attrib.texcoords.at(2 * i.texcoord_index + 1);
          uvData.push_back(vert);
        }
      }
      indexOffset += fv;
    }
  }

  sphereVerts = vertData.size();

  GLuint vertVBO;
  glGenBuffers(1, &vertVBO);
  glBindBuffer(GL_ARRAY_BUFFER, vertVBO);
  glBufferData(GL_ARRAY_BUFFER, vertData.size() * sizeof(GLfloat) * 3, &vertData[0], GL_STATIC_DRAW);
  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0); //location 1 for shader
  glEnableVertexAttribArray(2);

}

void RenderContext::BuildShaders()
{
  //Program
  programID = glCreateProgram();
  //Vert
  std::string s = FYP::Util::ReadFromFile("../data/shaders/Simple.vert");
  GLchar *ccS = (GLchar*)s.c_str();
  GLuint id = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(id, 1, &ccS, NULL);
  glCompileShader(id);
  glAttachShader(programID, id);
  //Frag
  s = FYP::Util::ReadFromFile("../data/shaders/Simple.frag");
  ccS = (GLchar*)s.c_str();
  id = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(id, 1, &ccS, NULL);
  glCompileShader(id);
  glAttachShader(programID, id);
  //Link
  glLinkProgram(programID);

  VPID = glGetUniformLocation(programID, "VP");
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
  float fixedDelta = (t - lastT) / 1000.0f;
  deltaTime += fixedDelta;
  lastT = glutGet(GLUT_ELAPSED_TIME);

  //Pipeline handles fixed update - no logic needed
  FYP::Pipeline::Update(fixedDelta);

  //Setting vSync to ~60fps
  if (deltaTime >= 0.016f) //No vsync
  {
    //printf("Display\n");
    deltaTime = 0.0f;
    glutPostRedisplay();
  }
}

void RenderContext::Display()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  //Push position data into a given VBO
  FYP::Pipeline::CopyPosToVBOBuffer(bodyID);
  
  //Draw Position Data
  glUseProgram(programID);
  glBindVertexArray(VAO);

  glm::mat4 view = glm::lookAtRH(glm::vec3(0, 0, 10), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
  glm::mat4 proj = glm::perspective(glm::radians(45.0f), 1280.0f / 720.0f, 0.01f, 1000.0f);
  glm::mat4 VP = proj*view;

  glUniformMatrix4fv(VPID, 1, GL_FALSE, &VP[0][0]);

  glDrawArraysInstanced(GL_LINES, 0, sphereVerts, MAX_BODIES);

  glutSwapBuffers();
}