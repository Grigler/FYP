#include "Body.h"

using namespace FYP;

Body::Body()
{
  pos = glm::vec3(0);
  orientation = glm::quat();
  mass = 1.0f;

  radius = 1.0f;
  inertiaTensor = GenSphereInertiaTensor();

  linearVel = glm::vec3(0);
  angularVel = glm::vec3(0);

  if (true)
  {
    int r = rand() % 1000 - 500;
    bvMax = glm::vec3(r);
    bvMin = glm::vec3(r - 4);
  }
  else
  {
    bvMin = glm::vec3(-2);
    bvMax = glm::vec3(2);
  }  
}

glm::mat3 Body::GenSphereInertiaTensor()
{
  glm::mat3 it;
  float val = 0.4f * mass * (radius*radius);
  it[0][0] = val;
  it[1][1] = val;
  it[2][2] = val;
  return it;
}