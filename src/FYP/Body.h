#ifndef __FYP_BODY__
#define __FYP_BODY__

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

namespace FYP
{
  class Body
  {
  private:
    glm::mat3 GenSphereInertiaTensor();
  public:
    Body();

    glm::vec3 pos;
    glm::quat orientation;
    float mass;

    float radius;
    glm::mat3 inertiaTensor;

    glm::vec3 linearVel;
    glm::vec3 angularVel;

    glm::vec3 bvMin, bvMax;
  };
}

#endif