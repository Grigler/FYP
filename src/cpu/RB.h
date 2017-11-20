#ifndef __CPU_RB__
#define __CPU_RB__

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

struct Shape
{
  //Shape BV here
  glm::mat3 inertiaTensor;

  virtual void CalcTensor(float _mass) {}
};

struct Cube : public Shape
{
  void CalcTensor(float _mass, float _size)
  {
    inertiaTensor[0][0] = (_mass / 12.0f)*(_size*_size);
    inertiaTensor[1][1] = (_mass / 12.0f)*(_size*_size);
    inertiaTensor[2][2] = (_mass / 12.0f)*(_size*_size);
  }
};

class RB
{
public:
  RB();

  glm::vec3 pos;
  //glm::mat3 rot;
  glm::quat rot;

  glm::vec3 GetLinearVel() { return linearVel; }
  glm::vec3 GetAngularVel() { return angularVel; }

  void ApplyForceAtLocation(glm::vec3 _f, glm::vec3 _l);

  void Integrate(float _dts);

private:
  glm::vec3 force;
  glm::vec3 torque;

  glm::vec3 linearVel;
  glm::vec3 angularVel;

  glm::vec3 centMass;
  float mass;

  Cube shape;
};

#endif