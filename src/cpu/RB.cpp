#include "RB.h"

#include <stdio.h>

RB::RB()
{
  pos = glm::vec3(0);
  rot = glm::quat();
  mass = 10.0f;

  shape.CalcTensor(mass, 10.0f);
  glm::vec3 centMass = glm::vec3(0, 0, 0);
}

void RB::ApplyForceAtLocation(glm::vec3 _f, glm::vec3 _l)
{
  force += _f;
  
  //Torque calculation
  glm::vec3 localFLocation = centMass - _l;
  torque += glm::cross(localFLocation, _f);
}
//Uses semi-implicit Euler integration for speed and
//increased stability over Explicit/Implicit Euler
void RB::Integrate(float _dts)
{
  //------------Updating Velocities---------------------

  //Updating linear velocity from linear acceleration
  glm::vec3 linearAccel = force / mass; //F = m*a
  linearVel = linearAccel * _dts;      //v = a*dt

  //Updating angular velocity from angular acceleration
  //rotating local space inertia tensor
  glm::mat3 R = glm::toMat3(glm::normalize(rot));
  glm::mat3 Iinv = R*glm::inverse(shape.inertiaTensor)*glm::transpose(R);
  angularVel = Iinv*torque;

  //----------------------------------------------------


  //------------Transforming-------------------------

  //Transforming position by linearVelocity
  pos += linearVel * _dts;

  //Forming new rotation matrix from angularVel
  glm::quat q = glm::quat(angularVel)*rot;
  rot = q;

  //-------------------------------------------------
}