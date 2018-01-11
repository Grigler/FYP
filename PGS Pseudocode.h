
struct RB
{
  //General RigidBody implementation
  void ImpulseForce(float _force);
}

struct Contact
{
  float accumulatedImpulse;
  
  // J = [n | r x n] = [xyz | xyz]
  vec3 jNorm;
  vec3 jArm;
  
  //J * M^-1 * J^T
  vec3 jmjLeft;
  vec3 jmjRight;
  
  //M = m identity matrix
  mat3 massMat;
  mat3 inertiaTensor;
  
  RB *lhs;
  RB *rhs;
  
};

void SingleIteration()
{
  Contact *contacts;
  int contactNum;
  
  for(size_t i = 0; i < contactNum; i++)
  {
    Contact *c = contacts[i];
    
    float startingImpulse = c->accumulatedImpulse;
    
    //m = J * M^-1 * J^T
    
    //M^-1 * J^T  as block matrix components
    vec3 mInvJTransMass = c->massMat * c->jNorm;
    vec3 mInvJTransInertia = c->inertiaTensor * c->jArm;
    
    //Calculating delta impulse and accumulating
    c->accumulatedImpulse = startingImpulse - (1/m)*(J*V);
    //Ensuring force is positive - projected part of PGS
    c->accumulatedImpulse = max(0, c->accumulatedImpulse);
    
    //Applying force for this contact at this iteration
    c->lhs->applyImpulseForce(deltaImpulse);
    c->rhs->applyImpulseForce(deltaImpulse);
  }
  
}

void PGS()
{
  Constraints *constraints[];
  int constraintNum;
  for(int i = 0; i < constraintNum; i++)
  {
    vec3 jmjLeft;
    //Mass along identity
    mat3 massLeft = constrains->lhs->mass;
    massLeft = inverse(massLeft);
    jmjLeft = constraints[i]->jNorm * massLeft * transpose(constrainst[i]->jNorm);
    
    vec3 jmjRight;
    //Mass along identity
    mat3 massRight = constraints[i]->rhs->mass;
    massRight = inverse(massRight);
    jmjRight = constrains[i]->jArm * massRight * transpose(constraints[i]->jArm);
    
    
    contacts[i]->jmjLeft = jmjLeft;
    contacts[i]->jmjRight = jmjRight;
  }
  
  int iterationMax = 4;
  for(int i = 0; i < itertionMax; i++)
  {
    SingleIteration();
  }
  
}