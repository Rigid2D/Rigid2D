#include "ForceFunctions.h"
#include <cassert>

#include <iostream>
/**
 * Using the following damped spring force equation:
 * f = -[ks*norm(l) + kd*(l_dot*l)/norm(l)]*l/norm(l)
 *
 * f: vector representing force applied to center of mass of RigidBody
 *
 * l: vector representing distance between center of mass of RigidBody and mouse
 * position
 *
 * l_dot: vector representing difference between velocity of center of mass of
 * RigidBody and velocity of mouse.  Assumes mouse is stationary for the
 * purposes of computing force.
 *
 * norm(l): length of vector l
 *
 * ks: spring force constant
 *
 * kd: damping constant
 *
*/
void mouseSpringForce(RigidBody * const rigidBody, RBState * state, Vector2 * forceDst, Real * torqueDst, void * userData){
  assert(rigidBody != NULL);
  assert(forceDst != NULL);
  assert(torqueDst != NULL);
  assert(userData != NULL);

  Vector2 mousePos ( ((Real*)userData)[0], ((Real*)userData)[1] );  // Store mouse coordinants
  Vector2 centerOfMassPos = state->position;                        // Center of mass for RigidBody
  Vector2 mouseClickPos(((Real*)userData)[2], ((Real*)userData)[3]);    // Where the spring is attached to on RB
  Real ks = ((Real*)userData)[4];                                   // Spring constant
  Real kd = ((Real*)userData)[5];                                   // Damping constant

  // Compute Force acting on center of mass:

  // l; Distance between the RigidBody's center of mass, and mouse coordinates
  Vector2 deltaPosition(mouseClickPos.x - mousePos.x,
      mouseClickPos.y - mousePos.y);

  if (feq(deltaPosition.getLength(), 0.0)){
    forceDst->x = 0.0;
    forceDst->y = 0.0;
  }
  else {
    // l_dot; Assume mouse is not moving, and only factor-in velocity of RigidBody's center of mass.
    Vector2 deltaVelocity(state->linearMomentum / rigidBody->getMass());

    // kdFactor = (l_dot * l)/ norm(l)^2
    Real kdFactor = deltaVelocity.dot(deltaPosition) / deltaPosition.getLengthSquared();

    // Store computed force values
    forceDst->x = (ks + kd*kdFactor)*(-deltaPosition.x);  // x component of force
    forceDst->y = (ks + kd*kdFactor)*(-deltaPosition.y);  // y component of force

    // Compute Torque:
    *torqueDst = (mouseClickPos - centerOfMassPos).cross(*forceDst);
  }



}
