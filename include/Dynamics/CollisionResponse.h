#ifndef RIGID2D_COLLISION_RESPONSE_H
#define RIGID2D_COLLISION_RESPONSE_H

#include "Collision/NarrowPhase.h"
#include "Common/RigidSettings.h"
#include "Objects/RigidBody.h"

namespace Rigid2D{

  enum ContactType { Resting, Colliding, Separating };

  // Given the contact argument representing an overlapping collision between
  // bodies a and b, resetStatesToTOI resets the current RBStates of the bodies
  // back in time to the momment of impact.  The momment of impact is some time
  // between the previous time frame and the current time frame such that
  // bodies a and b are non-intersecting and are within a squared distance 𝜀 > 0
  // from one another.
  Real resetStatesToTOI(Contact const &contact);

  void resolveCollision(Contact const &contact);
}

#endif
