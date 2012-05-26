#ifndef RIGID2D_COLLISION_RESPONSE_H
#define RIGID2D_COLLISION_RESPONSE_H

#include "Collision/NarrowPhase.h"
#include "Common/RigidSettings.h"
#include "Objects/RigidBody.h"

namespace Rigid2D{

  // Returns a value in (0,1) marking the fraction of frame time since the
  // previous frame where bodies a and b are non-intersecting and are within a
  // squared distance 𝜀 > 0 from one another.
  Real timeOfImpact(Contact const &contact);

  void resolveCollision(Contact const &contact);
}

#endif
