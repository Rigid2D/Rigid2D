#ifndef RIGID2D_RBSOLVER_H
#define RIGID2D_RBSOLVER_H

#include "Common/RigidSettings.h"
#include "RigidBody.h"

namespace Rigid2D
{
  /// Standard RungeKutta4 implementation
  class RBSolver
  {
    public:
			static const Real STEP_SIZE;

      //// NOTE: time is not being passed currently, but it's trivial
      //// to add if needed.
      static void nextStep(class RigidBody &rb, struct RBState &outState, Real stepSize = STEP_SIZE);
  };

}

#endif
