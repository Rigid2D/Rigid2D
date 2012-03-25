#ifndef RIGID2D_RBSOLVER_H
#define RIGID2D_RBSOLVER_H

#include "Common/RigidSettings.h"
#include "RigidBody.h"

#include <iostream>

namespace Rigid2D
{
  /// Standard RungeKutta4 implementation
  class RBSolver
  {
    public:
      //// NOTE: time is not being passed currently, but it's trivial
      //// to add if needed.
      static void nextStep(class RigidBody &rb, struct RBState &outState, Real stepSize = 0.01)
      {
        struct RBState inState, k1, k2, k3, k4;

        // Step 1
        rb.getState(inState);
        rb.computeForces(inState);
        rb.computeStateDeriv(inState, k1);
        k1 *= stepSize;

        // Step 2
        rb.computeForces(k1);
        rb.computeStateDeriv(0.5 * k1, k2);
        k2 *= stepSize;

        // Step 3
        rb.computeForces(k2);
        rb.computeStateDeriv(0.5 * k2, k3);
        k3 *= stepSize;

        // Step 4
        rb.computeForces(k3);
        rb.computeStateDeriv(k3, k4);
        k4 *= stepSize;

        outState = inState + (k1 + 2 * k2 + 2 * k3 + k4) / 6;
      }
  };
}

#endif
