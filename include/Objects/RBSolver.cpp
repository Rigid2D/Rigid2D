#include "RBSolver.h"

using namespace Rigid2D;

void RBSolver::nextStep(class RigidBody &rb, struct RBState &outState, Real stepSize) {
  struct RBState initialState, tempState, k1, k2, k3, k4;

  // Step 1
  rb.getState(initialState);
  rb.computeForces(initialState);
  rb.computeStateDeriv(initialState, k1);
  k1 *= stepSize;

  // Step 2
  tempState = initialState + k1;
  rb.computeForces(tempState);
  rb.computeStateDeriv(initialState + 0.5 * k1, k2);
  k2 *= stepSize;

  // Step 3
  tempState = initialState + k2;
  rb.computeForces(tempState);
  rb.computeStateDeriv(initialState + 0.5 * k2, k3);
  k3 *= stepSize;

  // Step 4
  tempState = initialState + k3;
  rb.computeForces(tempState);
  rb.computeStateDeriv(initialState + k3, k4);
  k4 *= stepSize;

  outState = initialState + (k1 + 2 * k2 + 2 * k3 + k4) / 6;
}
