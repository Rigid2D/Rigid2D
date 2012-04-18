#ifndef FORCE_FUNCTIONS_H
#define FORCE_FUNCTIONS_H
#include "Rigid2D.h"

using namespace Rigid2D;

void mouseSpringForce(RigidBody * const rigidBody, RBState * state, Vector2 * forceDst, Real * torqueDst, void * userData);

void gravity(RigidBody * const rigidBody, RBState * state, Vector2 * forceDst, Real *, void *);

#endif
