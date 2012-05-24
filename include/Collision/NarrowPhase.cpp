#include "NarrowPhase.h"
#include "Objects/RBSolver.h"

using namespace Rigid2D;

// Perform root finding method until body a is no longer contained in b
// and are within some 𝜀 > 0 of touching.
//
// secant root finding method: t_{n+1} = t_n - d(t_n)*[(t_n - t_{n-1})/(d(t_n) - d(t_{n-1}))]
// t: simulation time
// d(t): distance between bodies a and b at time t.
Real toi(Contact const &contact, Real current_simulation_time) {
  epsilon = 0.001f;
  RigidBody *a = contact.a;
  RigidBody *b = contact.b;
  RBState state_a = a->getState();
  RBState state_a_prev = a->getPrevState();
  RBState state_b = b->getState();
  RBState state_b_prev = b->getPrevState();
  unsigned int va_index   = contact.va_index;   // index of contact vertex for body a
  unsigned int vb1_index  = contact.vb1_index;  // index for first vertex of contact edge for body b
  unsigned int vb2_index  = contact.vb2_index;  // index for second vertex of contact edge for body b
  Vector2 va = a->getVertex(va_index);
  Vector2 vb1 = b->getVertex(vb1_index);
  Vector2 vb2 = b->getVertex(vb2_index);
  Vector2 v;    // temp variable
  Real t_prev;  // t_{n-1}
  Real t;       // t_n
  Real d_prev;  // d(t_{n-1})
  Real d;       // d(t_n)
  Real t_tmp;

  t = current_simulation_time;

  // distance between a and b during current simulation time.
  d = distance(state_a, state_b);

  // transform vertex va from a's body space to world space.
  v = a->getVertex(va_index);
  v = a->toWorldSpace(v);
  va.x = v.x;
  va.y = v.y;

  d_prev = distance(state_a_prev, state_b_prev);
  t_prev = state_a_prev.getTime();

  while (d > epsilon || b->pointIsInterior(va.x, va.y)) {
    t_tmp = t;

    // Compute t_{n+1}
    t = t - d*((t - t_prev)/(d - d_prev));

    t_prev = t_tmp;

    // Determine RigidBody states at time t.
    state_a_prev = state_a;
    state_b_prev = state_b;

    RBSolver::nextStep(&a,         // rigid body
                       state_a,    // outState
                       t,          // start time, tIn
                       t-t_prev,   // time step, dt
                       NULL);      // tOut


    RBSolver::nextStep(&b,         // rigid body
                       state_b,    // outState
                       t,          // start time, tIn
                       t-t_prev,   // time step, dt
                       NULL);      // tOut

    a->setState(state_a);
    b->setState(state_b);

    // Update previous values
    d_prev = d;
    t_prev = state_a_prev.getTime();

    // Compute distance between a and b at new time.
    d = distance(state_a, state_b);

    // transform vertex va from a's body space to world space.
    v = a->getVertex(va_index);
    v = a->toWorldSpace(v);
    va.x = v.x;
    va.y = v.y;
  }

  return t;
}
