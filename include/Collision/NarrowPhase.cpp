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
  RigidBody *a = contact.a,
            *b = contact.b;

  RBState state_a      = a->getState(),
          state_a_prev = a->getPrevState(),
          state_b      = b->getState(),
          state_b_prev = b->getPrevState();

  unsigned int va_index   = contact.va_index,   // index of contact vertex for body a
               vb1_index  = contact.vb1_index,  // index for first vertex of contact edge for body b
               vb2_index  = contact.vb2_index;  // index for second vertex of contact edge for body b

  Vector2 va  = a->getVertex(va_index),
          vb1 = b->getVertex(vb1_index),
          vb2 = b->getVertex(vb2_index),
          v;    // temp variable

  Real t_prev,  // t_{n-1}
       t,       // t_n
       d_prev,  // d(t_{n-1})
       d,       // d(t_n)
       t_tmp,
       epsilon = 0.001f;

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
