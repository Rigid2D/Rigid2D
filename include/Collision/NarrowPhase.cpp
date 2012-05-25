#include "NarrowPhase.h"
#include "Objects/RBSolver.h"
#include "float.h" // for FLT_MAX

using namespace Rigid2D;

// We let time t = 0 at previous frame and t = 1 at current frame.  Our root
// finding method is used to determine a value of t in (0,1), which represents
// the fractional frame time starting from the previous frame, where body a is
// no longer contained in b and bodies are within squared distnace 𝜀 > 0 of
// touching.
//
// secant root finding method: t_{n+1} = t_n - d(t_n)*[(t_n - t_{n-1})/(d(t_n) - d(t_{n-1}))]
// t: fractional simulation time since previous frame.
// d(t): distance between bodies a and b at time t.
Real toi(Contact const &contact) {
  RigidBody *a = contact.a,
            *b = contact.b;

  RBState state_a = a->getState(),  // current state of a, to be modified
          state_b = b->getState(),  // current state of b, to be modified
          state_tmp;

  RBState const state_a_prev_frame = a->getPrevState(),
          const state_b_prev_frame = b->getPrevState();

  unsigned int va_index   = contact.va_index,   // index of contact vertex for body a
               vb1_index  = contact.vb1_index,  // index for first vertex of contact edge for body b
               vb2_index  = contact.vb2_index;  // index for second vertex of contact edge for body b

  Vector2 va  = a->getVertex(va_index),
          vb1 = b->getVertex(vb1_index),
          vb2 = b->getVertex(vb2_index),
          v;    // temp variable

  Real t_prev = 0.0f,  // t_{n-1}
       t      = 1.0f,  // t_n
       d_prev,         // d(t_{n-1})
       d,              // d(t_n)
       t_tmp,
       epsilon = 0.001f;

  // Compute d, the distance between contact vertex and contact edge
  // during the current frame.

  // transform vertex va from a's body space to world space.
  v = state_a->getVertex(va_index);
  va = state_a->localToWorldTransform(v);

  // transform vertex vb1 from b's body space to world space.
  v = state_b->getVertex(vb1_index);
  vb1 = state_b->localToWorldTransform(v);

  // transform vertex vb2 from b's body space to world space.
  v = state_b->getVertex(vb2_index);
  vb2 = state_b->localToWorldTransform(v);

  // compute squared distance from vertex va to edge (vb1, vb2).
  d = SqDistPointSegment(va, vb1, vb2);


  // Compute d_prev, the distance between contact vertex and contact edge
  // during the previous frame.

  // transform vertex va from a's body space to world space.
  v = state_a_prev->getVertex(va_index);
  va = state_a_prev->localToWorldTransform(v);

  // transform vertex vb1 from b's body space to world space.
  v = state_b_prev->getVertex(vb1_index);
  vb1 = state_b_prev->localToWorldTransform(v);

  // transform vertex vb2 from b's body space to world space.
  v = state_b_prev->getVertex(vb2_index);
  vb2 = state_b_prev->localToWorldTransform(v);

  // compute squared distance from vertex va to edge (vb1, vb2).
  d_prev = SqDistPointSegment(va, vb1, vb2);

  // Only need to compute one distance d per loop
  while (d > epsilon || b->pointIsInterior(va.x, va.y)) {
    t_tmp = t;

    // Compute next t value using secant method.
    t = t - d*((t - t_prev)/(d - d_prev));

    t_prev = t_tmp;
    d_prev = d;

    // Update Rigid Body a by starting from the state state_a_prev_frame and advancing
    // the ODESolver by the step size RBSolver::step_size * t.


  }

  return t;
}
