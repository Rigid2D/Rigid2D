#include "CollisionResponse.h"
#include "Objects/RBSolver.h"

using namespace Rigid2D;

// Returns squared distance from contact vertex va of RigidBody a to contact
// edge (vb1, vb2) of Rigid Body b at specified time frame.  By default,
// time_frame is the current simulation frame, but can also be set to
// RBState::PREVIOUS, in order to compute distance based on previous simulation
// frame.  The argument reference va, will be updated with its location based
// on its world coordinates at given time_frame.
Real vertexEdgeDist(RigidBody *a, RigidBody *b, unsigned int va_index,
                    unsigned int vb1_index, unsigned int vb2_index,
                    Vector2 &va,
                    RBState::FrameSpecifier time_frame = RBState::CURRENT)
{
  assert( a != NULL);
  assert( b != NULL);

  Vector2 vb1 = b->getVertex(vb1_index),
          vb2 = b->getVertex(vb2_index);

  // transform vertex va from a's body space to world space.
  va = a->getVertex(va_index);
  va = a->localToWorldTransform(va, time_frame);

  // transform vertex vb1 from b's body space to world space.
  vb1 = b->localToWorldTransform(vb1, time_frame);

  // transform vertex vb2 from b's body space to world space.
  vb2 = b->localToWorldTransform(vb2, time_frame);

  // compute squared distance from vertex va to edge (vb1, vb2).
  return SqDistPointSegment(va, vb1, vb2);
}

// We let time t = 0 at previous time frame and t = 1 at current time frame.
// Our root finding method is used to determine a value of t in (0,1), which
// represents the fractional frame time starting from the previous frame, where
// body a is no longer contained in b and bodies are within squared distnace 𝜀
// > 0 of touching.
//
// secant root finding method: t_{n+1} = t_n - d(t_n)*[(t_n - t_{n-1})/(d(t_n) - d(t_{n-1}))]
// t: fractional simulation time since previous frame.
// d(t): distance between bodies a and b at time t.
//
// At each step the current RBStates of a and b are solved for using the
// RBSolver with initial state set to previous frame state and progressing
// forward with a step size equal to RBSolver::STEP_SIZE * t_n.  Once these new
// states are found, we compute the squared distance d(t_n) between Rigid Bodys
// a and b.
Real timeOfImpact(Contact const &contact) {
  RigidBody *a = contact.a,
            *b = contact.b;

  RBState state_tmp;

  Vector2 va;       // Contact vertex of Rigid Body a.

  unsigned int va_index   = contact.va_index,   // index of contact vertex for body a
               vb1_index  = contact.vb1_index,  // index for first vertex of contact edge for body b
               vb2_index  = contact.vb2_index;  // index for second vertex of contact edge for body b

  Real t_prev = 0.0f,  // t_{n-1}
       t      = 1.0f,  // t_n
       d_prev,         // d(t_{n-1})
       d,              // d(t_n)
       t_tmp,
       epsilon = 0.001f;

  a->getPrevState(state_tmp);
  const RBState state_a_prev_frame(state_tmp);  // store previous frame state, not modifiable.

  b->getPrevState(state_tmp);
  const RBState state_b_prev_frame(state_tmp);  // store previous frame state, not modifiable.

  // Compute d_prev, the distance between contact vertex and contact edge
  // during the previous time frame.
  d_prev = vertexEdgeDist(a, b, va_index, vb1_index, vb2_index, va, RBState::PREVIOUS);

  // Compute d, the distance between contact vertex and contact edge
  // during the current time frame.
  d = vertexEdgeDist(a, b, va_index, vb1_index, vb2_index, va);

  // Only need to compute one distance d per loop
  while (d > epsilon || b->pointIsInterior(va.x, va.y)) {
    t_tmp = t;

    // Compute next t value using secant method.
    t = t - d * ((t - t_prev)/(d - d_prev));

    t_prev = t_tmp;
    d_prev = d;

    // Update the current state of a and b by starting from the previous frame
    // state and advancing the ODESolver by a step size of RBSolver::STEP_SIZE * t.
    state_tmp = state_a_prev_frame;
    a->setState(state_tmp);
    RBSolver::nextStep(*a, state_tmp, RBSolver::STEP_SIZE * t);
    a->setState(state_tmp);

    state_tmp = state_b_prev_frame;
    b->setState(state_tmp);
    RBSolver::nextStep(*b, state_tmp, RBSolver::STEP_SIZE * t);
    b->setState(state_tmp);

    d = vertexEdgeDist(a, b, va_index, vb1_index, vb2_index, va);
  }

  return t;
}
