#include "CollisionResponse.h"
#include "Objects/RBSolver.h"

#include <iostream>

namespace Rigid2D {

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

    // Now that vertices va, vb1, and vb2 are all given in world space
    // coordinates, compute squared distance from vertex va to edge (vb1, vb2).
    return SqDistPointSegment(va, vb1, vb2);
  }

  // We let time t = 0 at previous time frame and t = 1 at current time frame.
  // Our root finding method is used to determine a value of t in (0,1), which
  // represents the fractional frame time starting from the previous frame, where
  // body a is no longer contained in b and bodies are within squared distnace 𝜀 > 0
  // of touching.
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
  void resetStatesToTOI(Contact const &contact) {
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

  std::cout << "d_prev: " << d_prev << "\n";

  // Compute d, the distance between contact vertex and contact edge
  // during the current time frame.
  d = vertexEdgeDist(a, b, va_index, vb1_index, vb2_index, va);

  while (d > epsilon || b->pointIsInterior(va.x, va.y)) {
    t_tmp = t;
    std::cout << "d: " << d << "\n";
    std::cout << "t: " << t << "\n";
    std::cout << "a.x: " << a->getPosition().x << "\n";

    // Compute next t value using secant method.
    std::cout << "v: " << (t - t_prev)/(d - d_prev) << "\n";
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

  // Current RBStates of a and b are now at the TOI, such that they are non-intersecting
  // and are a squared distance 𝜀 > 0 apart.
  }

  Contact::Type getContactType(Contact &contact) {
    RigidBody *a = contact.a;
    RigidBody *b = contact.b;

    Vector2 pb;  // contact point on body b, lies somwhere on the edge (vb1, vb2).
    Vector2 va;  // contact point on body a, which is a's contact vertex.

    va = a->getTransformedVertex(contact.va_index);

    Vector2 vb1 = b->getTransformedVertex(contact.vb1_index);

    Vector2 vb2 = b->getTransformedVertex(contact.vb2_index);

    // Determine contact point on body b in world coordinates.
    ClosestPtPointSegment(vb1, vb2, va, pb);
    std::cout << "closestpt -- " << pb.x << "    "  << pb.y << "      \n";
    pb = b->worldToLocalTransform(pb);  // Change to local coordinates.
    contact.pb = pb;  // Save contact point for easy look up later.

    // Compute velocity of b's contact point.
    Vector2 pb_velocity = b->pointVelocity(pb);

    // Compute velocity of a's contact vertex va.
    Vector2 va_velocity = a->pointVelocity(va);

    // Find outward normal of b's contact edge.
    Vector2 n = vb2 - vb1;
    n = n.perp();
    contact.n = n;  // Save contact edge normal for easy look up later.

    // n should be pointing away from edge(vb1, vb2) and towards a's contact
    // vertex va.
    if (n.dot(vb1) < 0.0f)
      n *= -1; // flip direction of n

    // Compute relative velocity
    Real v_rel = n.dot(va_velocity - pb_velocity);
    Real threshold = 0.0001f;

    contact.v_rel = v_rel;

    if (v_rel > threshold)
      // bodies are moving away from one another
      return Contact::Separating;
    else if (v_rel > -threshold)
      // bodies are neither moving toward nor away
      return Contact::Resting;
    else
      // bodies are moving toward one another
      return Contact::Colliding;
  }

  void resolveCollision(Contact &contact) {
    // Determine type of contact.
    Contact::Type type = getContactType(contact);
    if (true) {///(type == Contact::Colliding){
      RigidBody *a = contact.a;
      RigidBody *b = contact.b;

      // Do steps to compute impulse j

      // Vector from a's center of mass to a's contact vertex.
      Vector2 ra = a->getTransformedVertex(contact.va_index) - a->getPosition();

      // Vector from b's center of mass to b's contact point.
      Vector2 rb = contact.pb - b->getPosition();

      Vector2 n = contact.n;

      Real epsilon = (a->getRestitution() + b->getRestitution()) / 2;

      Real numerator = -(1 + epsilon) * contact.v_rel;

      Real term1 = a->getInvMass(),
           term2 = b->getInvMass(),
           term3 = n.dot(a->getInvMoi() * tripleCrossProduct(ra, n, ra)),
           term4 = n.dot(b->getInvMoi() * tripleCrossProduct(rb, n, rb));

      // Now compute impulse
      Real j = numerator / (term1 + term2 + term3 + term4);

      Vector2 force = j * n;
      std::cout << "numerator  " << numerator << "\n";
      std::cout << "term1 " << term1 << "\n";
      std::cout << "term2 " << term2 << "\n";
      std::cout << "term3 " << term3 << "\n";
      std::cout << "term4 " << term4 << "\n";
      std::cout << "force.x " << force[0] << "\n";

      RBState state;

      // Apply impulse to body a.
      a->getPrevState(state);
      state.linearMomentum -= force;
      state.angularMomentum -= ra.cross(force);
      a->setState(state);

      // Apply impulse to body b.
      b->getPrevState(state);
      state.linearMomentum += force;
      state.angularMomentum += rb.cross(force);
      b->setState(state);
    }
  }
}
