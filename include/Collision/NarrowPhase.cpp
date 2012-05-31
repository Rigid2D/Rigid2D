#include "NarrowPhase.h"

namespace Rigid2D {

  Vector2 findProjectionInterval(const RigidBody *rb, const Vector2 & normal)
  {
    Vector2 interval;     // stores left and right most projected positions
    Real position;

    // project each vertex on given slope
    for (unsigned j = 0; j < rb->getNumVertices(); j++) {
      position = normal.dot(rb->getTransformedVertex(j));
      if (j == 0) {              // set inital values, can we eliminate this?
        interval[0] = position;
        interval[1] = position;
      } else {                  // update min/max interval values
        if (position < interval[0]) {
          interval[0] = position;
        } else if (position > interval[1]) {
          interval[1] = position;
        }
      }
    }
    return interval;
  }

  /*void findContactInformation(Contact *contact)
  {
  }*/

  bool sat(RigidBody *rb1, RigidBody *rb2, Contact *contact, bool firstRB)
  {
    // TODO: add explanation

    // extrema points for the projected intervals of each RB
    Vector2 intervalRB1, intervalRB2;
    Vector2 min_interval;         // ends up being direction of MTV
    Real min_overlap = FLT_MAX;   // ends up being magnitude of MTV
    RigidBody const * rb;                // the body which has it's vertices projected

    if (firstRB) {
      rb1->updateTransformedVertices();
      rb2->updateTransformedVertices();
      rb = rb1;
    } else {
      rb = rb2;
    }

    // SAT for edges of this RB
    for (unsigned i = 0; i < rb->getNumVertices(); i++) {
      Vector2 normal = (rb->getTransformedVertex((i+1) % rb->getNumVertices()) - rb->getTransformedVertex(i)).perp();
      normal.normalize();

      intervalRB1 = findProjectionInterval(rb1, normal);
      intervalRB2 = findProjectionInterval(rb2, normal);
      // if no intersection, we are done
      if (intervalRB1[1] < intervalRB2[0] ||
          intervalRB1[0] > intervalRB2[1]) {
        return false;
      }
      // store min intersection for MTV
      // TODO: optimize with above if statement and fix ugliness
      if (intervalRB1[1] < intervalRB2[1] &&
          intervalRB1[0] < intervalRB2[0]) {
        if (intervalRB1[1] - intervalRB2[0] < min_overlap) {
          min_overlap = intervalRB1[1] - intervalRB2[0];
          min_interval = normal;
        }
      }
      else if ( intervalRB1[1] > intervalRB2[1] &&
          intervalRB1[0] > intervalRB2[0])
      {
        if (intervalRB2[1] - intervalRB1[0] < min_overlap) {
          min_overlap = intervalRB2[1] - intervalRB1[0];
          min_interval = normal;
        }
      }
      else if ( intervalRB1[1] > intervalRB2[1] &&
          intervalRB1[0] < intervalRB2[0])
      {
        if (intervalRB2[1] - intervalRB2[0] < min_overlap) {
          min_overlap = intervalRB2[1] - intervalRB2[0];
          min_interval = normal;
        }
      }
      else {
        if (intervalRB1[1] - intervalRB1[0] < min_overlap) {
          min_overlap = intervalRB2[1] - intervalRB1[0];
          min_interval = normal;
        }
      }
    }

    if (firstRB) {
      return sat(rb1, rb2, contact, false);
    } else {
      // fill out contact
      // NOTE: currently the passed in contact is rb1's contact_.
      contact->a = rb1;
      contact->b = rb2;
      contact->mtv = min_overlap * min_interval;

      return true;
    }
  }
}
