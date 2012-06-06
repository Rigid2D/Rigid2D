#include "NarrowPhase.h"
#include <limits>
#include <iostream>

namespace Rigid2D {

  Vector2 findProjectionInterval(const RigidBody *rb, const Vector2 & normal, unsigned &leftIndex, unsigned &rightIndex)
  {
    leftIndex = rightIndex = 0;
    Vector2 interval;     // stores left and right most projected positions
    Real position;

    // initial values
    position = normal.dot(rb->getTransformedVertex(0));
    interval[0] = position;
    interval[1] = position;

    // project each vertex on given slope
    for (unsigned j = 1; j < rb->getNumVertices(); j++) {
      position = normal.dot(rb->getTransformedVertex(j));
      if (position < interval[0]) {
        interval[0] = position;
        leftIndex = j;
      } else if (position > interval[1]) {
        interval[1] = position;
        rightIndex = j;
      }
    }
    return interval;
  }

  void findContactInformation(Contact *contact)
  {
    RigidBody * a = contact->a;
    RigidBody * b = contact->b;
    Vector2 mtv = contact->mtv;

    // find vertex of a furthest in b
    Real max = -1 * std::numeric_limits<Real>::max();
    unsigned index;
    Real projection;
    for (unsigned i = 0; i < a->getNumVertices(); i++) {
      projection = a->getTransformedVertex(i).dot(1*mtv);
      if (projection > max) {
        max = projection;
        index = i;
      }
    }
    contact->va_index = index;

    // find edge of b being intersected (we are interested in the one most
    // perpendicular to the mtv)
    max = -1 * std::numeric_limits<Real>::max();
    for (unsigned i = 0; i < b->getNumVertices(); i++) {
      projection = b->getTransformedVertex(i).dot(-1*mtv);
      if (projection > max) {
        max = projection;
        index = i;
      }
    }
    Vector2 v = b->getTransformedVertex(index);
    unsigned l_index = (index + b->getNumVertices() - 1 ) % b->getNumVertices();
    unsigned r_index = (index + 1 ) % b->getNumVertices();
    Vector2 v0 = b->getTransformedVertex(l_index);
    Vector2 v1 = b->getTransformedVertex(r_index);
    Vector2 l = v - v0;
    Vector2 r = v - v1;
    if (r.dot(mtv) < l.dot(mtv)) {
      contact->vb1_index = l_index;
      contact->vb2_index = index;
    } else {
      contact->vb1_index = index;
      contact->vb2_index = r_index;
    }

  }

  /**
    Separating axis theorem - 
  */
  bool sat(RigidBody *rb1, RigidBody *rb2, Contact *contact, bool firstRB)
  {
    // extrema points for the projected intervals of each RB
    Vector2 rb1Interval, rb2Interval;
    unsigned rb1LeftIndex, rb1RightIndex, rb2LeftIndex, rb2RightIndex;
    // indices for the projections forming the MTV
    unsigned rb1MtvLeftIndex, rb1MtvRightIndex, rb2MtvLeftIndex, rb2MtvRightIndex;

    // MTV - minimum translation vector
    Vector2 mtv;
    Real mtv_magnitude;
    // The rb containing the edge for we derive the MTV from
    RigidBody *mtv_rb;

    // Rb to work with
    RigidBody * rb;

    // Update world position values for vertices of both bodies and select which rb to work with
    if (firstRB) {
      rb1->updateTransformedVertices();
      rb2->updateTransformedVertices();
      rb = rb1;
      mtv_magnitude = std::numeric_limits<Real>::max();
    } else {
      mtv = contact->mtv;
      mtv_magnitude = contact->mtv_magnitude;
      rb = rb2;
    }


    // SAT for edges of selected rb
    unsigned numVertices = rb->getNumVertices();
    for (unsigned i = 0; i < numVertices; i++) {
      // normal of current edge
      Vector2 normal = (rb->getTransformedVertex((i+1) % numVertices) - rb->getTransformedVertex(i)).perp();
      normal.normalize();

      // find projection intervals
      rb1Interval = findProjectionInterval(rb1, normal, rb1LeftIndex, rb1RightIndex);
      rb2Interval = findProjectionInterval(rb2, normal, rb2LeftIndex, rb2RightIndex);

      // if no intersection between intervals, we are done - no collision
      if (rb1Interval[1] < rb2Interval[0] ||
          rb1Interval[0] > rb2Interval[1]) {
        return false;
      }

      // store min intersection for MTV
      // TODO: optimize with above if statement and fix ugliness
      // Proper left intersection
      if (rb1Interval[1] > rb2Interval[0] &&
          rb1Interval[0] < rb2Interval[0]) {
        if (rb1Interval[1] - rb2Interval[0] < mtv_magnitude) {
          mtv_magnitude = rb1Interval[1] - rb2Interval[0];
          mtv = normal;
          mtv_rb = rb;
          rb1MtvLeftIndex = rb1LeftIndex;
          rb1MtvRightIndex = rb1RightIndex;
          rb2MtvLeftIndex = rb2LeftIndex;
          rb2MtvRightIndex = rb2RightIndex;
        }
      }
      // Proper right intersection
      else if (rb1Interval[1] > rb2Interval[1] &&
          rb1Interval[0] < rb2Interval[1])
      {
        if (rb2Interval[1] - rb1Interval[0] < mtv_magnitude) {
          mtv_magnitude = rb2Interval[1] - rb1Interval[0];
          mtv = normal;
          mtv_rb = rb;
          rb1MtvLeftIndex = rb1LeftIndex;
          rb1MtvRightIndex = rb1RightIndex;
          rb2MtvLeftIndex = rb2LeftIndex;
          rb2MtvRightIndex = rb2RightIndex;
        }
      }
      // first interval fully contains second
      else if ( rb1Interval[1] > rb2Interval[1] &&
          rb1Interval[0] < rb2Interval[0])
      {
        if (rb2Interval[1] - rb2Interval[0] < mtv_magnitude) {
          mtv_magnitude = rb2Interval[1] - rb2Interval[0];
          mtv = normal;
          mtv_rb = rb;
          rb1MtvLeftIndex = rb1LeftIndex;
          rb1MtvRightIndex = rb1RightIndex;
          rb2MtvLeftIndex = rb2LeftIndex;
          rb2MtvRightIndex = rb2RightIndex;
        }
      }
      // second interval fully contains first
      else {
        if (rb1Interval[1] - rb1Interval[0] < mtv_magnitude) {
          mtv_magnitude = rb2Interval[1] - rb1Interval[0];
          mtv = normal;
          mtv_rb = rb;
          rb1MtvLeftIndex = rb1LeftIndex;
          rb1MtvRightIndex = rb1RightIndex;
          rb2MtvLeftIndex = rb2LeftIndex;
          rb2MtvRightIndex = rb2RightIndex;
        }
      }
    }

    if (firstRB) {
      contact->mtv = mtv;
      contact->mtv_magnitude = mtv_magnitude;
      return sat(rb1, rb2, contact, false);
    } else {
      // fill out contact
      // The body which generates the MTV is the one with the colliding edge,
      // so we store it as contact->b.
      if (mtv_rb == rb2) {
        contact->a = rb1;
        contact->b = rb2;
      } else {
        contact->a = rb2;
        contact->b = rb1;
      }
      contact->mtv = mtv;
      contact->mtv_magnitude = mtv_magnitude;

      findContactInformation(contact);

      return true;
    }
  }
}
