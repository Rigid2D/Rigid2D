#include "NarrowPhase.h"
#include <limits>
#include <iostream>

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

  void findContactInformation(Contact *contact)
  {
    RigidBody * a = contact->a;
    RigidBody * b = contact->b;
    Vector2 mtv = contact->mtv;
    //mtv.normalize();

    // find vertex of a furthest in b
    Real max = -1 * std::numeric_limits<Real>::max();
    unsigned index;
    Real projection;
    for (unsigned i = 0; i < a->getNumVertices(); i++) {

      std::cout << "curindex " << index << std::endl;
      projection = a->getTransformedVertex(i).dot(1*mtv);
      std::cout << "proj: " << projection << std::endl;
      std::cout << "max: " << max << std::endl;
      if (projection > max) {
        max = projection;
        index = i;
      }
    }
    std::cout << index << std::endl;
    contact->va_index = index;

    // find edge of b being intersected (we are interested in the one most
    // perpendicular to the mtv)
    max = -1 * std::numeric_limits<Real>::min();
    for (unsigned i = 0; i < b->getNumVertices(); i++) {
      projection = b->getTransformedVertex(i).dot(-1*mtv);
      if (projection > max) {
        max = projection;
        index = i;
      }
    }
    Vector2 v = b->getVertex(index);
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

  bool sat(RigidBody *rb1, RigidBody *rb2, Contact *contact, bool firstRB)
  {
    // TODO: add explanation

    // extrema points for the projected intervals of each RB
    Vector2 intervalRB1, intervalRB2;
    Vector2 min_interval;                // ends up being direction of MTV
    Real min_overlap = std::numeric_limits<Real>::max();   // ends up being magnitude of MTV
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
      findContactInformation(contact);

      return true;
    }
  }
}
