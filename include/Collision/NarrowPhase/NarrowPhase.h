#ifndef RIGID2D_NARROW_PHASE_H
#define RIGID2D_NARROW_PHASE_H

#include "Common/RigidSettings.h"

namespace Rigid2D {

  // Structure for holding contact information between two rigid bodies once
  // collision is detected.
  struct Contact {
      RigidBody *a;	            // Reference body.  The msv is computed with respect to this body.
      RigidBody *b;	            // Incident body that body a is colliding with.
      Vector2 msv; 	            // Minimum separation vector, for translating a out of b.

      // Each Rigid Body contains a vertice list, and the following indices
      // allow us to keep track of specific vertices involved in the contact,
      // at various points in time.
      unsigned int va_index;    // Vertice index for vertex of a that is contained in b.
      unsigned int vb1_index; 	// Vertice index for first vertex that composes contact edge of b.
      unsigned int vb2_index; 	// Vertice index for second vertex that composes contact edge of b.

      // TODO: implement constructor, accessors, and mutators.
  };

  // Separating axis test
  bool sat(RigidBody *rb1, RigidBody *rb2, Contact &contact);

  // Returns the time of impact (toi) between the two rigid bodies referenced
  // by the given contact structure.  The referenced rigid bodies are assumed
  // to be currently overlapping.  The returned toi value is the time just
  // before now when the bodies were within some epsilon away from each other,
  // and were considered "touching".
  Real toi(Contact const &contact, Real current_system_time);

  // Shortest distance between the vertex va, and the edge that is composed of
  // the vertices vb1 and vb2.
  //
  // TODO: change to "pointToLineDist(...)" and move to folder include/Math?
  Real minDistance(Contact const &contact);

}

#endif
