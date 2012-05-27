#ifndef RIGID2D_NARROW_PHASE_H
#define RIGID2D_NARROW_PHASE_H

#include "Common/RigidSettings.h"
#include "Objects/RigidBody.h"

namespace Rigid2D {

  // Structure for holding contact information between two rigid bodies once
  // collision is detected.
  struct Contact {
      RigidBody *a;	            // Reference body.  The msv is computed with respect to this body.
      RigidBody *b;	            // Incident body that body a is colliding with.

      // Each Rigid Body contains a vertex list, and the following indices
      // allow us to keep track of specific vertices involved in the contact
      // at various moments in time.
      unsigned int va_index;     // Index of vertex of a that is contained in b.
      unsigned int vb1_index;    // Index of first vertex that composes contact edge of b.
      unsigned int vb2_index;    // Index for second vertex that composes contact edge of b.

      Vector2 pb;                // Contact point on body b, given in local body coordinates.
      Vector2 n;                 // Outward pointing normal to contact edge of b.

      enum ContactType { Resting, Colliding, Separating };
  };

  // Separating axis test
  // Returns true if inter-penetration is detected between Rigid Bodys rb1 and
  // rb2.  Otherwise, false is returned. If inter-penetration is detected the
  // contact structure will be filled out, otherwise it will not be modified.
  bool sat(RigidBody *rb1, RigidBody *rb2, Contact &contact);

}

#endif
