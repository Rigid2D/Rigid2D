#ifndef RIGID2D_AABB_H
#define RIGID2D_AABB_H

#include "Common/RigidSettings.h"
#include "Common/Vector2.h"

namespace Rigid2D
{
  class AABB
  {
    public:
      AABB() {};
      AABB(Real *vertex_arr, unsigned int num_vertices);
      AABB transform(Vector2 translation, Real rotation) const;
      bool isIntersecting(AABB &other) const;

      Vector2 minVertex_;    // bottom-left
      Vector2 maxVertex_;    // top-right
  };
}

#endif
