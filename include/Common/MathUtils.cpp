#include "MathUtils.h"
#include <cassert>

namespace Rigid2D
{
  int or2d(Vector2 & pt1, Vector2 & pt2, Vector2 & pt3)
  {
    Real det = det3(1.0, 1.0, 1.0,
              pt1.x, pt2.x, pt3.x,
              pt1.y, pt2.y, pt3.y);
    if (feq(det,0))
      return 0;
    else if (det < 0)
      return -1;
    else
      return 1;
  }

  Real det3(Real a0, Real a1, Real a2, Real b0, Real b1, Real b2, Real c0, Real c1, Real c2)
  {
    return ((a0 * b1 * c2) + (a1 * b2 * c0) + (a2 * b0 * c1)
      - (a2 * b1 * c0) - (a1 * b0 * c2) - (a0 * b2 * c1));
  }

	Vector2 centroid (unsigned int vertice_count, Vector2 **vertices)
	{
	  	
	}

	Real signedArea (unsigned int vertice_count, Vector2 **vertices)
	{
    if (vertice_count < 2)
      return 0.0F;

    assert(vertices != NULL);

    Vector2 **v = vertices;
    Real result = 0;

    for(unsigned int i = 0; i < vertice_count; ++i) {
      result += v[i]->x * v[i+1]->y - v[i+1]->x * v[i]->y
    }

    return (0.5 * result);
	}
}
