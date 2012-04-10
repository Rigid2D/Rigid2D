#include "MathUtils.h"
#include "RigidSettings.h"
#include "feq.h"
#include <cassert>

namespace Rigid2D
{
  int orient2d(Vector2 & pt1, Vector2 & pt2, Vector2 & pt3)
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


	Real signedArea (unsigned int num_vertices, Vector2 *vertices)
	{
    if (num_vertices < 2)
      return 0.0F;

    assert(vertices != NULL);

    Vector2 *v = vertices;
    unsigned int n = num_vertices;
    Real result = 0;

    for(unsigned int i = 0; i < n - 1; ++i) {
      result += v[i].x * v[i+1].y - v[i+1].x * v[i].y;
    }

    // Last result wraps around to index 0
    result += v[n-1].x * v[0].y - v[0].x * v[n-1].y;

    return (0.5 * result);
	}

  Vector2 centroid (unsigned int num_vertices, Vector2 *vertices) throw (Rigid2D::Exception)
  {
    assert(vertices != NULL);

    if (num_vertices < 3) {
      throw InvalidParameterException(__LINE__, __FUNCTION__, __FILE__,
          "num_vertices cannot be 0");
    }

    Vector2 *v = vertices;
    Real Cx = 0;
    Real Cy = 0;
    Real A;
    unsigned int n = num_vertices;
    unsigned int i;

    for(i = 0; i < n - 1; ++i) {
      Cx += (v[i].x + v[i+1].x) * (v[i].x * v[i+1].y - v[i+1].x * v[i].y);
      Cy += (v[i].y + v[i+1].y) * (v[i].x * v[i+1].y - v[i+1].x * v[i].y);
    }

    // Last index wraps around to 0
    Cx += (v[n-1].x + v[0].x) * (v[n-1].x * v[0].y - v[0].x * v[n-1].y);
    Cy += (v[n-1].y + v[0].y) * (v[n-1].x * v[0].y - v[0].x * v[n-1].y);

    A = signedArea(n, v);

    Cx /= 6*A;
    Cy /= 6*A;

    return Vector2(Cx, Cy);
  }

  Vector2 * realArrayToVector2Array(unsigned int num_vertices, const Real *vertex_array)
    throw (Rigid2D::Exception, std::bad_alloc)
  {
    assert(vertex_array != NULL);

    if (num_vertices == 0) {
      throw InvalidParameterException(__LINE__, __FUNCTION__, __FILE__,
          "num_vertices cannot be less than 1");
    }

    Vector2 *result = new Vector2 [num_vertices];

    // Every two Reals in vertex_array makes a Vector2, where no element in vertex_array is ever
    // used twice.
    for(unsigned int i = 0; i < num_vertices; ++i){
      result[i] = Vector2(vertex_array[2*i], vertex_array[2*i+1]);
    }

    return result;
  }

}
