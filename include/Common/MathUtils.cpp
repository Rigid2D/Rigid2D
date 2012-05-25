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


	Real signedArea (unsigned int num_vertices, Vector2 const *vertices)
	{
    if (num_vertices < 2)
      return 0.0F;

    assert(vertices != NULL);

    Vector2 const *v = vertices;
    unsigned int n = num_vertices;
    Real result = 0;

    for(unsigned int i = 0; i < n - 1; ++i) {
      result += v[i].x * v[i+1].y - v[i+1].x * v[i].y;
    }

    // Last result wraps around to index 0
    result += v[n-1].x * v[0].y - v[0].x * v[n-1].y;

    return (0.5 * result);
	}

  Vector2 centroid (unsigned int num_vertices, Vector2 const *vertices) throw (Rigid2D::Exception)
  {
    assert(vertices != NULL);

    if (num_vertices < 3) {
      throw InvalidParameterException(__LINE__, __FUNCTION__, __FILE__,
          "num_vertices cannot be less than 3");
    }

    Vector2 const *v = vertices;
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

  Real amoi_triangle (Vector2 const & v0, Vector2 const & v1, Vector2 const & v2) {
    Real a_squared = Vector2::getLengthSquared(v0-v2);
    Real b_squared = Vector2::getLengthSquared(v2-v1);
    Real c_squared = Vector2::getLengthSquared(v1-v0);

    return (a_squared + b_squared + c_squared) / 36;
  }

  Real momentOfInertia (unsigned int num_vertices, Vector2 const *vertices, Real mass) {
    Real result = 0;
    Real A;                         // Area of polygon representing.
    Vector2 C;                      // Centroid of polygon.
    Vector2 C_i;                    // Centroid of ith triangle.
    Vector2 tmp[3];                 // Stores specific elements of vertices_.
    Vector2 const *v = vertices;    // Alias.
    unsigned int n = num_vertices;  // Alias.

    C = centroid(n, v);
    A = signedArea(n, v);

    // If n = 3, the shape is a triangle.
    if (n == 3) {
      return mass * amoi_triangle(v[0], v[1], v[2]);
    }

    // Break up polygon into n triangles.
    // Compute moment of inertia about each triangle.
    // Sum up using parallel axis theorem.

    // Do calculation with all but the last triangle using vertices 1 to n - 1.
    for(unsigned int i = 0; i <= n - 2; ++i) {
      tmp[0] = v[i];
      tmp[1] = v[i+1];
      tmp[2] = C;

      C_i = centroid(3, tmp);
      result += signedArea(3, tmp) * (amoi_triangle(tmp[0], tmp[1], tmp[2]) +
          Vector2::getLengthSquared (C - C_i));
    }

    // Now add in last triangle, which uses vertices n and 1.
    tmp[0] = v[n-1];
    tmp[1] = v[0];
    tmp[2] = C;

    C_i = centroid(3, tmp);
    result += signedArea(3, tmp) * (amoi_triangle(tmp[0], tmp[1], tmp[2]) +
        Vector2::getLengthSquared (C - C_i));

    return result * mass / A;
  }

  Vector2 * realArrayToVector2Array (unsigned int num_vertices, Real const *vertex_array)
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

  // Parameterize the line segment ab with P(t) = a + t*(b - a).  The point d(t) = a + t * (b - a),
  // where t = (c - a) . n/|b - a|, with t in [0,1] and n = (b - a) / |b - a|.
  void ClosestPtPointSegment(Vector2 const &c, Vector2 const &a, Vector2 const &b, Vector2 &d) {
    Vector2 ab = b - a;
    // Project c onto ab, but defer dividing by ab.dot(ab).
    Real t = (c - a).dot(ab);
    if (t <= 0.0f) {
      // c projects outside the interval [a,b] and is closest to the point a.
      t = 0.0f;
      d = a;
    }
    else {
      Real denom = ab.dot(ab);
      if (t >= denom) {
        // c projects outside the interval [a,b] and is closest to the point b.
        t = 1.0f;
        d = b;
      }
      else {
        // c projects onto the interval [a,b]; must do deferred division now.
        t = t / denom;
        d = a + t * ab;
      }
    }
  }

} // end namespace Rigid2D
