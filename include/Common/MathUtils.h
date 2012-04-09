#ifndef RIGID2D_MATH_UTILS_H
#define RIGID2D_MATH_UTILS_H

#include "RigidSettings.h"
#include "Vector2.h"
#include "RigidException.h"

// Class for common math functions.
// Functions:
// *or2d - finds the orientation of a triangle given three points
// *det3 - determinant of a 3x3 matrix

namespace Rigid2D
{
  // Given points A, B, and C, if orient2d(A,B,C) > 0, C lies to the left of the
  // directed lien AB, and the triangle ABC is oriented counterclockwise.  If
  // orient2d(A,B,C) < 0, C lies to the right of the directed line AB, and the
  // triangle ABC is oriented clockwise.  When orient2d(A,B,C) = 0, the three
  // points are collinear.
  int orient2d(Vector2 & pt1, Vector2 & pt2, Vector2 & pt3);

  Real det3(Real a0, Real a1, Real a2, Real b0, Real b1, Real b2, Real c0, Real c1, Real c2);

  // Returns the signed area of the polygon represented by <vertices>.  Vertices
  // are assumed to be numbered in order of their occurrence along the polygon's
  // perimeter, either in clockwise or counterclockwise order.  If the polygon
  // is simple (non-intersecting sides), with the vertices numbered in a
  // counterclockwise direction, the signed area is the area.
  // If num_vertices is less than 2, then signedArea returns zero.
	Real signedArea (unsigned int num_vertices, Vector2 **vertices);

	// Returns the centroid, represented as a Vector2, of the polygon
	// whos vertices are given by the parameter <vertices>.
  // Vertices are assumed to be numbered in order of their occurrence along the
  // polygon's perimeter, either in clockwise or counterclockwise order.
  // Assumes num_vertices > 2.  If num_vertices <= 2, method throws an
  // InvalidParameterException.
  Vector2 centroid (unsigned int num_vertices, Vector2 **vertices) throw (Rigid2D::Exception);

  // For each pair of Reals within vertex_array, a new Vector2 object is created
  // to hold their values.  A pointer to an array of pointers to all Vector2
  // objects created is then returned.
  // Assumes num_vertices > 0.  If num_vertices = 0, method will throw an
  // InvalidParameterException error.
  Vector2 ** realsToVector2s(unsigned int num_vertices, const Real *vertex_array) throw (Rigid2D::Exception, std::bad_alloc);
}

#endif
