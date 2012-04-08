#ifndef RIGID2D_MATH_UTILS_H
#define RIGID2D_MATH_UTILS_H

#include "RigidSettings.h"
#include "Vector2.h"

// Class for common math functions.
// Functions:
// *or2d - finds the orientation of a triangle given three points
// *det3 - determinant of a 3x3 matrix

namespace Rigid2D
{
  int or2d(Vector2 & pt1, Vector2 & pt2, Vector2 & pt3);

  Real det3(Real a0, Real a1, Real a2, Real b0, Real b1, Real b2, Real c0, Real c1, Real c2);

	// Returns the centroid, represented as a Vector2, of the polygon
	// whos vertices are given by the parameter <vertices>.
	Vector2 centroid (unsigned int vertice_count, Vector2 **vertices);

	// Returns the signed area of the polygon represented by <vertices>.
	// If the polygon is simple (non-intersecting sides), with the
	// vertices numbered in a counterclockwise direction, the signed area
	// is the area.
  // If vertice_count is less than 2, then zero is the signed area returned.
	Real signedArea (unsigned int vertice_count, Vector2 **vertices);
}

#endif
