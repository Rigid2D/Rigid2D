#include "AABB.h"
#include <cmath>

namespace Rigid2D
{
  AABB::AABB(Real *vertex_arr, unsigned int num_vertices)
  {
    Real minX, minY, maxX, maxY;
    minX = maxX = vertex_arr[0];
    minY = maxY = vertex_arr[1];

    // find minmax vertices
    for (unsigned int i = 1; i < num_vertices; i++) {
      if (vertex_arr[2*i] < minX) {
        minX = vertex_arr[2*i];
      } else if (vertex_arr[2*i] > maxX) {
        maxX = vertex_arr[2*i];
      }

      if (vertex_arr[2*i+1] < minY) {
        minY = vertex_arr[2*i+1];
      } else if (vertex_arr[2*i+1] > maxY) {
        maxY = vertex_arr[2*i+1];
      }
    }

    minVertex_ = Vector2(minX, minY);
    maxVertex_ = Vector2(maxX, maxY);
  }

  AABB AABB::transform(Vector2 translation, Real rotation) const
  {
    // Array that stores 4 vertices, starting with top-left going ccw
    Real vertex_arr[8];
    vertex_arr[0] = minVertex_.x;
    vertex_arr[1] = maxVertex_.y;
    vertex_arr[2] = maxVertex_.x;
    vertex_arr[3] = maxVertex_.y;
    vertex_arr[4] = maxVertex_.x;
    vertex_arr[5] = minVertex_.y;
    vertex_arr[6] = minVertex_.x;
    vertex_arr[7] = minVertex_.y;

    // Apply rotation
    Real cos_theta = cos(rotation);
    Real sin_theta = sin(rotation);

    for (int i = 0; i < 4; i++) {
      vertex_arr[2*i] = vertex_arr[2*i] * cos_theta - vertex_arr[2*i+1] * sin_theta;
      vertex_arr[2*i+1] = vertex_arr[2*i] * sin_theta + vertex_arr[2*i+1] * cos_theta;
    }

    // Apply translation
    for (int j = 0; j < 4; j++) {
      vertex_arr[2*j] += translation.x;
      vertex_arr[2*j+1] += translation.y;
    }

    return AABB(vertex_arr, 4);
  }

  bool AABB::isIntersecting(AABB &other) const
  {
    if (maxVertex_.x < other.minVertex_.x || minVertex_.x > other.maxVertex_.x) 
      return 0;
    if (maxVertex_.y < other.minVertex_.y || minVertex_.y > other.maxVertex_.y) 
      return 0;

    return 1;
  }
}
