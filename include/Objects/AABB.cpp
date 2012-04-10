#include "AABB.h"
#include <cmath>

namespace Rigid2D
{
  AABB::AABB(const Vector2 *vertex_arr, unsigned int num_vertices)
  {
    Real minX, minY, maxX, maxY;
    minX = maxX = vertex_arr[0].x;
    minY = maxY = vertex_arr[0].y;

    // find minmax vertices
    for (unsigned int i = 1; i < num_vertices; i++) {
      if (vertex_arr[i].x < minX) {
        minX = vertex_arr[i].x;
      } else if (vertex_arr[i].x > maxX) {
        maxX = vertex_arr[i].x;
      }

      if (vertex_arr[i].y < minY) {
        minY = vertex_arr[i].y;
      } else if (vertex_arr[i].y > maxY) {
        maxY = vertex_arr[i].y;
      }
    }

    minVertex_ = Vector2(minX, minY);
    maxVertex_ = Vector2(maxX, maxY);
  }

  AABB AABB::transform(Vector2 translation, Real rotation) const
  {
    // Array that stores 4 vertices, starting with bottom-left going ccw
    Vector2 vertex_arr[4];
    vertex_arr[0] = minVertex_;
    vertex_arr[1] = Vector2(minVertex_.x, maxVertex_.y);
    vertex_arr[2] = maxVertex_;
    vertex_arr[3] = Vector2(maxVertex_.x, minVertex_.y);

    // Apply rotation
    Real cos_theta = cos(rotation);
    Real sin_theta = sin(rotation);

    for (int i = 0; i < 4; i++) {
      vertex_arr[i].x = vertex_arr[i].x * cos_theta - vertex_arr[i].y * sin_theta;
      vertex_arr[i].y = vertex_arr[i].x * sin_theta + vertex_arr[i].y * cos_theta;
    }

    // Apply translation
    for (int j = 0; j < 4; j++) {
      vertex_arr[j].x += translation.x;
      vertex_arr[j].y += translation.y;
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
