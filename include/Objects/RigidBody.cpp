#include "RigidBody.h"
#include "RBSolver.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>       // for infinity

namespace Rigid2D
{

  RigidBody::RigidBody(const Vector2 & position, const Vector2 & velocity, Real mass,
      const Real *vertices, unsigned int num_vertices)
  {
    assert(vertices != NULL);

    state_.position = position;
    state_.linearMomentum = velocity * mass;
    mass_ = mass;
    num_vertices_ = num_vertices;
		
    vertices_ = new Vector2 [num_vertices];

    for(unsigned int i = 0; i < num_vertices; ++i){
      vertices_[i] = Vector2(vertices[2 * i], vertices[2 * i + 1]);
    }

    forceAccumulator_ = Vector2(0, 0);

    // compute staticBB
    staticBB_ = AABB(vertices_, num_vertices);
    bp_isIntersecting_ = false;
  }

  RigidBody::RigidBody(const Vector2 & position, const Vector2 & velocity, Real mass,
      const Vector2 *vertices, unsigned int num_vertices)
  {
    assert(vertices != NULL);

    state_.position = position;
    state_.linearMomentum = velocity * mass;
    mass_ = mass;
    num_vertices_ = num_vertices;

    vertices_ = new Vector2 [num_vertices];

    for(unsigned int i = 0; i < num_vertices; ++i){
      vertices_[i] = vertices[i];
    }

    forceAccumulator_ = Vector2(0, 0);

    // compute staticBB
    staticBB_ = AABB(vertices_, num_vertices);
    bp_isIntersecting_ = false;
  }

  RigidBody::~RigidBody()
  {
    delete [] vertices_;
  }

  void RigidBody::update()
  {
    bp_isIntersecting_ = false;
    RBState result;
    RBSolver::nextStep(*this, result);
    state_ = result;

    worldBB_ = staticBB_.transform(state_.position, 0);
  }

  void RigidBody::computeForces(RBState & state)
  {
    forceAccumulator_ = Vector2(0, 0);
    std::unordered_set<Force*>::iterator it;
    Vector2 forceResult;
    for (it = forces_.begin(); it != forces_.end(); ++it) 
    {
      (*it)->computeForce(this, &state, &forceResult);
      forceAccumulator_ += forceResult;
    }
  }

  void RigidBody::computeStateDeriv(const RBState &state, RBState &dState) const
  {
    dState.position = state.linearMomentum / mass_;
    dState.linearMomentum = forceAccumulator_;
  }

  bool RigidBody::checkCollision(RigidBody *rb)
  {
    if (broadPhase(rb)) {
       return narrowPhase(rb);
    } else {
      return false;
    }
  }

  void RigidBody::addForce(Force *force) 
  {
    forces_.insert(force);
  }

  void RigidBody::addForces(Force **forces, unsigned int numForces) 
  {
    for (unsigned i = 0; i < numForces; ++i) {
      forces_.insert(forces[i]);
    }
  }

  void RigidBody::removeForce(Force *force) {
    forces_.erase(force);
  }

  void RigidBody::removeForces(Force **forces, unsigned int numForces) 
  {
    for (unsigned int i = 0; i < numForces; ++i) {
      forces_.erase(forces[i]);
    }
  }

  Vector2 RigidBody::getPosition() const
  {
    return state_.position;
  }

  Vector2 RigidBody::getVelocity() const
  {
    return state_.linearMomentum / mass_;
  }

  Vector2 RigidBody::getLinearMomentum() const
  {
    return state_.linearMomentum;
  }

  Real RigidBody::getMass() const
  {
    return mass_;
  }

  void RigidBody::getState(RBState & dest) const
  {
    dest = state_;
  }

  void RigidBody::setState(RBState & state)
  {
    state_ = state;
  }

  void RigidBody::setPosition(const Vector2 & position)
  {
    state_.position = position;
  }

  void RigidBody::setPosition (Real xPos, Real yPos)
  {
    state_.position.x = xPos;
    state_.position.y = yPos;
  }

  void RigidBody::setVelocity(const Vector2 & velocity)
  {
    state_.linearMomentum = velocity * mass_;
  }

  void RigidBody::setVelocity (Real xVel, Real yVel)
  {
    state_.linearMomentum = Vector2(xVel, yVel) * mass_;
  }

  void RigidBody::setMass(const Real &mass)
  {
    state_.linearMomentum /= mass_ / mass;
    mass_ = mass;
  }

  unsigned int RigidBody::getNumVertices() const
  {
    return num_vertices_;
  }

  const Vector2 * RigidBody::getVertices() const
  {
    //[?]
    /*Vector2 *result = new Vector2 *[num_vertices_];

    for(unsigned int i = 0; i < num_vertices_; ++i){
      result[i] = new Vector2(vertices_[i]->x, vertices_[i]->y);
    }
    */
    return vertices_;
  }

  AABB* RigidBody::getStaticBB() 
  {
    return &staticBB_;
  }

  AABB* RigidBody::getWorldBB() 
  {
    return &worldBB_;
  }

  bool RigidBody::bp_isIntersecting() const
  {
    return bp_isIntersecting_;
  }

  bool RigidBody::np_isIntersecting() const
  {
    return np_isIntersecting_;
  }

  bool RigidBody::broadPhase(RigidBody *rb)
  {
    if (worldBB_.isIntersecting(*(rb->getWorldBB()))) {
      bp_isIntersecting_ = true;
      rb->bp_isIntersecting_ = true;
      return true;
    }

    return false;
  }

  bool RigidBody::narrowPhase(RigidBody *rb)
  {
    /*
    // Use the Separation Axis Theorem to find distance between
    // two polygons. The steps are as follows:
    // 1) For each edge of both polygons find perpendicular
    // 2) Project all vertices into this perpendicular axis
    // 3) If the projected interval of p1 and p2 don't overlap, 
    // there is no collision, otherwise continue
    // 4) If for every 'edge-axis' projected intervals overlap,
    // there is a collision, otherwise there isn't

    // extremum points for the intervals of each RB
    Real min1, max1, min2, max2;

    Real axis_slope, axis_length;
    Real deltaX, deltaY, pos;

    // SAT for edges of "this" RB
    for (int i = 0; i < num_vertices_-1; i++) {
      deltaX = vertex_array_[i*2 + 2] - vertex_array_[i*2];
      deltaY = vertex_array_[i*2 + 1] - vertex_array_[i*2 + 3];

      // find perpendicular slope of edge
      if (deltaY == 0) {
        if (deltaX == 0) {        // repeating vertex
          continue;
        } else {                  // horizontal line
          axis_slope = std::numeric_limits<Real>::infinity();
        }
      } else if (deltaX == 0) {   // vertical line
        axis_slope = 0;
      } else {
        axis_slope = -deltaX/deltaY;
      }

      axis_length = sqrt(1 + axis_slope * axis_slope);

      // project each vertex of "this" RB
      for (int j = 0; j < num_vertices_; j++) {
        if (axis_slope == 0) {
          pos = vertex_array_[j*2];
        } else if (axis_slope == std::numeric_limits<Real>::infinity()) {
          pos = vertex_array_[j*2 + 1];
        } else {
          pos = (vertex_array_[j*2] + vertex_array_[j*2 + 1] * axis_slope);
          pos /= axis_length;
        }

       if (j == 0) {              // set inital values, can we eliminate this?
          min1 = pos;
          max1 = pos;
        } else {                  // update min/max interval values
          if (pos > max1) {
            max1 = pos;
          } else if (pos < min1) {
            min1 = pos;
          }
        }
      }

      // project each vertex of other RB
      for (int j = 0; j < rb->num_vertices_; j++) {
        if (axis_slope == 0) {
          pos = rb->vertex_array_[j*2];
        } else if (axis_slope == std::numeric_limits<Real>::infinity()) {
          pos = rb->vertex_array_[j*2 + 1];
        } else {
          pos = (rb->vertex_array_[j*2] + rb->vertex_array_[j*2 + 1] * axis_slope);
          pos /= axis_length;
        }

       if (j == 0) {              // set inital values, can we eliminate this?
          min2 = pos;
          max2 = pos;
        } else {                  // update min/max interval values
          if (pos > max2) {
            max2 = pos;
          } else if (pos < min2) {
            min2 = pos;
          }
        }
      }

    }
    */
    return false;
  }


  bool RigidBody::pointIsInterior(Real x, Real y)
  {
    // Go through all the edges calculating orient2d(Mouse, pt1, pt2)
    Vector2 pt1, pt2, pt3;
    pt1.x = x;
    pt1.y = y;

    for (unsigned int i = 0; i < num_vertices_ - 1; i++)
    {
      pt2 = vertices_[i];
      pt3 = vertices_[i+1];
      if (orient2d(pt1, pt2, pt3) != -1)
        return false;
    }
    // special check for last edge
    pt2 = vertices_[num_vertices_ - 1];
    pt3 = vertices_[0];
    if (orient2d(pt1, pt2, pt3) != -1)
      return false;
    return true;
  }
}
