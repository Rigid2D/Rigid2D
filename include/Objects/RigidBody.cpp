#include "RigidBody.h"
#include "RBSolver.h"
#include "Common/MathUtils.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>       // for infinity

namespace Rigid2D
{
  void RBState::operator *= (Real scalar)
  {
      position *= scalar;
      linearMomentum *= scalar;
      orientation *= scalar;
      angularMomentum *= scalar;
  }

  void RBState::operator /= (Real scalar)
  {
    position /= scalar;
    linearMomentum /= scalar;
    orientation /= scalar;
    angularMomentum /= scalar;
  }

  RBState RBState::operator + (const RBState & s) const
  {
    return RBState(position + s.position,
                   linearMomentum + s.linearMomentum,
                   orientation + s.orientation,
                   angularMomentum + s.angularMomentum);
  }

  RBState RBState::operator - (const RBState & s) const
  {
    return RBState(position - s.position,
                   linearMomentum - s.linearMomentum,
                   orientation - s.orientation,
                   angularMomentum - s.angularMomentum);
  }

  RBState RBState::operator * (const Real scalar) const
  {
    return RBState(position * scalar,
                   linearMomentum * scalar,
                   orientation * scalar,
                   angularMomentum * scalar);
  }

  RBState operator * (const Real scalar, const RBState &state)
  {
    return state * scalar;
  }

  RBState RBState::operator / (const Real scalar) const
  {
    assert(feq(scalar, 0.0) == false);
    return RBState(position / scalar,
                   linearMomentum / scalar,
                   orientation / scalar,
                   angularMomentum / scalar);
  }

  void RBState::operator = (const RBState & other)
  {
    position = other.position;
    linearMomentum = other.linearMomentum;
    angularMomentum = other.angularMomentum;
    orientation = other.orientation;
  }

  void RBState::normalizeOrientAngle()
  {
    orientation = fmod(orientation, TAU);
  }


  RigidBody::RigidBody(unsigned int num_vertices,
                       Real const *vertex_array,
                       Vector2 const &position,
                       Real mass,
                       Vector2 const &velocity,
                       Angle orientation)
  {
    assert(vertex_array != NULL);
		vertices_ = realArrayToVector2Array(num_vertices, vertex_array);
    initialize(num_vertices, vertices_, position, mass, velocity, orientation);
  }

  RigidBody::RigidBody(unsigned int num_vertices,
                       Vector2 const *vertices,
                       Vector2 const &position,
                       Real mass,
                       Vector2 const &velocity,
                       Angle orientation)
  {
    assert(vertices != NULL);
    vertices_ = new Vector2 [num_vertices];

    for(unsigned int i = 0; i < num_vertices; ++i){
      vertices_[i] = Vector2(vertices[i].x, vertices[i].y);
    }

    initialize(num_vertices, vertices, position, mass, velocity, orientation);
  }

  void RigidBody::initialize(unsigned int num_vertices,
                             Vector2 const *vertices,
                             Vector2 const &position,
                             Real mass,
                             Vector2 const &velocity,
                             Angle orientation)
  {
    // Need at least 3 vertices to make a convex polygon.
    if (num_vertices < 3) {
      throw InvalidParameterException(__LINE__, __FUNCTION__, __FILE__,
          "num_vertices cannot be less than 3");
    }

    if (signedArea(num_vertices, vertices) < 0) {
      throw InvalidParameterException(__LINE__, __FUNCTION__, __FILE__,
          "vertices must be given in counter-clockwise order so that signed-area of vertices is positive.");
    }

    state_.position = position;
    state_.linearMomentum = velocity * mass;
    state_.orientation = orientation;
    state_.angularMomentum = 0.0;
    prevState_ = state_;

    mass_ = mass;
    num_vertices_ = num_vertices;
    forceAccumulator_ = Vector2(0, 0);
    moi_ = momentOfInertia(num_vertices, vertices, mass);

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
    np_isIntersecting_ = false;
    RBState result;
    RBSolver::nextStep(*this, result);
    prevState_ = state_;
    state_ = result;

    // Add in damping to stop Rigid Bodies in motion
    state_.angularMomentum *= (1 - 0.005);
    state_.linearMomentum *= (1 - 0.005);

    worldBB_ = staticBB_.transform(state_.position, state_.orientation);
  }

  void RigidBody::computeForces(RBState & state)
  {
    forceAccumulator_ = Vector2(0, 0);
    torqueAccumulator_ = 0.0;
    std::unordered_set<Force*>::iterator it;
    Vector2 forceResult;
    Real torqueResult;

    for (it = forces_.begin(); it != forces_.end(); ++it) 
    {
      forceResult.x = 0.0;
      forceResult.y = 0.0;
      torqueResult = 0.0;

      (*it)->computeForce(this, &state, &forceResult, &torqueResult);
      forceAccumulator_ += forceResult;
      torqueAccumulator_ += torqueResult;
    }
  }

  void RigidBody::computeStateDeriv(const RBState &state, RBState &dState) const
  {
    assert(moi_ != 0);

    dState.position = state.linearMomentum / mass_;
    dState.linearMomentum = forceAccumulator_;
    dState.orientation = state.angularMomentum / moi_;
    dState.angularMomentum = torqueAccumulator_;
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

  Real RigidBody::getAngularMomentum() const
  {
    return state_.angularMomentum;
  }

  Real RigidBody::getMass() const
  {
    return mass_;
  }

  Angle RigidBody::getOrientation() const
  {
    return state_.orientation;
  }

  Real RigidBody::getMomentOfInertia() const
  {
    return moi_;
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

  void RigidBody::setOrientation(const Real orientation)
  {
    state_.orientation = orientation;
  }

  void RigidBody::setMass(const Real mass)
  {
    state_.linearMomentum /= mass_ / mass;
    mass_ = mass;
  }

  unsigned int RigidBody::getNumVertices() const
  {
    return num_vertices_;
  }

  Vector2 const * RigidBody::getVertices() const
  {
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

  Vector2 RigidBody::worldToLocalTransform(const Vector2 & point) const
  {
    Vector2 temp, result;
    temp = point - prevState_.position;
    result = temp;

    Real cos_theta = cos(state_.orientation - prevState_.orientation);
    Real sin_theta = sin(state_.orientation - prevState_.orientation);

    result.x = cos_theta * temp.x - sin_theta * temp.y;
    result.y = sin_theta * temp.x + cos_theta * temp.y;
    return result;
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

  Real RigidBody::findSlope(const Vector2 & v1, const Vector2 & v2) const
  {
    Real deltaX = v2.x - v1.x;
    Real deltaY = v2.y - v1.y;

    // find perpendicular slope of edge
    if (feq(deltaY, 0) && feq(deltaX, 0)) {   // repeating vertex
      return 0/0; // TODO should throw invalid parameter exception
    } else if (feq(deltaY, 0)) {              // horizontal line
      return std::numeric_limits<Real>::infinity();
    } else if (feq(deltaX, 0)) {              // vertical line
      return 0;
    } else {
      return -deltaX/deltaY;
    }
  }

  Real RigidBody::projectPointOnSlope(const Vector2 & point, Real slope) const
  {
    Real position;
    if (feq(slope, 0)) {
      position = point.x;
    } else if (slope == std::numeric_limits<Real>::infinity()) {
      position = point.y;
    } else {
      position = (point.x + point.y / slope);
      //?position /= axis_length;
    }
    return position;
  }

  Vector2 RigidBody::findProjectionInterval(Real slope) const
  {
    Vector2 interval;     // stores left and right most projected positions
    Real position;

    // project each vertex on given slope
    for (unsigned j = 0; j < num_vertices_; j++) {
      position = projectPointOnSlope(vertices_[j], slope);
      if (j == 0) {              // set inital values, can we eliminate this?
        interval[0] = position;
        interval[1] = position;
      } else {                  // update min/max interval values
        if (position < interval[0]) {
          interval[0] = position;
        } else if (position > interval[1]) {
          interval[1] = position;
        }
      }
    }
    return interval;
  }

  bool RigidBody::narrowPhase(RigidBody *rb, bool firstRB)
  {
    // two polygons. The steps are as follows:
    // 1) For each edge of both polygons find perpendicular
    // 2) Project all vertices into this perpendicular axis
    // 3) If the projected interval of p1 and p2 don't overlap, 
    // there is no collision, otherwise continue
    // 4) If for every 'edge-axis' projected intervals overlap,
    // there is a collision, otherwise there isn't

    // extrema points for the projected intervals of each RB
    Vector2 intervalRB1, intervalRB2;

    Real axis_slope, axis_length;

    // SAT for edges of this RB
    for (int i = 0; i < num_vertices_; i++) {
      axis_slope = atan(findSlope(vertices_[i], vertices_[(i+1) % num_vertices_]));
      axis_slope += state_.orientation;
      axis_slope = tan(axis_slope);
      printf("edge:slope  %d   %f\n", i, axis_slope);
      //?axis_length = sqrt(1 + axis_slope * axis_slope);

      intervalRB1 = findProjectionInterval(axis_slope);
      intervalRB2 = rb->findProjectionInterval(axis_slope);
      // if no intersection, we are done
      if (intervalRB1[1] < intervalRB2[0] ||
          intervalRB1[0] > intervalRB2[1]) {
        return false;
      }
      // store min intersection for MSV?
      //
    }

    if (firstRB) {
      return rb->narrowPhase(this, false);
    } else {
      np_isIntersecting_ = true;
      rb->np_isIntersecting_ = true;
      return true;
    }
  }


  bool RigidBody::pointIsInterior(Real x, Real y)
  {
    // Transform input from World to Local coordinate space
    x -= state_.position.x;
    y -= state_.position.y;

    Real xtemp, ytemp, cos_theta, sin_theta;
    cos_theta = cos(-state_.orientation);
    sin_theta = sin(-state_.orientation);

    xtemp = cos_theta*x - sin_theta*y;
    ytemp = sin_theta*x + cos_theta*y;
    x = xtemp;
    y = ytemp;

    // Go through all the edges calculating orient2d(Mouse, pt1, pt2)
    Vector2 pt1, pt2, pt3;
    pt1.x = x;
    pt1.y = y;

    for (unsigned int i = 0; i < num_vertices_ - 1; i++)
    {
      pt2 = vertices_[i];
      pt3 = vertices_[i+1];
      if (orient2d(pt1, pt2, pt3) != 1)
        return false;
    }
    // special check for last edge
    pt2 = vertices_[num_vertices_ - 1];
    pt3 = vertices_[0];
    if (orient2d(pt1, pt2, pt3) != 1)
      return false;
    return true;
  }
}
