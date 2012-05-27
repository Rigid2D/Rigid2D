#include "RigidBody.h"
#include "RBSolver.h"
#include "Common/MathUtils.h"
#include "Common/RigidException.h"
#include <cassert>
#include <cstring>
#include <iostream>

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
                       Angle orientation,
                       Real restitution)
  {
    assert(vertex_array != NULL);
		vertices_ = realArrayToVector2Array(num_vertices, vertex_array);
    initialize(num_vertices, vertices_, position, mass, velocity, orientation, restitution);
  }

  RigidBody::RigidBody(unsigned int num_vertices,
                       Vector2 const *vertices,
                       Vector2 const &position,
                       Real mass,
                       Vector2 const &velocity,
                       Angle orientation,
                       Real restitution)
  {
    assert(vertices != NULL);
    vertices_ = new Vector2 [num_vertices];

    for(unsigned int i = 0; i < num_vertices; ++i){
      vertices_[i] = Vector2(vertices[i].x, vertices[i].y);
    }

    initialize(num_vertices, vertices, position, mass, velocity, orientation, restitution);
  }

  void RigidBody::initialize(unsigned int num_vertices,
                             Vector2 const *vertices,
                             Vector2 const &position,
                             Real mass,
                             Vector2 const &velocity,
                             Angle orientation,
                             Real restitution)
  {
    assert(mass != 0); // Needed by invMass_ to prevent divide by zero.

    // Need at least 3 vertices to make a convex polygon.
    if (num_vertices < 3) {
      throw InvalidParameterException(__LINE__, __FUNCTION__, __FILE__,
          "num_vertices cannot be less than 3");
    }

    if (signedArea(num_vertices, vertices) < 0) {
      throw InvalidParameterException(__LINE__, __FUNCTION__, __FILE__,
          "vertices must be given in counter-clockwise order so that signed-area of vertices is positive.");
    }

    transformed_vertices_ = new Vector2 [num_vertices];
    state_.position = position;
    state_.linearMomentum = velocity * mass;
    state_.orientation = orientation;
    state_.angularMomentum = 0.0;
    prevState_ = state_;

    mass_ = mass;
    invMass_ = 1/mass;
    restitution_ = restitution;
    num_vertices_ = num_vertices;
    forceAccumulator_ = Vector2(0, 0);
    moi_ = momentOfInertia(num_vertices, vertices, mass);
    invMoi_ = 1/moi_;

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

    // Add in dampening to stop Rigid Bodies in motion
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
    Contact *contact = new Contact;
    if (broadPhase(rb)) {
       return narrowPhase(rb, contact);
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

  Real RigidBody::getInvMass() const
  {
    return invMass_;
  }

  Real RigidBody::getInvMoi() const
  {
    return invMoi_;
  }

  Angle RigidBody::getOrientation() const
  {
    return state_.orientation;
  }

  Real RigidBody::getRestitution() const
  {
    return restitution_;
  }

  Real RigidBody::getMomentOfInertia() const
  {
    return moi_;
  }
  void RigidBody::getState(RBState & dest) const
  {
    dest = state_;
  }

  void RigidBody::getPrevState(RBState & dest) const
  {
    dest = prevState_;
  }

  void RigidBody::setState(RBState & state)
  {
    state_ = state;
  }

  void RigidBody::setPrevState(RBState & prev_state)
  {
    prevState_ = prev_state;
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

  Vector2 const * RigidBody::getTransformedVertices() const
  {
    return transformed_vertices_;
  }

  Vector2 const * RigidBody::getVertices() const
  {
    return vertices_;
  }

  Vector2 RigidBody::getVertex(unsigned int index) const
  {
    assert(index < num_vertices_);
    return vertices_[index];
  }

  Vector2 RigidBody::getTransformedVertex(unsigned int index) const
  {
    assert(index < num_vertices_);
    return transformed_vertices_[index];
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

  Vector2 RigidBody::prevWorldToCurrentLocalTransform(const Vector2 & point) const
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

  // Apply reverse transform T(-x) * R(-theta) to the point, where x is translation from world coordinate origin
  // to center of mass of Rigid Body, and theta is world space orientation angle of Rigid Body.
  Vector2 RigidBody::worldToLocalTransform(const Vector2 & point, RBState::FrameSpecifier frame) const
  {
    RBState state;
    if (frame == RBState::CURRENT)
      state = state_;
    else
      state = prevState_;

    Vector2 result;
    result = point;

    Real cos_theta = cos(-state.orientation);
    Real sin_theta = sin(-state.orientation);

    result.x = cos_theta * point.x - sin_theta * point.y;
    result.y = sin_theta * point.x + cos_theta * point.y;

    result -= state.position;
    return result;
  }

  // Apply transform T(x) * R(theta) to the point, where x is translation from world coordinate origin
  // to center of mass of Rigid Body, and theta is world space orientation angle of Rigid Body.
  Vector2 RigidBody::localToWorldTransform(Vector2 const & point, RBState::FrameSpecifier frame) const
  {
    RBState state;
    if (frame == RBState::CURRENT)
      state = state_;
    else
      state = prevState_;

    Vector2 result;
    result = point;

    Real cos_theta = cos(state.orientation);
    Real sin_theta = sin(state.orientation);

    result.x = cos_theta * point.x - sin_theta * point.y;
    result.y = sin_theta * point.x + cos_theta * point.y;

    result += state.position;
    return result;
  }

  void RigidBody::updateTransformedVertices() const {
    for (unsigned i = 0; i < num_vertices_; i++) {
      transformed_vertices_[i] = localToWorldTransform(vertices_[i]);
    }
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


  bool RigidBody::narrowPhase(RigidBody *rb, Contact *contact)
  {

    if ( sat(this, rb, contact) ) {
      np_isIntersecting_ = true;
      rb->np_isIntersecting_ = true;
      return true;
    } else {
      return false;
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

    // Go through all the edges calculating orient2d(pt1, pt2, pt3), where pt1
    // is the point composed of arguments x and y, and the points pt2 and pt3
    // cycle through all vertices of calling object.
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

  // Velocity of a point on Rigid Body is given by
  // v + ω ^ (p - x)
  //
  // v = linear velocity of Rigid Body
  // ω = angular velocity of Rigid Body
  // p = point on Rigid Body
  // x = center of mass of Rigid Body
  // ^ = cross product
  Vector2 RigidBody::pointVelocity(Vector2 point) {
    Vector2 v = state_.linearMomentum * invMass_;  // linear velocity
    Vector2 r = point - state_.position;           // (p - x)
    Real omega = state_.angularMomentum * invMoi_; // angular velcity

    return v + omega * r.perp();
  }
}
