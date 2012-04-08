#include "RigidBody.h"
#include "RBSolver.h"
#include <cassert>
#include <cstring>
#include <iostream>

namespace Rigid2D
{

  RigidBody::RigidBody(const Vector2 & position, const Vector2 & velocity, Real mass,
      Real *vertex_array, unsigned int vertex_count)
  {
    state_.position = position;
    state_.momentum = velocity * mass;
    mass_ = mass;
    vertex_count_ = vertex_count;
    vertex_array_ = new Real[2 * vertex_count];
    memcpy(vertex_array_, vertex_array, 2 * vertex_count * sizeof(Real));
    forceAccumulator_ = Vector2(0, 0);
    bp_isIntersecting_ = false;

    // compute staticBB
    staticBB_ = AABB(vertex_array, vertex_count);
  }

  RigidBody::~RigidBody()
  {
    delete vertex_array_;
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
    dState.position = state.momentum / mass_;
    dState.momentum = forceAccumulator_;
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
    std::cout << state_.momentum.x << std::endl;
    return state_.momentum / mass_;
  }

  Vector2 RigidBody::getMomentum() const
  {
    return state_.momentum;
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
    state_.momentum = velocity * mass_;
  }

  void RigidBody::setVelocity (Real xVel, Real yVel)
  {
    state_.momentum = Vector2(xVel, yVel) * mass_;
  }

  void RigidBody::setMass(const Real &mass)
  {
    state_.momentum /= mass_ / mass;
    mass_ = mass;
  }

  int RigidBody::getVertexCount() const
  {
    return vertex_count_;
  }

  Real* RigidBody::getVertexArray() const
  {
    return vertex_array_;
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
    // Use the Separation Axis Theorem to find distance between
    // two polygons.
    

    return false;
  }

  bool RigidBody::pointIsInterior(Real x, Real y)
  {
    // Go through all the edges calculating or2d(Mouse, pt1, pt2)
    Vector2 pt1, pt2, pt3;
    pt1.x = x;
    pt1.y = y;

    for (int i = 0; i < vertex_count_ - 1; i++)
    {
      pt2.x = vertex_array_[i*2];
      pt2.y = vertex_array_[i*2 + 1];
      pt3.x = vertex_array_[i*2 + 2];
      pt3.y = vertex_array_[i*2 + 3];
      if (or2d(pt1, pt2, pt3) != -1)
        return false;
    }
    // special check for last edge
    pt2.x = vertex_array_[2 * vertex_count_ - 2];
    pt2.y = vertex_array_[2 * vertex_count_ - 1];
    pt3.x = vertex_array_[0];
    pt3.y = vertex_array_[1];
    if (or2d(pt1, pt2, pt3) != -1)
      return false;
    return true;
  }
}
