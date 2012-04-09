#include "RigidBody.h"
#include "RBSolver.h"
#include <cassert>
#include <cstring>

namespace Rigid2D
{

  RigidBody::RigidBody(const Vector2 & position, const Vector2 & velocity, Real mass,
      const Real *vertex_array, unsigned int num_vertices)
  {
    assert(vertex_array != NULL);

    state_.position = position;
    state_.linearMomentum = velocity * mass;
    mass_ = mass;
    num_vertices_ = num_vertices;
    vertex_array_ = new Real[2 * num_vertices];
    memcpy(vertex_array_, vertex_array, 2 * num_vertices * sizeof(Real));
    forceAccumulator_ = Vector2(0, 0);

		vertices_ = realsToVector2s(num_vertices, vertex_array);
  }

  RigidBody::RigidBody(const Vector2 & position, const Vector2 & velocity, Real mass,
      const Vector2 **vertices, unsigned int num_vertices)
  {
      assert(vertices != NULL);

      state_.position = position;
      state_.linearMomentum = velocity * mass;
      mass_ = mass;
      num_vertices_ = num_vertices;
      forceAccumulator_ = Vector2(0, 0);
      vertex_array_ = NULL;

      vertices_ = new Vector2 *[num_vertices];

      for(unsigned int i = 0; i < num_vertices; ++i){
        vertices_[i] = new Vector2(vertices[i]->x, vertices[i]->y);
      }
  }

  RigidBody::~RigidBody()
  {
    if (vertex_array_ != NULL)
      delete [] vertex_array_;

    if (vertices_ != NULL) {
      for(unsigned int i = 0; i < num_vertices_; ++i){
        delete vertices_[i];
      }
      delete [] vertices_;
    }
  }

  void RigidBody::update()
  {
    RBState result;
    RBSolver::nextStep(*this, result);
    state_ = result;
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

  Vector2 ** RigidBody::getVertices() const
  {
    Vector2 **result = new Vector2 *[num_vertices_];

    for(unsigned int i = 0; i < num_vertices_; ++i){
      result[i] = new Vector2(vertices_[i]->x, vertices_[i]->y);
    }

    return result;
  }

  bool RigidBody::pointIsInterior(Real x, Real y)
  {
    // Go through all the edges calculating or2d(Mouse, pt1, pt2)
    Vector2 pt1, pt2, pt3;
    pt1.x = x;
    pt1.y = y;

    for (unsigned int i = 0; i < num_vertices_ - 1; i++)
    {
      pt2.x = vertex_array_[i*2];
      pt2.y = vertex_array_[i*2 + 1];
      pt3.x = vertex_array_[i*2 + 2];
      pt3.y = vertex_array_[i*2 + 3];
      if (orient2d(pt1, pt2, pt3) != -1)
        return false;
    }
    // special check for last edge
    pt2.x = vertex_array_[2 * num_vertices_ - 2];
    pt2.y = vertex_array_[2 * num_vertices_ - 1];
    pt3.x = vertex_array_[0];
    pt3.y = vertex_array_[1];
    if (orient2d(pt1, pt2, pt3) != -1)
      return false;
    return true;
  }
}
