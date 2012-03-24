#ifndef RIGID2D_RIGID_BODY_H
#define RIGID2D_RIGID_BODY_H

#include "Common/RigidSettings.h"
#include "Common/MathUtils.h"
#include "Common/Vector2.h"
#include "Objects/Force.h"
#include <unordered_set>

namespace Rigid2D
{
  class Force;
  class RBSolver;

  // Stores the state needed for force calculations.
  struct RBState {
    Vector2 position;
    Vector2 momentum;

    RBState() {}
    RBState(const Vector2 &pos, const Vector2 &mom) {
      position = pos;
      momentum = mom;
    }

    void operator *= (Real scalar) {
      position *= scalar;
      momentum *= scalar;
    }

    void operator /= (Real scalar) {
      position /= scalar;
      momentum /= scalar;
    }

    RBState operator + (const RBState & s) const {
      return RBState(position + s.position, momentum + s.momentum);
    }

    RBState operator - (const RBState & s) const {
      return RBState(position - s.position, momentum - s.momentum);
    }

    RBState operator * (const Real scalar) const {
      return RBState(position * scalar, momentum * scalar);
    }

    friend RBState operator * (const Real scalar, const RBState &state) {
      return state * scalar;
    }

    RBState operator / (const Real scalar) const {
      assert(feq(scalar, 0.0) == false);
      return RBState(position / scalar, momentum / scalar);
    }
  };


  // Stores a rigid body - the base object used in the physics simulation.
  class RigidBody
  {
    public:
      /** Constructor for RigidBody
       *
       * @param vertex_array should be an array of tuples in the form of (x,y). It will get deep-copied
       * @param vertex_count is the number of tuples (not the number of Reals) */
      RigidBody(const Vector2 &position, const Vector2 &velocity, 
                Real mass, Real *vertex_array, int vertex_count);
      ~RigidBody();

      /** Computes and sets the state of the object for next frame.
       * ODE computations are done from this function.
      */
      void update();

      /** Zeroes and then sums forces into forceAccumulator_ */
      void computeForces(RBState &state);
      void computeStateDeriv(const RBState &state, RBState &dState) const;


      /** Tells RigidBodySystem to apply the given force from here on out.
			 * If the force was already previously given, it does not apply it a
			 * second time.
       *
       * @param force pointer to a Force object.
       * @see addForces()
       * @see removeForce()
       * @see removeForces()
       */
      void addForce(Force *force);

      /** Same as addForce() but for multiple forces.
			 *
       * @param forces array of Force object pointers.
			 * @param numForces length of forces.
       * @see addForce()
       * @see removeForce()
       * @see removeForces()
       */
			void addForces(Force **forces, unsigned int numForces);

      /** Tells RigidBodySystem to remove the force from the list of forces
			 * being applied. If the force was not being applied, it does nothing.
       *
       * @param force pointer to a Force object.
       * @see addForces()
       * @see addForces()
       * @see removeForces()
       */
			void removeForce(Force *force);

      /** Same as removeForce() but for multiple forces.
			 *
       * @param forces array of Force object pointers.
			 * @param numForces length of forces.
       * @see addForce()
       * @see addForces()
       * @see removeForce()
       */
			void removeForces (Force **forces, unsigned int numForces);

      Vector2 getPosition() const;
      Vector2 getVelocity() const;
      Vector2 getMomentum() const;
      Real getMass() const;
      void getState(RBState & state) const;

			void setState(RBState & state);
      void setPosition (const Vector2 & position);
      void setPosition (Real xPos, Real yPos);
      void setVelocity (const Vector2 & velocity);
      void setVelocity (Real xVel, Real yVel);
      void setMass(const Real &);


      int getVertexCount() const;
      Real* getVertexArray() const;

      /* Given a point in graphics coordinate space, this function returns true if
       * the point lies within the convex polygon defined by vertex_array_.*/
      bool pointIsInterior(Real x, Real y);

    protected:
      RBState state_;             // Position, momentum
      //Vector2 position_;          // Position of center of mass
      Vector2 velocity_;          // Velocity of center of mass (implicitly calculated)
      //Vector2 momentum_;          // Total momentum of RigidBody
      Vector2 forceAccumulator_;  // Sum of forces acting on the center of mass of RigidBody
      Real mass_;                 // Object mass
      std::unordered_set<Force*> forces_;    // all forces being applied to this RB
      int vertex_count_;
      Real *vertex_array_;

  };
}

#endif
