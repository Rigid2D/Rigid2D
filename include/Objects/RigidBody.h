#ifndef RIGID2D_RIGID_BODY_H
#define RIGID2D_RIGID_BODY_H

#include "Objects/RigidBody.h"
#include "Objects/Force.h"
#include "Common/RigidSettings.h"
#include "Common/MathUtils.h"
#include "Common/Vector2.h"
#include "Common/RungeKutta4RigidBodySolver.h"
#include <unordered_set>

namespace Rigid2D
{
  class RigidBody
  {
    public:
      /** Constructor for RigidBody
       *
       * @param vertex_array should be an array of tuples in the form of (x,y). It will get deep-copied
       * @param vertex_count is the number of tuples (not the number of Reals) */
      RigidBody(Vector2 position, Real mass, Real *vertex_array, int vertex_count, Vector2 velocity);
      ~RigidBody();

      /** Computes and sets the state of the object for next frame.
       * ODE computations are done from this function.
      */
      void update();

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


      // returns position of center of mass for RigidBody
      Vector2 getPosition() const;

      Vector2 getVelocity() const;

      Vector2 getMomentum() const;

      Real getMass() const;

      int getVertexCount() const;

      Real* getVertexArray() const;

      Vector2 getForceAccumulator() const;

			// Copies RigidBody state information, such as position, momentum,
			// orientation, and force accumulator, to the destination array dst.
      void getState(Real *dst) const;

			// Copies state information from source array, such as position, momentum,
			// orientation, and force accumulator, and stores within respective fields
			// for the current RigidBody.
			void setState(Real *source);

			// Builds the state derivative array dSdt from current RigidBody state
			// information and stores entries in destination array dst.
      void getStateDeriv(Real *dst) const;

      void setPosition (const Vector2 & position);

      void setPosition (Real xPos, Real yPos);

      void setVelocity (const Vector2 & velocity);

      void setVelocity (Real xComponent, Real yComponent);

      //void setMomentum (const Vector2 & momentum);

      //void setMomentum (Real xComponent, Real yComponent);

      void setMass(const Real &);

      void addToForceAccum(const Vector2 &);

      void zeroForceAccum();

      /* Given a point in graphics coordinate space, this function returns true if
       * the point lies within the convex polygon defined by vertex_array_.*/
      bool pointIsInterior(Real x, Real y);

    protected:
      Vector2 position_;          // Position of center of mass
      Vector2 velocity_;          // Velocity of center of mass
      Vector2 momentum_;          // Total momentum of RigidBody
      Vector2 forceAccumulator_;  // Sum of forces acting on the center of mass of RigidBody
      Real mass_;                 // Total mass
      int vertex_count_;
      unordered_set<Force*> forces_;    // all forces being applied to this RB
      Real *vertex_array_;

  };
}

#endif
