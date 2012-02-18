#ifndef RIGID2D_RIGID_BODY_SYSTEM_H
#define RIGID2D_RIGID_BODY_SYSTEM_H

#include "Common/RigidSettings.h"
#include "Common/OdeSolver.h"
#include "RigidBody.h"
#include "Force.h"

namespace Rigid2D
{
  class RigidBodySystem
	{
    public:
      RigidBodySystem();
      ~RigidBodySystem();

      /** Uses OdeSolver to update state information for each RigidBody associated
       * with the current RigidBodySystem.
       */
      void update();

      /**
       * Tells RigidBodySystem to keep track of a RigidBody. If the body (pointer)
       * was previously added, it does not get added a second time.
       *
       * @param rigidBody pointer to a RigidBody object.
       * @see addRigidBodies()
       * @see removeRigidBody()
       * @see removeRigidBodies()
       */
      void addRigidBody(RigidBody * rigidBody);

      /**
       * Same as addRigidBody() but for multiple bodies.
       *
       * @param rigidBodyArray array of RigidBody object pointers.
       * @param numBodies length of rigidBodyArray.
       * @see addRigidBody()
       * @see removeRigidBody()
       * @see removeRigidBodies()
       */
      void addRigidBodies(RigidBody ** rigidBodyArray, unsigned int numBodies);

      /** Tells RigidBodySystem to not keep track of a RigidBody. If the body
       * (pointer) was not previously added, it does nothing.
       *
       * @param rigidBody pointer to a RigidBody object.
       * @see addRigidBody()
       * @see addRigidBodies()
       * @see removeRigidBodies()
       */
      void removeRigidBody(RigidBody * rigidBody);

      /**
       * Same as removeRigidBody() but for multiple bodies.
       *
       * @param rigidBodyArray array of RigidBody object pointers.
       * @param numBodies length of rigidBodyArray.
       * @see addRigidBody()
       * @see addRigidBodies()
       * @see removeRigidBody()
       */
      void removeRigidBodies (RigidBody **rigidBodyArray, unsigned int numBodies);

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

			/** TODO: add documentation
			 */
      unsigned int getDimension();

    private:
      // Computes the derivative dS/dt, from the given inputs:
      // t - simulation time
      // S - system state array
      void computeStateDeriv(Real t, const Real * S, Real * dSdt);

      // Iterates through each RigidBody collecting state information and appending
			// it to S_
      void buildSystemStateArray();

      // Set the forceAccumulator field for each RigidBody in the system to zero.
      void clearForceAccumulators();

      // Update each RigidBody in this system with the state information contained in
      // the system state array S_.
      void updateRigidBodies();

      unordered_set<RigidBody*> rigidBodies_;   // Collection of all tracked rigid bodies
      unordered_set<Force*> forces_;            // Collection of all forces
      Real * S_;                                // state array S, representing a collection of
                                                // all RigidBody state information

      PreciseReal time_;                        // Simulation clock
      OdeSolver *solver_;                       // Used to solve the system dS/dt = G(t,S)

      int systemDimension_;                     // Dimension of the system.  This is the
                                                // number of RigidBodies*4, where each RigidBody
                                                // has 4 states: position, momentum, orientation,
                                                // and angular momemtum.

      int statesPerRigidBody_;                  // Number of states we store per Rigid Body.
	};

} // end namespace Rigid2D

#endif
