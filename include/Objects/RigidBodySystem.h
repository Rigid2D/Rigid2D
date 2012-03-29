#ifndef RIGID2D_RIGID_BODY_SYSTEM_H
#define RIGID2D_RIGID_BODY_SYSTEM_H

#include "Common/RigidSettings.h"
#include "Common/OdeSolver.h"
#include "Objects/RigidBody.h"
#include <unordered_set>

namespace Rigid2D
{
  class RigidBodySystem
	{
    public:
      RigidBodySystem();
      ~RigidBodySystem();

      /** Calls each RigidBody's update method */
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

    private:
      std::unordered_set<RigidBody*> rigidBodies_;   // Collection of all tracked rigid bodies
      Real time_;  // Simulation clock
	};

}

#endif
