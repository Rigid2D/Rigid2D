#ifndef RIGID2D_RIGID_BODY_SYSTEM_H
#define RIGID2D_RIGID_BODY_SYSTEM_H

#include "Common/RigidSettings.h"
#include "Collision/NarrowPhase.h"
#include "Objects/RigidBody.h"
#include <unordered_set>
#include <vector>

namespace Rigid2D
{
  class RigidBodySystem
	{
    public:
      RigidBodySystem();
      ~RigidBodySystem();

      /** Calls each RigidBody's update method.  Each Rigid Body is moved
       * sequentially one at a time.  This speeds up the simulation at the cost
       * of some small physical inaccuracies.
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
      void addRigidBody(RigidBody *rigidBody);

      /**
       * Same as addRigidBody() but for multiple bodies.
       *
       * @param rigidBodyArray array of RigidBody object pointers.
       * @param numBodies length of rigidBodyArray.
       * @see addRigidBody()
       * @see removeRigidBody()
       * @see removeRigidBodies()
       */
      void addRigidBodies(RigidBody **rigidBodyArray, unsigned int numBodies);

      /** Tells RigidBodySystem to not keep track of a RigidBody. If the body
       * (pointer) was not previously added, it does nothing.
       *
       * @param rigidBody pointer to a RigidBody object.
       * @see addRigidBody()
       * @see addRigidBodies()
       * @see removeRigidBodies()
       */
      void removeRigidBody(RigidBody *rigidBody);

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

      std::vector<Contact*> * getContacts();
			/// Perform collision detection between all pairs of rigid bodies.
      void checkCollision();

    private:
      std::unordered_set<RigidBody*> rigidBodies_;   // Collection of all tracked rigid bodies
      std::vector<Contact*> contacts_;               // All contacts after checkCollision runs
      Real time_;                                    // Simulation clock
	};

}

#endif
