#include "RigidBodySystem.h"
#include "Common/RigidException.h"

using namespace std;

namespace Rigid2D {

  RigidBodySystem::RigidBodySystem() 
  {
    time_ = 0.0;
  }


  RigidBodySystem::~RigidBodySystem() 
  {
    // look up what to do with rigidBodies_
  }


  void RigidBodySystem::update()
  {
    unordered_set<RigidBody*>::iterator it;
    for (it = rigidBodies_.begin(); it != rigidBodies_.end(); ++it)
    {
      it->update();
    }
  }


  void RigidBodySystem::addRigidBody(RigidBody *rigidBody) 
  {
    assert(rigidBody != NULL);
    rigidBodies_.insert(rigidBody);
  }


  void RigidBodySystem::addRigidBodies(RigidBody **rigidBodyArray, unsigned int numBodies) 
  {
    for (unsigned int i = 0; i < numBodies; ++i) {
      rigidBodies_.insert(rigidBodyArray[i]);
    }
  }


  void RigidBodySystem::removeRigidBody(RigidBody *rigidBody) 
  {
    rigidBodies_.erase(rigidBody);
  }


  void RigidBodySystem::removeRigidBodies(RigidBody **rigidBodyArray, unsigned int numBodies) 
  {
    for (unsigned int i = 0; i < numBodies; ++i) {
      rigidBodies_.erase(rigidBodyArray[i]);
    }
  }

}
