#include "RigidBodySystem.h"
#include "Common/RigidException.h"
#include "Dynamics/CollisionResponse.h"

using namespace std;

namespace Rigid2D {

  RigidBodySystem::RigidBodySystem()
  {
    time_ = 0.0;
  }


  RigidBodySystem::~RigidBodySystem()
  {
    // Delete rigidBodies_ here once we have a memory pool working.
  }


  void RigidBodySystem::update()
  {
    // clear contacts from previous frame (this should later be done by collision response code)
    contacts_.clear();

    unordered_set<RigidBody*>::iterator it;
    for (it = rigidBodies_.begin(); it != rigidBodies_.end(); ++it) {
      (*it)->update();
    }
    checkCollision();

    std::vector<Contact*>::iterator contact_it;
    for (contact_it = contacts_.begin(); contact_it < contacts_.end(); contact_it++) {
      resolveCollision(**contact_it);
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

  std::vector<Contact*> * RigidBodySystem::getContacts()
  {
    return &contacts_;
  }

	// Perform collision detection between all pairs of rigid bodies without
	// duplicating work.  Given n Rigid Bodies, we call RigidBody::checkCollision()
	// "n choose 2" times.
  // This function generates all the Contact structure needed to handle collision response.
  void RigidBodySystem::checkCollision()
  {
    Contact *contact = new Contact;
    unordered_set<RigidBody*>::iterator it1;
    unordered_set<RigidBody*>::iterator it2;
    for (it1 = rigidBodies_.begin(); it1 != rigidBodies_.end(); ++it1) {
      it2 = it1;
      it2++;
      for ( ; it2 != rigidBodies_.end(); ++it2) {
        if ( (*it1)->checkCollision(*it2, contact) ) {
          // Collision Detected
          contacts_.push_back(contact);
          contact = new Contact;
        }
      }
    }
    delete contact;
  }

}
