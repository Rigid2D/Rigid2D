#ifndef RIGID2D_FORCE_H
#define RIGID2D_FORCE_H
#include "Common/RigidSettings.h"
#include "Common/Vector2.h"
#include "Objects/RigidBody.h"
#include <unordered_set>

namespace Rigid2D {
  using namespace std;
  /// A wrapper around a ForceFunctionPtr with an additional userData field.
  /// userData is a void pointer to any user specified data to be used inside
  /// the ForceFunction.

  class Force {
    public:
      typedef
      void (*ForceFunctionPtr)(RigidBody * const rigidBody,    // Memory location for a particular
                                                               // RigidBody that the force is acting
                                                               // upon.

                               Real * state,                   // RB state vector (position, momentum, ...)

                               Vector2 * dst,                  // Destination for storing force
                                                               // components.

                               void * userData);               // User specific data which can be
                                                               // used in computing forces.

      Force(ForceFunctionPtr forceFunction, void * userData = 0);
      ~Force();

      // Calls the ForceFunction with the parameters specified.
      // @state is the array containing position, momentum, etc.
      // @results a two dimensional array containing the resultant 
      // force.x and force.y.
      void computeForce(RigidBody * const rb, RBState *state, Real *result);

      // Set funct as the new ForceFunctionPtr
      void setForceFunction(ForceFunctionPtr funct);

      // Returns true if force is enabled and false otherwise.
      bool isEnabled();

      // Enable current Force object by passing true, or disable it by passing false.
      // If the Force object is diabled, it will not act upon its current list of
      // RigidBodies until it is enabled again.
      void setEnabled(bool trueOrFlase);

      // Set the value of the userData_ pointer
      void setUserData(void * userData);

    protected:
      void * userData_; // Pointer to additional data the user wants available 
                        // in the force function.
      bool enabled_;    // A flag to turn on and off the force.
      ForceFunctionPtr forceFunction_;
  };
} // end namespace Rigid2D

#endif
