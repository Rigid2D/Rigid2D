#include "Force.h"

using namespace std;

namespace Rigid2D {

  Force::Force(ForceFunctionPtr forceFunction, void * userData = 0)
    : userData_(userData),
      enabled_(true),
      forceFunction_(forceFunction)
  {
    // nothing to be done
  }

  void Force::computeForce(RigidBody * const rb, RBState *state, Vector2 & result)
  {
    forceFunction_(rb, state, result, userData_);
  }

  void setForceFunction(ForceFunctionPtr funct)
  {
    forceFunction_ = funct;
  }

  bool isEnabled()
  {
    return enabled_;
  }

  void setEnabled(bool trueOrFlase)
  {
    enabled_ = trueOrFalse;
  }

  void setUserData(void * userData)
  {
    userData_ = userData;
  }
}
