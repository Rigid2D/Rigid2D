#include "Force.h"

using namespace std;

namespace Rigid2D {

  Force::Force(ForceFunctionPtr forceFunction, void *userData)
    : userData_(userData),
      enabled_(true),
      forceFunction_(forceFunction)
  {
    // nothing to be done
  }

  Force::~Force() {}

  void Force::computeForce(RigidBody * const rb, RBState *state, Vector2 *result)
  {
    forceFunction_(rb, state, result, userData_);
  }

  void Force::setForceFunction(ForceFunctionPtr funct)
  {
    forceFunction_ = funct;
  }

  bool Force::isEnabled()
  {
    return enabled_;
  }

  void Force::setEnabled(bool trueOrFalse)
  {
    enabled_ = trueOrFalse;
  }

  void Force::setUserData(void * userData)
  {
    userData_ = userData;
  }
}
