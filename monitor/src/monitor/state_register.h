#ifndef StateRegister_h__
#define StateRegister_h__

/* GLOBAL INCLUDES */
#include <stdlib.h>

/* LOCAL INCLUDES */
#include "gen_events.h"
/* INCLUDES END */

class StateRegister {
private:
  static StateRegister * rootState;
  StateRegisterType stateRegisterValue;
  StateRegister * rightNode;
  StateRegister * leftNode;

  static StateRegister * insertState(StateRegisterType stateReg = stateRegister, StateRegister * root = rootState);

public:

  StateRegister();
  ~StateRegister();

  //Global stateRegister
  static StateRegisterType stateRegister;
  static void clearEvents();
  static bool isEventCurrentlyFired(StateRegisterType eventCode);
  static void freeState(StateRegister *root = rootState);

  static StateRegister * getStatePointer(StateRegisterType StateRegisterCopy = stateRegister);

  friend class Property;
};

#endif // StateRegister_h__
