#ifndef StateRegister_h__
#define StateRegister_h__

/* GLOBAL INCLUDES */
#include <stdlib.h>

/* LOCAL INCLUDES */
#include "gen_events.h"
/* INCLUDES END */

class StateRegister{
private:
  static StateRegister* root_state;
  static StateRegister* insertState(StateRegisterType stateReg=state_register, StateRegister* root=root_state);

  StateRegisterType state_register_value;
  StateRegister* right_node;
  StateRegister* left_node;  
public:
  StateRegister();
  ~StateRegister();

  static StateRegisterType state_register;
  static void clearEvents();
  static bool isEventCurrentlyFired(StateRegisterType event_code);
  static void freeState(StateRegister* root=root_state);
  static StateRegister* getStatePointer(StateRegisterType state_register_copy=state_register);

  friend class Property;
};

#endif // StateRegister_h__
