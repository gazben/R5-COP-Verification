#ifndef EventHandler_h__
#define EventHandler_h__

/* GLOBAL INCLUDES */
#include <stdio.h>

/* LOCAL INCLUDES */
#include "StateRegister.h"
#include "OutputState.h"
#include "Events.h"

/* INCLUDES END */

/* TEST FUNCTIONS */
class Eventhandler{
public:
  static void addEvent(SR_regtype event_code){
    StateRegisterState::stateRegister = StateRegisterState::stateRegister | event_code;
  }

  static void removeEvent(SR_regtype event_code){
    StateRegisterState::stateRegister = StateRegisterState::stateRegister ^ event_code;
  }

  static void clearEvents(){
    StateRegisterState::stateRegister = 0;
  }
};

#endif // EventHandler_h__
