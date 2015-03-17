#ifndef EventHandler_h__
#define EventHandler_h__

/* GLOBAL INCLUDES */

/* LOCAL INCLUDES */
#include "StateRegister.h"
#include "OutputState.h"
#include "Events.h"

/* INCLUDES END */

/* TEST FUNCTIONS */
void addEvent(SR_regtype event_code){
  stateRegister = stateRegister | event_code;
}

void removeEvent(SR_regtype event_code){
  stateRegister = stateRegister ^ event_code;
}
#endif // EventHandler_h__
