#ifndef EventHandler_h__
#define EventHandler_h__

/* GLOBAL INCLUDES */

/* LOCAL INCLUDES */
#include "StateRegister.h"
#include "OutputState.h"
#include "Events.h"

/* INCLUDES END */

/* TEST FUNCTIONS */
void EVENT_addEvent(SR_regtype event_code){
  stateRegister = stateRegister | event_code;
}

void EVENT_removeEvent(SR_regtype event_code){
  stateRegister = stateRegister ^ event_code;
}

void EVENT_clearEvents(){
  stateRegister = 0;
}
#endif // EventHandler_h__
