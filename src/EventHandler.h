#ifndef EventHandler_h__
#define EventHandler_h__

/* GLOBAL INCLUDES */

/* LOCAL INCLUDES */
#include "StateRegister.h"
#include "OutputState.h"

/* INCLUDES END */

/* TEST FUNCTIONS */
void addEvent(unsigned long long int event_code){
  stateRegister = stateRegister | event_code;
}

void removeEvent(unsigned long long int event_code){
  stateRegister = stateRegister ^ event_code;
}

OutputState getEvent(unsigned long long int stateRegisterCopy, unsigned long long int event_code){
  return (stateRegisterCopy & event_code) ? TRUE : FALSE;
}

#endif // EventHandler_h__
