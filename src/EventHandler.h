#ifndef EventHandler_h__
#define EventHandler_h__

/* GLOBAL INCLUDES */
#include <stdio.h>

/* LOCAL INCLUDES */
#include "StateRegister.h"
#include "OutputState.h"
#include "Events.h"

#include "LineReader.h"

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

void EVENT_getTestEvents(){
  EVENT_clearEvents();
}

/* EVENT STREAM FUNCTIONS */
void EVENT_initStateReader(char* fileName){
  FILE* fp = NULL;
  fp = fopen(fileName, "rt");

  LR_init(&LR_eventReader, fp);
}

void EVENT_deinitStatereader(){
  LR_freeLineReader(&LR_eventReader);
}
SR_regtype EVENT_readNextState(){
  SR_regtype tempStateReg = 0;
  size_t lineSize = 0;
  char* nextLine = NULL;

  nextLine = LR_nextLine(&LR_eventReader, &lineSize);

  char c;
  for (int i = 0; i < lineSize; i++){
    c = nextLine[i];

    switch (c)
    {
    case 'a':
      tempStateReg |= EVENT_A;
      break;
    case 'b':
      tempStateReg |= EVENT_B;
      break;
    case 'c':
      tempStateReg |= EVENT_C;
      break;
    case 'd':
      tempStateReg |= EVENT_D;
      break;
    case '\n':
      break;
    default:
      printf("Invalid state found in testfile!\n");
      break;
    }
  }

  SR_setStateRegister(tempStateReg);
  return tempStateReg;
}

#endif // EventHandler_h__
