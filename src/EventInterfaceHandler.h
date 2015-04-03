#ifndef EventInterfaceHandler_h__
#define EventInterfaceHandler_h__

/*
This file will contain the handling of Events, accured in the system we want to observe.
If an event happens in the system, it will trigger a function here.
*/

/* GLOBAL INCLUDES */
#include <fstream>
#include <deque>
#include <string>
/* LOCAL INCLUDES */
#include "EventHandler.h"
/* INCLUDES END */

class EventInterfaceHandler{
private:
  SR_regtype readNextLine(){
    std::string line;
    SR_regtype tempStateReg;
    std::getline(eventFile, line);

    for (char c : line){
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
      return tempStateReg;
    }
  }

protected:
  std::ifstream eventFile;
  std::deque<SR_regtype> eventQueue;

public:

  ~EventInterfaceHandler(){
    if (eventFile.is_open())
      eventFile.close();
  }

  SR_regtype getNextEvent(){
    if (eventFile.is_open())
      eventQueue.push_back(readNextLine());

    SR_regtype front = eventQueue.front();
    eventQueue.pop_front();
    return front;
  }

  void readEventsFromFile(std::string filename){
    eventFile.open(filename);
  }
  
};

#endif // EventInterfaceHandler_h__
