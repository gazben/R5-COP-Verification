/* GLOBAL INCLUDES */

/*
This is the suscriber node code for the R5-COP monitor.
*/
#include <iostream>

/* LOCAL INCLUDES */
#include "EventInterfaceHandler.h"
#include "Property.h"
/* INCLUDES END */

int main(){
  Eventhandler::clearEvents();
  Eventhandler::addEvent(EVENT_A);
  EventInterfaceHandler::getinstance()->readEventsFromFile("test1.txt");
  
  Property property1(1, 2);
  property1.constructChildrenNode = constructS0;

  std::cout << Property::Evaluate(&property1);
  getchar();
  return 0;
}