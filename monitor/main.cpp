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
  EventInterfaceHandler::getinstance()->readEventsFromFile("test1.txt");

  Property property;
  property.constructChildrenNodeFunc = construct_START;
  construct_START(&property);  //Bruteforce for the first element

  std::cout << Property::Evaluate(&property);
  getchar();
  return 0;
}