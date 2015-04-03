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

  Property property1;
  property1.constructChildrenNodeFunc = constructS0;
  constructS0(&property1);  //Bruteforce for the first element

  std::cout << Property::Evaluate(&property1);
  getchar();
  return 0;
}