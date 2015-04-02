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
  Property property1;
  
  std::cout << property1.Eval();  
  getchar();
  return 0;
}