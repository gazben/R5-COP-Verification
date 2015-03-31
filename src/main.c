/* GLOBAL INCLUDES */

/*
This is the suscriber node code for the R5-COP monitor.
*/
#include <stdio.h>

/* LOCAL INCLUDES */
#include "ros/ros.h"
#include "EventInterfaceHandler.h"
#include "PropertyHandler.h"
/* INCLUDES END */

OutputState result;


int main(){

  ros::init(argc, argv, "listener");

  Property* property1Root = NULL;
  property1Root = PROP_constructS0(property1Root);
  EVENT_clearEvents();

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("uRandomTalker/randomPoints", 1000, monitorCallback);


  ros::spin();
  


  result = PROP_evaluateProperty(property1Root);
  printf("The result of the rule checking: %s\n", OS_tostring(result));
  
  getchar();
  return 0;
}