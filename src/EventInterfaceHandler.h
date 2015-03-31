#ifndef EventInterfaceHandler_h__
#define EventInterfaceHandler_h__

/*
This file will contain the handling of Events, accured in the system we want to observe.
If an event happens in the system, it will trigger a function here.
*/

/* GLOBAL INCLUDES */

/* LOCAL INCLUDES */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "EventHandler.h"
/* INCLUDES END */

void monitorCallback(const std_msgs::String::ConstPtr& msg){
  printf("Callback happened!");

  /*  */
}

#endif // EventInterfaceHandler_h__
