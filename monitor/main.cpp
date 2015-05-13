/* GLOBAL INCLUDES */

/*
This is the suscriber node code for the R5-COP monitor.
*/
#include <iostream>
/* LOCAL INCLUDES */
#include "EventInterfaceHandler.h"
#include "Property.h"
/* INCLUDES END */

#include <ros/ros.h>
#include <ros/param.h>
#include <rosconsole/macros_generated.h>
#include <ros/init.h>
#include <ros/message.h>
#include <std_msgs/String.h>
/* INCLUDES END */

Property property;

void chatterCallback(const std_msgs::String& msg)
{
  SR_regtype tempStateReg;
  /*
  if (msg->R)
    tempStateReg |= EVENT_A;
  if (msg->D)
    tempStateReg |= EVENT_B;
  if (msg->P)
    tempStateReg |= EVENT_C;
    */
  EventInterfaceHandler::getinstance()->insertEvent(tempStateReg);
  Property::EvaluateROS(&property);
  
  //ROS_INFO("Incoming: [R:%d][D:%d][P:%d] Result: %s", msg->R, msg->D, msg->P, "no result");
  ROS_INFO( msg.data.c_str() );
}

int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  Eventhandler::clearEvents();
  //EventInterfaceHandler::getinstance()->readEventsFromFile("test1.txt");


  property.constructChildrenNodeFunc = construct_START;
  construct_START(&property);  //Bruteforce for the first element

  ros::spin();

  std::cout << Property::Evaluate(&property);
  getchar();
  return 0;
}