/* GLOBAL INCLUDES */

/*
This is the suscriber node code for the R5-COP monitor.
*/
#include <iostream>
/* LOCAL INCLUDES */
#include "EventInterfaceHandler.h"
#include "Property.h"
/* INCLUDES END */

#include "ros/ros.h"
#include "beginner_tutorials/Parameters.h"

Property property1;

void chatterCallback(const beginner_tutorials::Parameters::ConstPtr& msg)
{
  SR_regtype tempStateReg;
  if(msg->R)
        tempStateReg |= EVENT_A;
  if(msg->D)
        tempStateReg |= EVENT_B;
  if(msg->P)
        tempStateReg |= EVENT_C;
  EventInterfaceHandler::getinstance()->insertEvent(tempStateReg);
  Property::EvaluateROS(&property1);
  ROS_INFO("Incoming: [R:%d][D:%d][P:%d] Result: %s", msg->R, msg->D, msg->P, "no result");
}

int main(int argc, char **argv){
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  Eventhandler::clearEvents();
  EventInterfaceHandler::getinstance()->readEventsFromFile("test1.txt");

  property1.constructChildrenNodeFunc = construct_START;
  construct_START(&property1);  //Bruteforce for the first element

  ros::spin();

  std::cout << Property::Evaluate(&property1);
  getchar();
  return 0;
}
