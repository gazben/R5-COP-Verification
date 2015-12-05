#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
/* LOCAL INCLUDES */
//#include "monitor/EventHandler.h"
#include "monitor/property.h"
/* INCLUDES END */

Property* property1;

void velMessageRecieved(const geometry_msgs::Twist &msg) {

  SR_regtype tempStateReg = 0;
  if(msg.linear.y > 0){
    tempStateReg |= EVENT_UP;
    ROS_INFO_STREAM("UP");
  }

  if(msg.linear.y < 0){
    tempStateReg |= EVENT_DOWN;
    ROS_INFO_STREAM("DOWN");
  }

  if(msg.linear.x > 0){
    tempStateReg |= EVENT_LEFT;
    ROS_INFO_STREAM("LEFT");
  }

  if(msg.linear.x < 0){
    tempStateReg |= EVENT_RIGHT;
    ROS_INFO_STREAM("RIGHT");
  }
  StateRegister::stateRegister = tempStateReg;

  if( property1 == nullptr ){
    property1 = new Property();
    property1->constructChildrenNodeFunc = construct_block0;
    construct_block0(property1);
  }
  property1->Evaluate();

  ROS_INFO_STREAM("---------------");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscribe_to_vel");
  ros::NodeHandle nh;

  StateRegister::clearEvents();
  ros::Subscriber sub = nh.subscribe("turtle1/command_velocity", 1000, &velMessageRecieved);
  ros::spin();

  delete property1;
  return 0;
}
