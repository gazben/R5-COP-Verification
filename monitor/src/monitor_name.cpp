/* LOCAL INCLUDES */
#include "monitor/property.h"

/* INCLUDES END */
#ifndef DEBUG_NO_ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#else
namespace geometry_msgs {
  struct Twist {
    struct linear {
      linear() { clear(); }
      double x, y, z;
      void clear() { x = 0, y = 0, x = 0; }
    }linear;
  };
}
#endif


Property* property1;

void velMessageRecieved(const geometry_msgs::Twist &msg) {
  ROS_INFO_STREAM("-Processing commands-");
  StateRegisterType tempStateReg = 0;

  if (msg.linear.z > 0) {
    tempStateReg |= EVENT_END;
    ROS_INFO_STREAM("END");
  }

  if (msg.linear.y > 0) {
    tempStateReg |= EVENT_UP;
    ROS_INFO_STREAM("UP");
  }

  if (msg.linear.y < 0) {
    tempStateReg |= EVENT_DOWN;
    ROS_INFO_STREAM("DOWN");
  }

  if (msg.linear.x > 0) {
    tempStateReg |= EVENT_LEFT;
    ROS_INFO_STREAM("LEFT");
  }

  if (msg.linear.x < 0) {
    tempStateReg |= EVENT_RIGHT;
    ROS_INFO_STREAM("RIGHT");
  }
  ROS_INFO_STREAM("-Processing commands finished-");

  //This should be moved to somewhere else
  StateRegister::stateRegister = tempStateReg;
  if (property1 == nullptr) {
    property1 = new Property();
    property1->constructChildrenNodeFunc = construct_block0;
    construct_block0(property1);
  }
  property1->Evaluate();
}

int main(int argc, char **argv) {
  StateRegister::clearEvents();

#ifndef DEBUG_NO_ROS
  ros::init(argc, argv, "subscribe_to_vel");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("turtle1/command_velocity", 1000, &velMessageRecieved);
  ros::spin();
#else
  std::string commands = "xrd";
  for (auto& entry : commands) {
    ROS_INFO_STREAM("-Signalling commands-");
    geometry_msgs::Twist msg;
    msg.linear.clear();
    switch (entry) {
    case 'x':
      ROS_INFO_STREAM("DOWN+LEFT");
      msg.linear.y = -1.0;
      msg.linear.x = 1.0;
      break;
    case 'l':
      ROS_INFO_STREAM("LEFT");
      msg.linear.x = 1.0;
      break;
    case 'r':
      ROS_INFO_STREAM("RIGHT");
      msg.linear.x = -1.0;
      break;
    case 'u':
      ROS_INFO_STREAM("UP");
      msg.linear.y = 1.0;
      break;
    case 'd':
      ROS_INFO_STREAM("DOWN");
      msg.linear.y = -1.0;
      break;
    default:
      ROS_INFO_STREAM("UNKNOWN COMMAND");
      break;
    }
    ROS_INFO_STREAM("-Signalling commands finished-");
    velMessageRecieved(msg);
  }
  ROS_INFO_STREAM("END_OF_STREAM");
  geometry_msgs::Twist end_msg;
  end_msg.linear.z = 1;
  velMessageRecieved(end_msg);
#endif
  getchar();
  delete property1;
  return 0;
}