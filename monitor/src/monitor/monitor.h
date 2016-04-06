#ifndef monitor_h__
#define monitor_h__

#include <vector>
#include <string>

#include "monitor/gen_blocks.h"

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

//Keep track of all registered event handlers
class EventInterface {
private:
  static std::vector<EventInterface*> event_handlers;

protected:
  std::string topic_name;

public:

  static std::vector<EventInterface*>& getEventHandlers() {
    return event_handlers;
  }

  const std::string& getTopicName() {
    return topic_name;
  }

  EventInterface() {
    event_handlers.push_back(this);
  }

};


Property* property1;


/*

  StateRegister::clearEvents();

#ifndef DEBUG_NO_ROS
  ros::init(argc, argv, "subscribe_to_vel");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("turtle1/command_velocity", 1000, &velMessageRecieved);
  ros::spin();
#else
  std::string commands = "xrds";
  for (auto& entry : commands) {
    ROS_INFO_STREAM("-Signalling commands-");
    geometry_msgs::Twist msg;
    msg.linear.clear();
    switch (entry) {
    case 's':
      ROS_INFO_STREAM("STOP");
      msg.linear.z = 1.0;
      break;
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
  getchar();
#endif
*/

class Monitor {

public:
  static trilean evaluate(StateRegisterType event) {
    StateRegister::stateRegister = event;
    if (property1 == nullptr) {
      property1 = new Property();
      property1->construct_children_node_func = construct_block0;
      construct_block0(property1);
    }

    return FALSE;
  }

  static void run(int argc, char **argv) {
    ros::init(argc, argv, "subscribe_to_vel");
    ros::NodeHandle nh;

    /* --SUBSCRIBE--  */
    for (auto entry : EventInterface::getEventHandlers()) {
      nh.subscribe(entry->getTopicName(), 1000, &velMessageRecieved);
    }

    ros::spin();  //the point of no return
  }
};


#endif // monitor_h__
