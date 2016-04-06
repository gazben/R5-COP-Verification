#ifndef --event_name--_event_h__
#define --event_name--_event_h__

#include <string>

#include "../monitor.h"

/*
 * This is a skeleton of an event handler.
 * Feel free, to write plus functions, but do not delete anything that was predefined!
 */

class --event_name--_event : public EventInterface {
public: 
  --event_name--_event() {
    topic_name = "";  //turtle1/command_velocity
    node_handler.subscribe(topic_name, 1000, &messageRecieved);
  }


  void messageRecieved(/* message type */ msg) {  //geometry_msgs::Twist
    ROS_INFO_STREAM("");
    StateRegisterType event = 0;

    if (msg.linear.z > 0) {
      tempStateReg = EVENT_END;
      ROS_INFO_STREAM("--event_name-- occoured");
    }

    Monitor::evaluate(event);
  }
};
--event_name--_event --event_name--_inst; //do not delete!

#endif

//Handler example:
/*
  if (msg.linear.z > 0) {
    event |= EVENT_END;
      ROS_INFO_STREAM("END");
  }

  if (msg.linear.y > 0) {
    event |= EVENT_UP;
    ROS_INFO_STREAM("UP");
  }

  if (msg.linear.y < 0) {
    event |= EVENT_DOWN;
    ROS_INFO_STREAM("DOWN");
  }

  if (msg.linear.x > 0) {
    event |= EVENT_LEFT;
    ROS_INFO_STREAM("LEFT");
  }

  if (msg.linear.x < 0) {
    event |= EVENT_RIGHT;
    ROS_INFO_STREAM("RIGHT");
  }
 */
