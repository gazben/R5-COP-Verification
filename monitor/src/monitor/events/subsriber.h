#ifndef subscriber_h__
#define subscriber_h__

#include <string>

#ifndef DEBUG_NO_ROS
#include <geometry_msgs/Twist.h>
#endif

/*
 * This is a skeleton of an event handler.
 * Feel free, to write plus functions, but do not delete anything that was predefined!
 */


//////////////////////////////////////////////////////////////////////////
//  Event hadling code 
ros::NodeHandle node_handler;

void messageRecieved(geometry_msgs::Twist msg) {
  ROS_INFO_STREAM("");
  StateRegisterType event = 0;

  if (msg.linear.z > 0) {
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
      event |= EVENT_DOWN;
      ROS_INFO_STREAM("LEFT");
    }

    if (msg.linear.x < 0) {
      event |= EVENT_RIGHT;
      ROS_INFO_STREAM("RIGHT");
    }

    Monitor::evaluate(event);
  }
}


//////////////////////////////////////////////////////////////////////////
// Init
void subscriber_init() {
  //Init code comes here
  //Runs before subscriber_subsribe()
}


//////////////////////////////////////////////////////////////////////////
// Subscribe
void subscriber_subsribe() {
  node_handler.subscribe("turtle1/command_velocity", 1000, &messageRecieved);
}

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
