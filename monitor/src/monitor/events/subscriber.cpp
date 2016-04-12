#include <string>

#include "../monitor.h"

/*
 * This is a skeleton of an event handler.
 * Feel free, to write plus functions, but do not delete anything that was predefined!
 */


//////////////////////////////////////////////////////////////////////////
//  Event hadling code
ros::NodeHandle* node_handler = nullptr;

void messageRecieved(const geometry_msgs::Twist& msg) {
  ROS_INFO_STREAM("we are here");
  StateRegisterType event = 0;

  if (msg.linear.z > 0) {
    event |= event_r;
    ROS_INFO_STREAM("END");
  }

  if (msg.linear.y > 0) {
    event |= event_r;
    ROS_INFO_STREAM("UP");
  }

  if (msg.linear.y < 0) {
    event |= event_r;
    ROS_INFO_STREAM("DOWN");
  }

  if (msg.linear.x > 0) {
    event |= event_r;
    ROS_INFO_STREAM("LEFT");
  }

  if (msg.linear.x < 0) {
    event |= event_r;
    ROS_INFO_STREAM("RIGHT");
  }

  Monitor::evaluate(event);
}


//////////////////////////////////////////////////////////////////////////
// Init
void Monitor::subscriber_init() {
  ROS_INFO_STREAM("SUBSRIBER INIT...");
  node_handler = new ros::NodeHandle();
  //Init code comes here
  //Runs before subscriber_subsribe()
}

void Monitor::subscriber_deinit(){
  ROS_INFO_STREAM("SUBSRIBER DEINIT...");
  delete node_handler;
}


//////////////////////////////////////////////////////////////////////////
// Subscribe
void Monitor::subscriber_subsribe() {
  ROS_INFO_STREAM("SUBSRIBER SUBSCRIBE...");
  node_handler.subscribe("turtle1/cmd_vel", 1000, &messageRecieved);
}

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
