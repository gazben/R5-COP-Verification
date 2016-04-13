#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "../monitor.h"

/*
 * This is a skeleton of an event handler. Feel free, to write plus functions,
 * but do not delete anything that was predefined!
 *
 * You can use all the LTL variables you declared when using the generator
 */


//////////////////////////////////////////////////////////////////////////
//  Event hadling

// TODO write event handlers
// tutorial: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29


void messageRecieved(const geometry_msgs::Twist& msg){
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

  Monitor::getInstance()->evaluate(event);
}


//////////////////////////////////////////////////////////////////////////
// Subscribe
void subscriber_subsribe() {
  ros::NodeHandle nodeHandle;

  //TODO subscribe to publishers

  nodeHandle.subscribe("turtle1/cmd_vel", 1000, &messageRecieved);
}


//////////////////////////////////////////////////////////////////////////
// Init
void Monitor::subscriber_init(){
  ROS_INFO_STREAM("SUBSRIBER INIT...");

  //TODO init custom classes/variables

  subscriber_subsribe();
}

//Called during monitor exit
void Monitor::subscriber_deinit(){
  ROS_INFO_STREAM("SUBSRIBER DEINIT...");
}


//Handler example:
/*
void messageRecieved(const geometry_msgs::Twist& msg){
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

  Monitor::getInstance()->evaluate(event);
}
 */
