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


//////////////////////////////////////////////////////////////////////////
// Subscribe
void Monitor::subscriber_subsribe() {
  ros::NodeHandle nodeHandle;

  //TODO subscribe to publishers

  ros::Subscriber sub = nodeHandle.subscribe("turtle1/cmd_vel", 1000, &messageRecieved);
  ros::spin();
}


//////////////////////////////////////////////////////////////////////////
// Init
void Monitor::subscriber_init(){
  ROS_INFO_STREAM("SUBSRIBER initializing");
  //TODO init custom classes/variables
}

//Called during monitor exit
void Monitor::subscriber_deinit(){
  ROS_INFO_STREAM("SUBSRIBER deinitializing");
  //TODO free all variables
}

//////////////////////////////////////////////////////////////////////////
// Result handling

void Monitor::true_action(){
  ROS_INFO_STREAM("Executing given command: " + true_command);
  system(true_command.c_str());
  //TODO what should happen on TRUE result?
}

void Monitor::false_action(){
  ROS_INFO_STREAM("Executing given command: " + false_command);
  system(false_command.c_str());
  //TODO what should happen on FALSE result?
}

//Handler example:
/*
void messageRecieved(const geometry_msgs::Twist& msg){
  std::string event_log = "EVENT_RECIEVED: ";
  StateRegisterType event = 0;

  if (msg.linear.z > 0) {
    event |= EVENT_END;
    event_log += " END";
  }

  if (msg.linear.y > 0) {
    event |= event_r;
    event_log += " UP";
  }

  if (msg.linear.y < 0) {
    event |= event_d;
    event_log += " DOWN";
  }

  if (msg.linear.x < 0) {
    event |= event_r;
    event_log += " LEFT";
  }

  if (msg.linear.x > 0) {
  event |= event_p;
  event_log += " RIGHT";
  }

  ROS_INFO_STREAM(event_log);
  Monitor::getInstance().evaluate(event);
}
 */
