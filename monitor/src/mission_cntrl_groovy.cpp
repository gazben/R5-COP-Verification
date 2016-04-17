#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

class Turtle {
public:
    Turtle(){
      publisher = nodeHandle.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    }
    void signalCommands();

private:
    ros::NodeHandle nodeHandle;
    ros::Publisher publisher;
};

//command string; l=left, u=up, r=right, d=down, s=stop, w=wait, x=up+right
std::string commands = "xrds";

void Turtle::signalCommands() {
  sleep(1); //monitor init time

  for (auto& entry : commands) {
    ROS_INFO_STREAM("-Signalling commands-");
    geometry_msgs::Twist msg;
    switch (entry) {
      case 's':
        ROS_INFO_STREAM("STOP");
        msg.linear.z = 1.0;
        break;
      case 'x':
        ROS_INFO_STREAM("UP+RIGHT");
        msg.linear.y = 1.0;
        msg.linear.x = 1.0;
        break;
      case 'l':
        ROS_INFO_STREAM("LEFT");
        msg.linear.x = -1.0;
        break;
      case 'r':
        ROS_INFO_STREAM("RIGHT");
        msg.linear.x = 1.0;
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

    publisher.publish(msg);
    sleep(1);
  }
  sleep(10);

  return;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "turtle");
  Turtle turtle;
  turtle.signalCommands();
  ros::shutdown();
  return 0;
}