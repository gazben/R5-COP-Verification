#ifndef monitor_h__
#define monitor_h__

#include <string>

#include "gen_events.h"
#include "gen_blocks.h"
#include "events/subsriber.h"

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

//Class that will do the evaluation
class Monitor {
private:
  static Property* property1; //root property pointer
public:
  static ros::NodeHandle& getNodeHandler();
  static trilean evaluate(StateRegisterType event);
  static void run(int argc, char **argv);
};


#endif // monitor_h__
