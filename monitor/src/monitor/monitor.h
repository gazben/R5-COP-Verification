#ifndef monitor_h__
#define monitor_h__

#include <vector>
#include <string>

#include "gen_blocks.h"

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
    static ros::NodeHandle node_handler;
public:
    EventInterface(){
        event_handlers.push_back(this);
    }
    const std::string getTopicName();
};

//Class that will do the evaluation
class Monitor {
private:
    static Property* property1; //root property pointer
public:
    static trilean evaluate(StateRegisterType event);
    static void run(int argc, char **argv);
};


#endif // monitor_h__
