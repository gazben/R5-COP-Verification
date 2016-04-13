#ifndef monitor_h__
#define monitor_h__

#include <string>

/* INCLUDES END */
#ifndef DEBUG_NO_ROS
#include <string>
#include <ros/ros.h>
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
/* INCLUDES END */


#include "gen_events.h"
#include "gen_blocks.h"


//Class that will do the evaluation
class Monitor {
private:
    static Monitor* instance;
    Property* property1; //root property pointer
    Monitor();
    ~Monitor();
public:
    trilean evaluate(StateRegisterType event);
    void run();
    void subscriber_init();
    void subscriber_deinit();
    static Monitor* getInstance();
};

#endif // monitor_h__
