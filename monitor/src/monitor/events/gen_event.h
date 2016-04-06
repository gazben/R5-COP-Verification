#ifndef --event_name--_event_h__
#define --event_name--_event_h__

#include <string>

#include "../monitor.h"


class --event_name--_event : public EventInterface {
public: 
  --event_name--_event() {
    topic_name = "";
    node_handler.subscribe(topic_name, 1000, &messageRecieved);
  }


  static void messageRecieved(/*  */ msg) {
    ROS_INFO_STREAM("");
    StateRegisterType event = 0;

    if (msg.linear.z > 0) {
      tempStateReg = EVENT_END;
      ROS_INFO_STREAM("--event_name-- occoured");
    }

    Monitor::evaluate(event);
  }
};

#endif 
