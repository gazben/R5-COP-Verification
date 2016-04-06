#ifndef event_name_event_h__
#define --event_name--_event_h__

#include <string>

#include "..\monitor.h"


class --event_name--_event : public EventInterface {
public: 
  --event_name--_event() {
    topic_name = ""; 
  }


  static void messageRecieved(/*  */ msg) {
    ROS_INFO_STREAM("");
    StateRegisterType event;

    if (msg.linear.z > 0) {
      tempStateReg = EVENT_END;
      ROS_INFO_STREAM("--event_name-- occoured");
    }

    Monitor::evaluate(event);
  }
};

#endif 
