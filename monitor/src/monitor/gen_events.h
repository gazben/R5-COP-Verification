#ifndef Events_h__
#define Events_h__

typedef unsigned long long int StateRegisterType;

//dedicated end event
const StateRegisterType EVENT_END = 1;

//--EVENTS--

#ifdef DEBUG_NO_ROS
//Example
const StateRegisterType EVENT_UP = 2;
const StateRegisterType EVENT_DOWN = 4;
const StateRegisterType EVENT_RIGHT = 8;
#endif

#endif // Events_h__