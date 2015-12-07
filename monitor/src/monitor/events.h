#ifndef Events_h__
#define Events_h__

typedef unsigned long long int StateRegisterType;
const StateRegisterType EVENT_UP = 1;
const StateRegisterType EVENT_DOWN = 2;
const StateRegisterType EVENT_RIGHT = 4;
const StateRegisterType EVENT_LEFT = 8;
const StateRegisterType EVENT_END = 16;

#endif // Events_h__