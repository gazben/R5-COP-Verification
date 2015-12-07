#ifndef Events_h__
#define Events_h__

typedef unsigned long long int StateRegisterType;
const StateRegisterType EVENT_UP = 0x1;
const StateRegisterType EVENT_DOWN = 0x2;
const StateRegisterType EVENT_RIGHT = 0x4;
const StateRegisterType EVENT_LEFT = 0x8;

#endif // Events_h__