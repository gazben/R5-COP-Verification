#include "EventInterfaceHandler.h"

EventInterfaceHandler* EventInterfaceHandler::instance = nullptr;

void EventInterfaceHandler::readEventsFromFile(std::string filename)
{
  eventFile.open(filename);
}

SR_regtype EventInterfaceHandler::getNextEvent()
{
  if (eventFile.is_open())
    eventQueue.push_back(readNextLine());

  SR_regtype front = eventQueue.front();
  StateRegisterState::stateRegister = front;
  eventQueue.pop_front();
  return front;
}

EventInterfaceHandler::~EventInterfaceHandler()
{
  if (eventFile.is_open())
    eventFile.close();
}

SR_regtype EventInterfaceHandler::readNextLine()
{
  std::string line;
  SR_regtype tempStateReg;
  std::getline(eventFile, line);

  for (char c : line){
    switch (c)
    {
    case 'a':
      tempStateReg |= EVENT_A;
      break;
    case 'b':
      tempStateReg |= EVENT_B;
      break;
    case 'c':
      tempStateReg |= EVENT_C;
      break;
    case 'd':
      tempStateReg |= EVENT_D;
      break;
    case '\n':
      break;
    default:
      printf("Invalid state found in testfile!\n");
      break;
    }
    return tempStateReg;
  }
}

EventInterfaceHandler* EventInterfaceHandler::getinstance()
{
  if (instance == nullptr)
    instance = new EventInterfaceHandler();

  return instance;
}
