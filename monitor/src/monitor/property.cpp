#include "property.h"

//True command
std::string true_command = "--true_command--";
//False command
std::string false_command = "--false_command--";

//Static field init
unsigned int Property::currentMaxID = 0;
unsigned int Property::level = 0;
Property* Property::currentBlock = nullptr;
bool Property::evaluated = false;

trilean Property::isEventFired(StateRegisterType eventCode)
{
  return (stateRegisterPtr->stateRegisterValue & eventCode) ? TRUE : FALSE;
}

trilean Property::Evaluate()
{
  //Initial block
  if (currentBlock == nullptr)
    currentBlock = this;
  
  //Get the current state register for uninitialized block
  if (currentBlock->stateRegisterPtr == nullptr) {
    currentBlock->stateRegisterPtr = StateRegister::getStatePointer();
  }

  //STOP signal handling
  if (currentBlock->isEventFired(EVENT_END) == TRUE) {
    ROS_INFO_STREAM("--END signal found. All input values are FALSE--");
    currentBlock = currentBlock->rootNode;
    currentBlock->freeChildrenNode();

    for (auto& entry : currentBlock->inputStates) {
      entry = trilean(OutputState::FALSE);
    }
    level--;
  }

  //Begin normal evaluation
  ROS_INFO_STREAM("--Block evaluation--");
  ROS_INFO_STREAM("-Current block state-");
  printBlock(currentBlock);

  trilean result = UNKNOWN;
  bool isChanged = false;
  ROS_INFO_STREAM("-Evaluating-");
  for (int i = 0; i < currentBlock->outputStates.size(); i++)
  {
    trilean tempOutputResult = currentBlock->evalFunctions[i](currentBlock);
    if (tempOutputResult != currentBlock->outputStates[i])
    {
      isChanged = true;
      currentBlock->outputStates[i] = tempOutputResult;
    }
  }

  //Output of the descendant node changed. We can go up in the stack.
  if (isChanged) {
    ROS_INFO_STREAM("-Block changed-");

    if (currentBlock->rootNode != nullptr) {
      //Parent node exist. Move up.
      ROS_INFO_STREAM("-Current block state-");
      printBlock(currentBlock);

      currentBlock = currentBlock->rootNode;

      if (currentBlock->inputStates.size() != currentBlock->childrenNode->outputStates.size()) {
        ROS_INFO_STREAM("Invalid input/output size on block: " + std::to_string(currentBlock->ID) + " and " + std::to_string(currentBlock->childrenNode->ID));
        ROS_INFO_STREAM("The system will use the smaller input. This can result in wrong result!");
      }
      for (int i = 0;
      i < ((currentBlock->inputStates.size() < currentBlock->childrenNode->outputStates.size()) ? currentBlock->inputStates.size() : currentBlock->childrenNode->outputStates.size());
        i++) {
        currentBlock->inputStates[i] = currentBlock->childrenNode->outputStates[i];
      }
      currentBlock->freeChildrenNode();

      level--;

      currentBlock->Evaluate();
    }
    else {
      //No parent node -> GOAL REACHED
      ROS_INFO_STREAM("-No parent node. Goal reached!-");
      evaluated = true;
      result = currentBlock->outputStates[0];
      ROS_INFO_STREAM("Result: " + trilean::tostring(result));
      currentBlock->freeChildrenNode();
      ROS_INFO_STREAM("Executing given command: " + ((result == TRUE) ? true_command : false_command));

      if (result == TRUE) {
        system(true_command.c_str());
      }
      if (result == FALSE) {
        system(false_command.c_str());
      }
    }
  }
  else {
    level++;
    //No change happened we go deeper
    ROS_INFO_STREAM("No change. Going deeper!");

    currentBlock->constructChildrenBlock();
    currentBlock = currentBlock->childrenNode;
  }

  return result;
}

void Property::freeChildrenNode()
{
  delete currentBlock->childrenNode;
  currentBlock->childrenNode = nullptr;
}

Property* Property::constructChildrenBlock()
{
  Property* childrenBlockTemp = new Property();
  childrenBlockTemp->rootNode = this;
  childrenNode = constructChildrenNodeFunc(childrenBlockTemp);
  return childrenBlockTemp;
}

Property::~Property()
{
  delete childrenNode;

  if (rootNode != nullptr)
    rootNode->childrenNode = nullptr;
}

Property::Property()
  :childrenNode(nullptr),
  rootNode(nullptr),
  stateRegisterPtr(nullptr),
  constructChildrenNodeFunc(nullptr)
{
  ID = currentMaxID;
  currentMaxID++;
  ROS_INFO_STREAM("BLOCK CREATED | ID " + std::to_string(ID));
}

void Property::printBlock(Property *block) {
  //Print the current block out
  std::string tempOut;
  for (auto& entry : block->outputStates) {
    tempOut += (entry == OutputState::FALSE) ? "F" : (entry == OutputState::TRUE) ? "T" : "U";
    tempOut += " ";
  }
  std::string tempIn;
  ROS_INFO_STREAM("Out: " + tempOut);
  ROS_INFO_STREAM("ID: " + std::to_string(block->ID) + " level: " + std::to_string(level)
    + " Statereg: " + std::to_string(block->stateRegisterPtr->stateRegisterValue));
  for (trilean& entry : block->inputStates) {
    tempIn += (entry == OutputState::FALSE) ? "F" : (entry == OutputState::TRUE) ? "T" : "U";
    tempIn += " ";
  }
  ROS_INFO_STREAM("In: " + tempIn);
}

//Construct
//--CONSTRUCTFUNCTIONS--

//Eval
//--EVALFUNCTIONS--
