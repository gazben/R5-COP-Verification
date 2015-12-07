#include "property.h"

//True command
std::string true_command = "--true_command--";
//False command
std::string false_command = "--false_command--";

//Static field init
unsigned int Property::currentMaxID = 0;
unsigned int Property::level = 0;
const unsigned int Property::maxDepth = 2; //will be generated
Property* Property::currentBlock = nullptr;
bool Property::evaluated = false;

trilean Property::isEventFired(StateRegisterType eventCode)
{
  return (stateRegisterPtr->stateRegister & eventCode) ? FALSE : TRUE;
}

trilean Property::Evaluate()
{
  if (currentBlock == nullptr)
    currentBlock = this;

  if (currentBlock->stateRegisterPtr == nullptr) {
    currentBlock->stateRegisterPtr = StateRegister::getStatePointer();
  }

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
        ROS_INFO_STREAM("Invalid input/output size on block: " + std::to_string(this->ID) + " and " + std::to_string(this->childrenNode->ID));
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
  //if we reach the button, we have to initialize the inputs to false
  if (level == maxDepth) {
    inputStates.resize(2);
    for (auto& entry : inputStates) {
      entry = trilean(OutputState::FALSE);
    }
  }
}

//Construct
//--CONSTRUCTFUNCTIONS--


//Eval
//--EVALFUNCTIONS--

/*
trilean EVAL_s0(Property* _prop)
{
  return
      NAND_3(
          NAND_3(
              _prop->isEventFired(EVENT_UP),
              AND_3(
                  NOT_3(_prop->isEventFired(EVENT_RIGHT)),
                  NAND_3(
                      _prop->isEventFired(EVENT_DOWN),
                      _prop->inputStates[0])
              )
          ),
          NAND_3(TRUE, _prop->inputStates[1])
      );
}


trilean EVAL_s1a(Property* _prop)
{
  return
      NAND_3(
          NOT_3(_prop->isEventFired(EVENT_RIGHT)),
          NAND_3(_prop->isEventFired(EVENT_DOWN), _prop->inputStates[1])
      );
}

Property* construct_START(Property* _rootNode)
{
  _rootNode->evalFunctions.push_back(EVAL_s0);
  _rootNode->constructChildrenNodeFunc = constructS1;
  _rootNode->outputStates.resize(1);
  _rootNode->inputStates.resize(2);
  return _rootNode;
}

Property* constructS1(Property* _rootNode)
{
  _rootNode->evalFunctions.resize(2);
  _rootNode->evalFunctions[0] = EVAL_s1a;
  _rootNode->evalFunctions[1] = EVAL_s0;
  _rootNode->constructChildrenNodeFunc = constructS1;
  _rootNode->outputStates.resize(2);
  _rootNode->inputStates.resize(2);
  return _rootNode;
}
*/

void Property::printBlock(Property *block) {
  //Print the current block out
  std::string tempOut;
  for (auto& entry : block->outputStates) {
    tempOut += (entry == OutputState::FALSE)? "F" : (entry == OutputState::TRUE)? "T" : "U";
    tempOut += " ";
  }
  std::string tempIn;
  ROS_INFO_STREAM("Out: " + tempOut);
  ROS_INFO_STREAM( "ID: " + std::to_string(block->ID) + " level: " + std::to_string(level)
                   + " Statereg: " + std::to_string(block->stateRegisterPtr->stateRegisterValue));
  for (trilean& entry : block->inputStates) {
    tempIn += (entry == OutputState::FALSE)? "F" : (entry == OutputState::TRUE)? "T" : "U";
    tempIn += " ";
  }
  ROS_INFO_STREAM("In: " + tempIn);
}
