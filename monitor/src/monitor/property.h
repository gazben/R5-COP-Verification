#ifndef Property_h__
#define Property_h__

#ifdef DEBUG_NO_ROS
#include <iostream>
#define ROS_INFO_STREAM(args) std::cout<<(args)<<std::endl
#else
#include <ros/ros.h>
#endif


/* GLOBAL INCLUDES */
#include <functional>
#include <vector>
#include <iostream>

/* LOCAL INCLUDES */
#include "output_state.h"
#include "state_register.h"
/* INCLUDES END */


/* FUNCTION TYPE DEFINITIONS */
using namespace std;

class Property {
protected:
  static Property*currentBlock;
  StateRegister * stateRegisterPtr;
  static unsigned int currentMaxID;
  static unsigned int level;
  unsigned int ID;
  static const unsigned int maxDepth;
  static bool evaluated;
public:
  Property();
  ~Property();

  vector<trilean> inputStates;
  vector<trilean> outputStates;
  Property* rootNode;
  Property* childrenNode;

  std::function < class Property*(class Property*) > constructChildrenNodeFunc;
  std::vector <std::function < trilean(class Property*) >> evalFunctions;
  Property* constructChildrenBlock();
  trilean isEventFired(StateRegisterType eventCode);
  trilean Evaluate();
  void freeChildrenNode();

  void printBlock(Property*);
};

//Declaration
trilean EVAL_AndNotAnd1AndNot3NotAnd2NotAndTrue(Property* _prop);

trilean EVAL_NotAndNot3NotAnd2(Property* _prop);
trilean EVAL_NotAndNotAnd1AndNot3NotAnd2NotAndTrue(Property* _prop);

Property* construct_block0(Property* _rootNode);
Property* construct_block1(Property* _rootNode);


#endif // Property_h__
