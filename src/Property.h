#ifndef Property_h__
#define Property_h__

/* GLOBAL INCLUDES */
#include <functional>
#include <vector>

/* LOCAL INCLUDES */
#include "EventInterfaceHandler.h"
/* INCLUDES END */

/* FUNCTION TYPE DEFINITIONS */
using namespace std;

class Property{
protected:

  StateRegisterState* stateRegisterPtr;

public:
  vector<trilean> inputStates;
  vector<trilean> outputStates;

  Property* rootNode;
  Property* childrenNode;
  std::function < class Property*(class Property*) > constructChildrenNodeFunc;

  std::vector <std::function < trilean(class Property*) >> evalFunctions;

  Property();

  ~Property();

  Property* constructChildrenBlock();

  std::vector<std::function < trilean(class Property*) >> EvalFunctions() const;

  void EvalFunctions(std::vector<std::function < trilean(class Property*) >> func);

  void freeChildrenNode();

  std::vector<trilean> InputStates() const;

  //Getter-setters
  Property* RootNode() const;
  void RootNode(class Property* val);

  Property* ChildrenNode() const;
  void ChildrenNode(Property* val);

  trilean isEventFired(SR_regtype eventCode);

  static trilean Evaluate(Property* root);
};

//////////////////////////////////////////////////////////////////////////
//EVAL FUNCTION
trilean EVAL_s0(Property* _prop);
trilean EVAL_s1a(Property* _prop);

//////////////////////////////////////////////////////////////////////////
//BLOCK CONSTRUCTION FUNCTIONS
Property* constructS0(Property* _rootNode);
Property* constructS1(Property* _rootNode);
#endif // Property_h__
