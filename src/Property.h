#ifndef Property_h__
#define Property_h__

/* GLOBAL INCLUDES */
#include <functional>
#include <vector>

/* LOCAL INCLUDES */
#include "Events.h"
#include "OutputState.h"
/* INCLUDES END */

/* FUNCTION TYPE DEFINITIONS */
class Property;
using propertyEvalFunc = std::function < trilean(class Property*) > ;
using constructChildrenBlock = std::function < class Property*(class Property*) > ;

class Property{
protected:
  Property* rootNode;
  Property* childrenNode;

  StateRegisterState* stateRegisterPtr;

  unsigned int inputSize;
  std::vector<trilean> inputStates;

  unsigned int outputSize;
  std::vector <propertyEvalFunc> evalFunctions;
  constructChildrenBlock constructChildrenNode;

  std::vector<trilean> outputStates;
public:

  Property(unsigned int _inputSize = 0, unsigned int _outputSize = 0)
    :childrenNode(nullptr),
    rootNode(nullptr),
    stateRegisterPtr(nullptr),
    inputSize(_inputSize),
    outputSize(_outputSize),
    constructChildrenNode(nullptr)
  {
    evalFunctions.resize(_outputSize);
    inputStates.resize(_inputSize);

    stateRegisterPtr = SR_getStatePointer(SR_getStateRegister());
  }

  void setEvalFunction(std::vector<propertyEvalFunc> func){
    evalFunctions = func;
  }

  void freeChildrenNode(){
    delete childrenNode;
  }

  //Getter-setters
  Property* RootNode() const { return rootNode; }
  void RootNode(class Property* val) { rootNode = val; }

  Property* ChildrenNode() const { return childrenNode; }
  void ChildrenNode(Property* val) { childrenNode = val; }

  trilean isEventFired(SR_regtype eventCode){
    return (stateRegisterPtr->stateRegisterState & eventCode) ? FALSE : TRUE;
  }
};

//////////////////////////////////////////////////////////////////////////
namespace EvalFunctions{

  trilean EVAL_s0(const Property& _prop){
    return
      AND_3(
      NAND_3(
      _prop.isEventFired(EVENT_A),
      AND_3(
      NOT_3(_prop.isEventFired(EVENT_B)),
      NAND_3(
      _prop.isEventFired(EVENT_C),
      _prop.inputStates[0])
      )
      ),
      NAND_3(TRUE, _prop->inputStates[1])
      );
  }


  trilean EVAL_s1a(const Property& _prop){
    return
      NAND_3(
      NOT_3(_prop.isEventFired(EVENT_B)),
      NAND_3(_prop.isEventFired(EVENT_C), _prop->inputStates[1])
      );
  }

}

//////////////////////////////////////////////////////////////////////////
namespace ConstructFunctions{

  Property* PROP_constructS0(Property* _rootNode){
    Property* newProppertyNode;

    newProppertyNode = PROP_createEmptyPropertyNode();
    newProppertyNode = PROP_initProperty(newProppertyNode, 2, 1);

    if (_rootNode != NULL){
      newProppertyNode->rootNode = _rootNode;
      _rootNode->descendantNode = newProppertyNode;
    }

    newProppertyNode->evalFunctions[0] = EVAL_s0;
    newProppertyNode->constructDescendantNode = PROP_constructS1;

    return newProppertyNode;
  }

  Property* PROP_constructS1(Property* _rootNode){
    Property* newProppertyNode;

    newProppertyNode = PROP_createEmptyPropertyNode();
    newProppertyNode = PROP_initProperty(newProppertyNode, 2, 2);

    if (_rootNode != NULL){
      newProppertyNode->rootNode = _rootNode;
      _rootNode->descendantNode = newProppertyNode;
    }

    newProppertyNode->evalFunctions[0] = EVAL_s1a;
    newProppertyNode->evalFunctions[1] = EVAL_s0;
    newProppertyNode->constructDescendantNode = PROP_constructS1;

    return newProppertyNode;
  }

}

trilean PROP_evaluateProperty(Property* root){
  Property* currentNode = root;
  trilean result = UNKNOWN;

  bool isChanged = 0;

  while (result == UNKNOWN){
    isChanged = false;

    for (int i = 0; i < currentNode->outputSize; i++){
      trilean tempOutputResult = currentNode->evalFunctions[i](currentNode);
      if (tempOutputResult != currentNode->outputStates[i]){
        isChanged = true;
        currentNode->outputStates[i] = tempOutputResult;
        break;
      }
    }

    //Output of the descendant node changed. We can go up in the stack.
    if (isChanged){
      //Free the current node.
      if (currentNode->rootNode != nullptr){
        currentNode = currentNode->rootNode;

        //give the output to the input
        if (currentNode->inputSize != currentNode->childrenNode->outputSize){
          printf("Not eqvivalent input/output count! Input: %d Output: %d\n", currentNode->inputSize, currentNode->outputSize);
        }

        //COPY right now, optimise later!
        for (int i = 0; i < currentNode->inputSize; i++){
          currentNode->inputStates[i] = currentNode->childrenNode->outputStates[i];
        }

        delete currentNode->childrenNode;
      }
      else{
        //GOAL REACHED
        result = currentNode->outputStates[0];
        PROP_freePropertyStack(root);
      }
    }
    //No change happened we go deeper
    else{
      currentNode->constructChildrenNode(currentNode);
      currentNode = currentNode->childrenNode;
    }
  }

  return result;
}

#endif // Property_h__
