#ifndef PropertyHandler_h__
#define PropertyHandler_h__

/* GLOBAL INCLUDES */
#include <stdio.h>

/* LOCAL INCLUDES */
#include "EventHandler.h"
#include "Property.h"

/* INCLUDES END */

Property* PROP_constructS0(Property* _rootNode);
Property* PROP_constructS1(Property* _rootNode);


OutputState EVENT_getEvent(Property* _this, SR_regtype event){
  return (_this->stateRegisterPtr->stateRegisterState & event) ? FALSE : TRUE;
}

/* TEST EVAL FUNCTIONS */
OutputState EVAL_s0( Property* _this){
  return 
    AND_3( 
      NAND_3( 
        EVENT_getEvent(_this, EVENT_A),
        AND_3(  
          NOT_3(EVENT_getEvent(_this, EVENT_B)),
          NAND_3(
            EVENT_getEvent(_this, EVENT_C),
            _this->inputStates[0])
        )
      ), 
      NAND_3(TRUE, _this->inputStates[1])
    );
}

OutputState EVAL_s1a(Property* _this){
  return 
    NAND_3(
      NOT_3(EVENT_getEvent(_this, EVENT_B)),
      NAND_3(EVENT_getEvent(_this, EVENT_C), _this->inputStates[1])
    );
}
Property* PROP_createEmptyPropertyNode(){
  Property* newProperty = (Property*)malloc(sizeof(Property));

  newProperty->descendantNode = NULL;
  newProperty->rootNode = NULL;

  newProperty->stateRegisterPtr = NULL;

  newProperty->inputSize = 0;
  newProperty->outputSize = 0;

  newProperty->constructDescendantNode = NULL;

  return newProperty;
}

Property* PROP_initProperty(Property* _this, unsigned int _inputSize, unsigned int _outputSize){
  _this->inputSize = _inputSize;
  _this->inputStates = (OutputState*)malloc(_inputSize * sizeof(OutputState));
  for (int i = 0; i < _this->inputSize; i++){
    _this->inputStates[i] = UNKNOWN;
  }

  _this->outputSize = _outputSize;
  _this->outputStates = (OutputState*)malloc(_outputSize * sizeof(OutputState));
  for (int i = 0; i < _this->outputSize; i++){
    _this->outputStates[i] = UNKNOWN;
  }

  _this->evalFunctions = (PROP_evalFunctionType*)malloc(_outputSize * sizeof(PROP_evalFunctionType));

  _this->stateRegisterPtr = SR_getStatePointer(SR_getStateRegister());
  return _this;
}

/* CONSTRUCT FUNCTIONS */
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

//Frees the dinamically allocated memora of a node. This will set the parent node
void PROP_freePropertyNode(Property* _this){
  if (_this->inputSize > 0){
    free(_this->inputStates);
  }

  if (_this->outputSize > 0){
    free(_this->outputStates);
    free(_this->evalFunctions);
  }

  if (_this->rootNode){
    _this->rootNode->descendantNode = NULL;
    free(_this);
  }
}

void PROP_freePropertyStack(Property* root) {
  if (root == NULL)
    return;

  PROP_freePropertyStack(root->descendantNode);

  PROP_freePropertyNode(root);
}

OutputState PROP_evaluateProperty(Property* root){
  
  //TEST
  EVENT_initStateReader("test1.txt");

  Property* currentNode = root;
  OutputState result = UNKNOWN;

  short isChanged = 0;

  while (result == UNKNOWN){
    isChanged = 0;

    for (int i = 0; i < currentNode->outputSize; i++){
      OutputState tempOutputResult = currentNode->evalFunctions[i](currentNode);
      if (tempOutputResult != currentNode->outputStates[i]){
        isChanged = 1;
        currentNode->outputStates[i] = tempOutputResult;
        break;
      }
    }

    //Output of the descendant node changed. We can go up in the stack.
    if (isChanged){
      //Free the current node.
      if (currentNode->rootNode != NULL){
        currentNode = currentNode->rootNode;


        //give the output to the input
        if (currentNode->inputSize != currentNode->descendantNode->outputSize){
          printf("Not eqvivalent input/output count! Input: %d Output: %d\n", currentNode->inputSize, currentNode->outputSize);
        }

        //COPY right now, optimise later!
        for (int i = 0; i < currentNode->inputSize; i++){
          currentNode->inputStates[i] = currentNode->descendantNode->outputStates[i];
        }


        PROP_freePropertyNode(currentNode->descendantNode);
      }
      else{
        //GOAL REACHED
        result = currentNode->outputStates[0];
        PROP_freePropertyStack(root);
      }
    }
    //No change happened we go deeper
    else{
      currentNode->constructDescendantNode(currentNode);
      currentNode = currentNode->descendantNode;
    }

    EVENT_readNextState();
  }

  EVENT_deinitStatereader();
  
  return result;
}
#endif // PropertyHandler_h__
