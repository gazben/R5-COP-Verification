#ifndef PropertyHandler_h__
#define PropertyHandler_h__

/* GLOBAL INCLUDES */
#include <stdio.h>

/* LOCAL INCLUDES */
#include "EventHandler.h"
#include "Property.h"

/* INCLUDES END */

OutputState getEvent(Property* _this, SR_regtype event){
  return (_this->stateRegisterPtr->stateRegisterState & event) ? FALSE : TRUE;
}

/* TEST EVAL FUNCTIONS */
OutputState EVAL_s0( Property* _this){
  return 
    AND_3( 
      NAND_3( 
        getEvent(_this, EVENT_R),
        AND_3(  
          NOT_3(getEvent(_this, EVENT_D)),
          NAND_3(
            getEvent(_this, EVENT_P),
            _this->inputStates[0])
        )
      ), 
      NAND_3(TRUE, _this->inputStates[1])
    );
}

OutputState EVAL_s1a(Property* _this, SR_regtype event){
  return 
    NAND_3(
      NOT_3(getEvent(_this, EVENT_D)),
      NAND_3(getEvent(_this, EVENT_P), _this->inputStates[1])
    );
}
Property* PROP_createEmptyProperty(){
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
Property* PROP_constructS0(Property* _this){
  _this->descendantNode = PROP_createEmptyProperty();
  _this->descendantNode = PROP_initProperty(_this, 2, 1);
  _this->descendantNode->rootNode = _this;

  _this->descendantNode->evalFunctions[0] = EVAL_s0;

  return _this;
}

Property* PROP_constructS1(Property* _this){
  _this->descendantNode = PROP_createEmptyProperty();
  _this->descendantNode = PROP_initProperty(_this, 2, 2);
  _this->descendantNode->rootNode = _this;

  _this->descendantNode->evalFunctions[0] = EVAL_s1a;
  _this->descendantNode->evalFunctions[1] = EVAL_s0;

  return _this;
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

Property* PROP_addNewPropertyToRoot(SR_regtype stateRegisterCopy, Property* rootproperty, PROP_evalFunctionType evalFunction){
  Property* tempPropertyPtr = PROP_createEmptyProperty();
  tempPropertyPtr->rootNode = rootproperty;

  tempPropertyPtr->stateRegisterPtr = SR_getStatePointer(SR_getStateRegister());

  if (rootproperty == NULL){
    rootproperty = tempPropertyPtr;
    tempPropertyPtr->rootNode = NULL;
  }
  else if (rootproperty->descendantNode == NULL){
    rootproperty->descendantNode = tempPropertyPtr;
  }

  return rootproperty;
}

OutputState PROP_evaluateProperty(Property* root){
  Property* currentNode = root;
  OutputState result = UNKNOWN;

  short isChanged = 0;

  //while (result == UNKNOWN){
    isChanged = 0;

    for (int i = 0; i < currentNode->outputSize; i++){
      OutputState tempOutputResult = currentNode->evalFunctions[i](currentNode->stateRegisterPtr->stateRegisterState);
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
        if (currentNode->inputSize != currentNode->outputSize){
          printf("Not eqvivalent input/output count! Input: %d Output: %d", currentNode->inputSize, currentNode->outputSize);
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
  //}

  return result;
}
#endif // PropertyHandler_h__
