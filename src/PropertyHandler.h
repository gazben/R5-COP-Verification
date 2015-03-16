#ifndef PropertyHandler_h__
#define PropertyHandler_h__

/* GLOBAL INCLUDES */

/* LOCAL INCLUDES */
#include "EventHandler.h"
#include "Property.h"

/* INCLUDES END */

/* TEST EVAL FUNCTIONS */
OutputState EVAL_Impl_U(unsigned long long int event_code){
  return UNKNOWN;
}
OutputState EVAL_Impl_T(unsigned long long int event_code){
  return TRUE;
}
OutputState EVAL_Impl_F(unsigned long long int event_code){
  return FALSE;
}
/*
OutputState EVAL_s0(int StateRegisterCopy, OutputState x1, OutputState x2){
return AND_3(NAND_3(getEvent(StateRegisterCopy, EVENT_A),
AND_3(NOT_3(getEvent(StateRegisterCopy, EVENT_B)),
NAND_3(getEvent(StateRegisterCopy, EVENT_C), x1))), NAND_3(TRUE, x2));
}

OutputState EVAL_s1a(int StateRegisterCopy, OutputState x1, OutputState x2){
return NAND_3(NOT_3(getEvent(StateRegisterCopy, EVENT_B)),
NAND_3(getEvent(StateRegisterCopy, EVENT_C), x1));
}
*/
/* CONSTRUCT FUNCTIONS */
void PROP_constructDescImplicateBlock(Property* _this){
  _this->descendantNode = PROP_createEmptyProperty();
  // _this->descendantNode->evalFunction
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

  _this->outputSize = _outputSize;
  _this->outputStates = (OutputState*)malloc(_outputSize * sizeof(OutputState));

  _this->evalFunctions = (PROP_evalFunctionType*)malloc(_outputSize * sizeof(PROP_evalFunctionType));
}

void PROP_freePropertyNode(Property* _this){
  if (_this->inputSize > 0)
    free(_this->inputStates);

  if (_this->outputSize > 0){
    free(_this->outputStates);
    free(_this->evalFunctions);
  }
}

void PROP_freePropertyTree(Property* root) {
  if (root == NULL)
    return;

  PROP_freePropertyTree(root->descendantNode);
  
  PROP_freePropertyNode(root);
  free(root);
}

Property* PROP_addNewPropertyToRoot(unsigned long long int stateRegisterCopy, Property* rootproperty, PROP_evalFunctionType evalFunction){
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

  while ( result == UNKNOWN ){

    for (int i = 0; i < currentNode->outputSize; i++){
      OutputState temp = currentNode->evalFunctions[i](currentNode->stateRegisterPtr->stateRegisterState);
      if (currentNode->outputStates[i] != temp){
        //TODO
        //state changed (we caqn go UP)
      }

    }
  }

}
#endif // PropertyHandler_h__
