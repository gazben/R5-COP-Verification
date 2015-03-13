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
  newProperty->evalFunction = NULL;
  newProperty->constructDescendantNode = NULL;
  newProperty->stateRegisterPtr = NULL;

  return newProperty;
}

Property* PROP_addNewPropertyToRoot(unsigned long long int stateRegisterCopy, Property* rootproperty, PROP_evalFunctionType evalFunction){
  Property* tempPropertyPtr = PROP_createEmptyProperty();
  tempPropertyPtr->rootNode = rootproperty;

  tempPropertyPtr->stateRegisterPtr = SR_getStatePointer(SR_getStateRegister());
  tempPropertyPtr->evalFunction = evalFunction;

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
  Property* temp;

  for (temp = root;; temp = temp->descendantNode){
    if (root->evalFunction(root->stateRegisterPtr->stateRegisterState) != UNKNOWN)
      break;
    //THIS IS NOT WORKING
  }
}

void PROP_freePropertyTree(Property* root) {
  if (root == NULL)
    return;

  PROP_freePropertyTree(root->descendantNode);
  free(root);
}

#endif // PropertyHandler_h__
