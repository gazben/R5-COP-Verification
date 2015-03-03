#ifndef Property_h__
#define Property_h__

#include <limits.h>
#include <stdlib.h>

#include "StateRegister.h"

#define EVENT_A 0x1
#define EVENT_B 0x2
#define EVENT_C 0x4
#define EVENT_D 0x8

typedef enum OutputState{
  TRUE, FALSE, UNKNOWN
}OutputState;

/* 3 state logic functions */
OutputState AND_3(OutputState a, OutputState b){
  return (a == FALSE | b == FALSE) ? FALSE : (a == UNKNOWN | b == UNKNOWN) ? UNKNOWN : TRUE;
}
OutputState NAND_3(OutputState a, OutputState b){
  return (a == FALSE | b == FALSE) ? TRUE : (a == UNKNOWN | b == UNKNOWN) ? UNKNOWN : FALSE;
}
OutputState OR_3(OutputState a, OutputState b){
  return (a == TRUE | b == TRUE) ? TRUE : (a == UNKNOWN | b == UNKNOWN) ? UNKNOWN : FALSE;
}
OutputState XOR_3(OutputState a, OutputState b){
  return (a == UNKNOWN | b == UNKNOWN) ? UNKNOWN : (a == b) ? FALSE : TRUE;
}
OutputState NOT_3(OutputState a){
  return (a == UNKNOWN) ? UNKNOWN : (a == FALSE) ? TRUE : FALSE;
}
int isUnknown(OutputState a){
  return (a == UNKNOWN) ? 1 : 0;
}

typedef OutputState(*evalFunctionType)(int stateRegisterCopy);

/* TEST EVAL FUNCTIONS */
OutputState evalImpl_U(int stateRegisterCopy){
  return UNKNOWN;
}
OutputState evalImpl_T(int stateRegisterCopy){
  return TRUE;
}
OutputState evalImpl_F(int stateRegisterCopy){
  return FALSE;
}

OutputState s0(int StateRegisterCopy, OutputState x1=UNKNOWN, OutputState x2=UNKNOWN){
  return AND_3(NAND_3(getEvent(StateRegisterCopy,EVENT_A),
    AND_3(NOT_3(getEvent(StateRegisterCopy,EVENT_B)),
    NAND_3(getEvent(StateRegisterCopy,EVENT_C),x1))),NAND_3(TRUE,x2));
}

OutputState s1a(int StateRegisterCopy, OutputState x1=UNKNOWN, OutputState x2=UNKNOWN){
  return NAND_3(NOT_3(getEvent(StateRegisterCopy,EVENT_B)),
    NAND_3(getEvent(StateRegisterCopy,EVENT_C),x1));
}

typedef struct Property{
  struct Property* rootNode;
  struct Property* leftNode;
  struct Property* rightNode;

  int stateRegisterCopy;
  evalFunctionType evalFunction;
}Property;

Property* PROP_createEmptyProperty(){
  Property* newProperty = (Property*)malloc(sizeof(Property));
  newProperty->leftNode = NULL;
  newProperty->rightNode = NULL;
  newProperty->rootNode = NULL;
  newProperty->evalFunction = NULL;

  return newProperty;
}

Property* PROP_addNewPropertyToRoot(int stateRegisterCopy, Property* rootproperty, evalFunctionType evalFunction){
  Property* tempPropertyPtr = PROP_createEmptyProperty();
  tempPropertyPtr->rootNode = rootproperty;

  tempPropertyPtr->stateRegisterCopy = stateRegisterCopy;
  tempPropertyPtr->evalFunction = evalFunction;

  if (rootproperty == NULL){
    rootproperty = tempPropertyPtr;
    tempPropertyPtr->rootNode = NULL;
  }
  else if (rootproperty->leftNode == NULL){
    rootproperty->leftNode = tempPropertyPtr;
  }
  else{
    rootproperty->rightNode = tempPropertyPtr;
  }

  return rootproperty;
}

void PROP_freePropertyTree(Property* root) {
  if (root == NULL)
    return;

  PROP_freePropertyTree(root->rightNode);
  PROP_freePropertyTree(root->leftNode);
  free(root);
}
#endif // Property_h__
