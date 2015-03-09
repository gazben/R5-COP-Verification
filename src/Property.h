#ifndef Property_h__
#define Property_h__

#include <limits.h>
#include <stdlib.h>

#include <stdio.h>

#define EVENT_R 0x1
#define EVENT_P 0x2
#define EVENT_D 0x4

typedef enum OutputState{
  TRUE, FALSE, UNKNOWN, DONTCARE
}OutputState;

char* OUTPUTSTATE_toString(OutputState state){
  switch (state)
  {
  TRUE:
    return "TRUE";
  FALSE:
    return "FALSE";
  UNKNOWN:
    return "UNKNOWN";
  DONTCARE:
    return "DONTCARE";
  default:
    return "ERROR Unknown state!";
  }
}

typedef struct EvalResult{
  OutputState leftResult;
  OutputState rightResult;
}EvalResult;

void EVALRESULT_init(EvalResult* _this){
  _this->leftResult = UNKNOWN;
  _this->rightResult = UNKNOWN;
}

/* 3 state logic functions */
OutputState AND_3(OutputState a, OutputState b){
  if (a == DONTCARE || b == DONTCARE){
    printf("ERROR unexpected DONTCARE state!");
    return FALSE;
  }

  return (a == FALSE | b == FALSE) ? FALSE : (a == UNKNOWN | b == UNKNOWN) ? UNKNOWN : TRUE;
}
OutputState NAND_3(OutputState a, OutputState b){
  if (a == DONTCARE || b == DONTCARE){
    printf("ERROR unexpected DONTCARE state!");
    return FALSE;
  }
  return (a == FALSE | b == FALSE) ? TRUE : (a == UNKNOWN | b == UNKNOWN) ? UNKNOWN : FALSE;
}
OutputState OR_3(OutputState a, OutputState b){
  if (a == DONTCARE || b == DONTCARE){
    printf("ERROR unexpected DONTCARE state!");
    return FALSE;
  }
  return (a == TRUE | b == TRUE) ? TRUE : (a == UNKNOWN | b == UNKNOWN) ? UNKNOWN : FALSE;
}
OutputState XOR_3(OutputState a, OutputState b){
  if (a == DONTCARE || b == DONTCARE){
    printf("ERROR unexpected DONTCARE state!");
    return FALSE;
  }
  return (a == UNKNOWN | b == UNKNOWN) ? UNKNOWN : (a == b) ? FALSE : TRUE;
}
OutputState NOT_3(OutputState a){
  if (a == DONTCARE){
    printf("ERROR unexpected DONTCARE state!");
    return FALSE;
  }
  return (a == UNKNOWN) ? UNKNOWN : (a == FALSE) ? TRUE : FALSE;
}
int isUnknown(OutputState a){
  return (a == UNKNOWN) ? 1 : 0;
}

/* FUNCTION TYPE DEFINITIONS */
typedef struct Property;
typedef OutputState(*PROP_evalFunctionType)(unsigned long long int);
typedef void(*PROP_constructDescendantNodeType)(Property*);

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

OutputState EVAL_s0(int StateRegisterCopy, OutputState x1, OutputState x2){
  return AND_3(NAND_3(getEvent(StateRegisterCopy, EVENT_R),
    AND_3(NOT_3(getEvent(StateRegisterCopy, EVENT_P)),
    NAND_3(getEvent(StateRegisterCopy, EVENT_D), x1))), NAND_3(TRUE, x2));
}

OutputState EVAL_s1_left(int StateRegisterCopy, OutputState x1, OutputState x2){
  return NAND_3(NOT_3(getEvent(StateRegisterCopy, EVENT_P)),
    NAND_3(getEvent(StateRegisterCopy, EVENT_D), x1));
}

OutputState EVAL_s1_right(int StateRegisterCopy, OutputState x1, OutputState x2){
}

/* CONSTRUCT FUNCTIONS */
void PROP_constructDescImplicateBlock(Property* _this){
  _this->descendantNode = PROP_createEmptyProperty();
  //_this->descendantNode->evalFunction
}

typedef struct Property{
  struct Property* rootNode;
  struct Property* descendantNode;

  StateRegisterState* stateRegisterPtr;

  PROP_evalFunctionType evalFunctionLeft;
  PROP_evalFunctionType evalFunctionRight;

  EvalResult evalResult;

  PROP_constructDescendantNodeType constructDescendantNode;
}Property;

Property* PROP_createEmptyProperty(){
  Property* newProperty = (Property*)malloc(sizeof(Property));
  newProperty->descendantNode = NULL;
  newProperty->rootNode = NULL;
  newProperty->evalFunctionLeft = NULL;
  newProperty->evalFunctionRight = NULL;
  newProperty->constructDescendantNode = NULL;
  newProperty->stateRegisterPtr = NULL;
  EVALRESULT_init(&newProperty->evalResult);

  return newProperty;
}

void PROP_evaluateNode(Property* _this){
  if (_this->evalResult.leftResult != DONTCARE)
    _this->evalResult.leftResult = _this->evalFunctionLeft;

  if (_this->evalResult.rightResult != DONTCARE)
    _this->evalResult.rightResult = _this->evalFunctionRight;
}

Property* PROP_addNewPropertyToRoot(unsigned long long int _stateRegisterCopy, Property* _rootproperty, PROP_evalFunctionType _evalFunctionLeft, PROP_evalFunctionType _evalFunctionRight){
  Property* tempPropertyPtr = PROP_createEmptyProperty();
  tempPropertyPtr->rootNode = _rootproperty;

  tempPropertyPtr->stateRegisterPtr = _stateRegisterCopy;

  tempPropertyPtr->evalFunctionLeft = _evalFunctionLeft;
  tempPropertyPtr->evalFunctionRight = _evalFunctionRight;

  if (_rootproperty == NULL){
    _rootproperty = tempPropertyPtr;
    tempPropertyPtr->rootNode = NULL;
  }
  else if (_rootproperty->descendantNode == NULL){
    _rootproperty->descendantNode = tempPropertyPtr;
  }

  return _rootproperty;
}

int PROP_evaluateProperty(Property* root){
  Property* currentBlock = root;
  OutputState result = UNKNOWN;

  if (root == NULL){
    return 0;
  }

  while (1){
    /*
          if (currentBlock->descendantNode == NULL){
          printf("ERROR descendantNode is NULL!");
          return UNKNOWN;
          }
          */

    result = AND_3(root->evalResult.leftResult, root->evalResult.rightResult);
    if (result != UNKNOWN){
      break;
    }
  }

  return (result == TRUE) ? TRUE : FALSE;
}

void PROP_freePropertyTree(Property* root) {
  if (root == NULL)
    return;

  PROP_freePropertyTree(root->descendantNode);
  free(root);
}
#endif // Property_h__
