#ifndef StateRegister_h__
#define StateRegister_h__

/* GLOBAL INCLUDES */
#include <stdlib.h>

/* LOCAL INCLUDES */
#include "Events.h"
/* INCLUDES END */

class StateRegisterState{
private:
  //Global stateRegister
  static SR_regtype stateRegister;
  SR_regtype stateRegisterState;

  StateRegisterState* rightNode;
  StateRegisterState* leftNode;

  StateRegisterState& insertState(StateRegisterState& ){
    if (root == NULL) {
      StateRegisterState *temp = (StateRegisterState*)malloc(sizeof(StateRegisterState));
      temp->leftNode = temp->rightNode = NULL;
      temp->stateRegisterState = StateRegisterCopy;
      return temp;
    }

    if (StateRegisterCopy < root->stateRegisterState)
      root->leftNode = SR_insertState(root->leftNode, StateRegisterCopy);
    else if (StateRegisterCopy > root->stateRegisterState)
      root->rightNode = SR_insertState(root->rightNode, StateRegisterCopy);
    else
      ;

  }
public:

  StateRegisterState(){
    leftNode = nullptr;
    rightNode = nullptr;
    stateRegisterState = stateRegister;
  }


  friend class Eventhandler;
};


StateRegisterState* SR_insertState(StateRegisterState* root, SR_regtype StateRegisterCopy){
  if (root == NULL) {
    StateRegisterState *temp = (StateRegisterState*)malloc(sizeof(StateRegisterState));
    temp->leftNode = temp->rightNode = NULL;
    temp->stateRegisterState = StateRegisterCopy;
    return temp;
  }

  if (StateRegisterCopy < root->stateRegisterState)
    root->leftNode = SR_insertState(root->leftNode, StateRegisterCopy);
  else if (StateRegisterCopy > root->stateRegisterState)
    root->rightNode = SR_insertState(root->rightNode, StateRegisterCopy);
  else
    ;

  return root;
}

/*If the state is not found it will be inserted to the tree. The new node pointer will be returned.*/
StateRegisterState* SR_getStatePointer(SR_regtype StateRegisterCopy){
  if (SR_rootPtr == nullptr){
    SR_rootPtr = SR_initStateRegisterState(SR_getStateRegister());
    return SR_rootPtr;
  }

  StateRegisterState* temp = SR_rootPtr;

  while (temp != nullptr && temp->stateRegisterState != SR_getStateRegister()) {
    if (StateRegisterCopy < temp->stateRegisterState){
      if (temp->leftNode == nullptr){
        temp->leftNode = SR_initStateRegisterState(SR_getStateRegister());
        return temp->leftNode;
      }
      temp = temp->leftNode;
    }
    else{
      if (temp->rightNode == nullptr){
        temp->rightNode = SR_initStateRegisterState(SR_getStateRegister());
        return temp->rightNode;
      }
      temp = temp->rightNode;
    }
  }

  return temp;
}

#endif // StateRegister_h__
