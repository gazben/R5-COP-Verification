#ifndef StateRegister_h__
#define StateRegister_h__

#include <stdlib.h>
#include "Property.h"

unsigned long long int stateRegister;

/* TEST FUNCTIONS */
void addEvent(unsigned long long int event_code){
  stateRegister = stateRegister | event_code;
}

void removeEvent(unsigned long long int event_code){
  stateRegister = stateRegister ^ event_code;
}

OutputState getEvent(unsigned long long int StateRegisterCopy, unsigned long long int event_code){
  return (stateRegisterCopy & event_code?TRUE:FALSE);
}

typedef struct StateRegisterState{
  unsigned long long int stateRegisterState;
  struct StateRegisterState* rightNode;
  struct StateRegisterState* leftNode;
}StateRegisterState;

StateRegisterState* SR_initStateRegisterState(unsigned long long int StateRegisterCopy){
  StateRegisterState* temp = (StateRegisterState*)malloc(sizeof(StateRegisterState));
  temp->leftNode = NULL;
  temp->rightNode = NULL;
  temp->stateRegisterState = StateRegisterCopy;

  return temp;
}

StateRegisterState* SR_insertState(StateRegisterState* root, unsigned long long int StateRegisterCopy){
  if (root == NULL) {
    StateRegisterState *uj = (StateRegisterState*)malloc(sizeof(StateRegisterState));
    uj->leftNode = uj->rightNode = NULL;
    uj->stateRegisterState = StateRegisterCopy;
    return uj;
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
StateRegisterState* SR_getStatePointer(StateRegisterState* root, unsigned long long int StateRegisterCopy){
  StateRegisterState* temp = root;

  while (temp != NULL && temp->stateRegisterState != StateRegisterCopy) {
    if (StateRegisterCopy < temp->stateRegisterState){
      if (temp->leftNode == NULL){
        temp->leftNode = SR_initStateRegisterState(StateRegisterCopy);
        return temp->leftNode;
      }
      temp = temp->leftNode;
    }
    else{
      if (temp->rightNode == NULL){
        temp->rightNode = SR_initStateRegisterState(StateRegisterCopy);
        return temp->rightNode;
      }
      temp = temp->rightNode;
    }
  }

  return temp;
}

#endif // StateRegister_h__
