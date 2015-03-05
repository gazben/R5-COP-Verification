#include <stdio.h>

#include "StateRegister.h"

int main(){
  stateRegister = 0;

  addEvent(EVENT_A);
  printf("%d\n", stateRegister);  //1
  addEvent(EVENT_B);
  printf("%d\n", stateRegister);  //3
  removeEvent(EVENT_A);
  printf("%d\n", stateRegister);  //2

  /*TEST1*/
  Property* root = NULL;
  root = PROP_addNewPropertyToRoot(stateRegister, root, evalImpl_U);
  PROP_addNewPropertyToRoot(stateRegister, root, evalImpl_T);
  PROP_addNewPropertyToRoot(stateRegister, root, evalImpl_F);

  PROP_freePropertyTree(root);

  getchar();
  return 0;
}