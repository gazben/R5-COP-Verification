#include <stdio.h>

#include "Property.h"

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
  root = addNewPropertyToRoot(stateRegister, root, evalImpl_U);
  addNewPropertyToRoot(stateRegister, root, evalImpl_T);
  addNewPropertyToRoot(stateRegister, root, evalImpl_F);

  freePropertyTree(root);

  getchar();
  return 0;
}