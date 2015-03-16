#include <stdio.h>

#include "EventHandler.h"
#include "Events.h"

int main(){
  stateRegister = 0;

  addEvent(EVENT_A);
  printf("%d\n", stateRegister);  //1
  addEvent(EVENT_B);
  printf("%d\n", stateRegister);  //3
  removeEvent(EVENT_C);
  printf("%d\n", stateRegister);  //2

  /*TEST1*/
  /*
  Property* root = NULL;
  root = PROP_addNewPropertyToRoot(stateRegister, root, EVAL_Impl_U);

  PROP_freePropertyTree(root);
  */
  getchar();
  return 0;
}