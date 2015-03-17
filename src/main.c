#include <stdio.h>

#include "PropertyHandler.h"

int main(){
  stateRegister = 0;

  addEvent(EVENT_R);
  printf("%d\n", stateRegister);  //1
  addEvent(EVENT_D);
  printf("%d\n", stateRegister);  //3
  removeEvent(EVENT_P);
  printf("%d\n", stateRegister);  //2

  /*TEST1*/
  Property* root = NULL;
  //root = PROP_addNewPropertyToRoot(stateRegister, root);


  getchar();
  return 0;
}