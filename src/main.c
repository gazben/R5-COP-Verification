#include <stdio.h>

#include "PropertyHandler.h"

int main(){
  /* INIT */
  Property* root = NULL;
  root = PROP_constructS0(root);
  EVENT_clearEvents();
  
  PROP_evaluateProperty(root);
  /*
  printf( "%s\n",  OS_tostring(PROP_evaluateProperty(root)));

  EVENT_addEvent(EVENT_A);
  */
  //root->descendantNode = root->constructDescendantNode(root);
  //root = PROP_addNewPropertyToRoot(stateRegister, root);


  getchar();
  return 0;
}