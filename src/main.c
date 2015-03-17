#include <stdio.h>

#include "PropertyHandler.h"

int main(){
  /* INIT */
  Property* root = NULL;
  root = PROP_constructS0(root);
  EVENT_clearEvents();
  

  EVENT_addEvent(EVENT_R);
  EVENT_addEvent(EVENT_P);
  printf( "%s\n",  OS_tostring(PROP_evaluateProperty(root)));

  EVENT_addEvent(EVENT_R);
  root->descendantNode = root->constructDescendantNode(root);
  //root = PROP_addNewPropertyToRoot(stateRegister, root);


  getchar();
  return 0;
}