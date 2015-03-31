/* GLOBAL INCLUDES */
#include <stdio.h>

/* LOCAL INCLUDES */
#include "PropertyHandler.h"
/* INCLUDES END */

int main(){
  
  Property* root = NULL;
  OutputState result;
  root = PROP_constructS0(root);
  EVENT_clearEvents();
  

  result = PROP_evaluateProperty(root);
  printf("The result of the rule checking: %s\n", OS_tostring(result));
  
  getchar();
  return 0;
}