#include <stdio.h>

#include "Property.h"


int main(){

  addEvent(EVENT_A);
  printf( "%d\n", stateRegister );  //1
  addEvent(EVENT_B);
  printf("%d\n", stateRegister);  //3
  removeEvent(EVENT_A);
  printf("%d\n", stateRegister);  //2


  getchar();
  return 0;
}