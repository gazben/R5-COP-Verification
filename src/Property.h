/*
The property is made of contraints. 
*/

/*
TODO:
   - build up a binary tree
   - 

*/
#include <limits.h>

#define EVENT_A 0x1
#define EVENT_B 0x2
#define EVENT_C 0x4
#define EVENT_D 0x8


 int stateRegister;

typedef enum OutputState{
  TRUE, FALSE, UNKNOWN
};

typedef struct Property{

  struct Property* rootNode;
  struct Property* leftNode;
  struct Property* rightNode;

  int stateRegisterCopy;

};

/* TEST FUNCTIONS */
void addEvent( int event_code ){
  stateRegister = stateRegister | event_code;
}

void removeEvent( int event_code){
  stateRegister = stateRegister & (INT_MAX ^ event_code);
}


