#ifndef Property_h__
#define Property_h__

/* GLOBAL INCLUDES */
#include <stdlib.h>

/* LOCAL INCLUDES */

/* INCLUDES END */

/* FUNCTION TYPE DEFINITIONS */
typedef struct Property;
typedef OutputState(*PROP_evalFunctionType)(struct Property*);
typedef struct Property*(*PROP_constructDescendantNodeType)(struct Property*);

typedef struct Property{
  struct Property* rootNode;
  struct Property* descendantNode;

  StateRegisterState* stateRegisterPtr;

  unsigned int inputSize;
  OutputState* inputStates;

  PROP_evalFunctionType* evalFunctions;

  unsigned int outputSize;
  OutputState* outputStates;

  PROP_constructDescendantNodeType constructDescendantNode;
}Property;

#endif // Property_h__
