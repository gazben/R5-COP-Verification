#include "Function.h"

unsigned int Function::ID_counter = 0;

Function::Function()
{
  ID = ID_counter++;
}