#include <iostream>
#include "Generator.h"


int main(){
  
  Generator gen;

  gen.Parse();
  gen.Generate();
  
  getchar();
  return 0;
}