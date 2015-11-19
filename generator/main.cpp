#include "generator.h"

int main(int argc, char* argv[]) {
  Generator gen(argc, argv);
  gen.run();
  return gen.getErrorCode();
}