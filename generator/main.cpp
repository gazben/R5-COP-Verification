#include "generator.h"

int main(int argc, char* argv[]) {
  Generator gen;
  gen.run(argc, argv);
  return gen.getErrorCode();
}