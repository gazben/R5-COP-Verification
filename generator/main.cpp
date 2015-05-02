#include "Generator.h"

using namespace std;

int main(int argc, char* argv[]){
  string expression;

  if (argc <= 1){
    cout << "ROS Automatic monitor generator ALFA. Made by Gazben" << endl;
    cout << "Please enter the expression!" << endl;
    getline(std::cin, expression);
  }
  else
  {
    expression = string(argv[0]);
  }
  Generator gen;

  gen.Parse();
  gen.Generate();

  cout << "Generation completed." << endl;
  cout << "Given expression: " + expression << endl;
  cout << "Press enter to quit." << endl;
  getchar();
  return 0;
}