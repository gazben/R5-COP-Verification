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
    expression = string(argv[1]);
  }
  Generator gen;

  if (gen.Parse(expression)){
    gen.Generate();
    cout << "Generation completed." << endl;
    cout << "Given expression: " + expression << endl;
  }

  cout << endl << "Press enter to quit." << endl;
  getchar();
  return 0;
}