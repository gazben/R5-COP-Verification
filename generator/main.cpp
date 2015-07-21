#include <iostream>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>

#include <SyntX/util/parser/parser.h>

#include "Generator.h"

using namespace util::parser;
using namespace std;

int main(int argc, char* argv[]) {

  base_rule::set_build_ast(true);

  rule addition, addend("addend"), expression;

  addition <<= -addend << *(-character("+") << -addend);
  addend <<= -range('0', '9') | -expression;
  expression <<= -character("(") << -addition << -character(")");

  std::string input = "2 + 3 + 4\n";
  base_rule::match_range context(input.cbegin(), input.cend());
  base_rule::match_range result;
  std::shared_ptr<base_rule::node> root;

  if (addition.match(context, result, root)) {
    std::cout << "Matched: " << std::string(result.first, result.second) << std::endl;
   
    ast_draw printer;
    printer.to_formatted_string(root);
    printer.draw_to_file(root);
  }
  else {
    std::cout << "Didn't match" << std::endl;
  }


  std::string given_expression;

  if (argc <= 1) {
    cout << "ROS Automatic monitor generator ALFA. Made by Gazder Bence" << endl;
    cout << "Please enter the expression!" << endl;
    getline(std::cin, given_expression);
  }
  else
  {
    expression = std::string(argv[1]);
  }
  Generator gen;

  if (gen.Parse(given_expression)) {
    gen.Generate();
    cout << "Generation completed." << endl;
    cout << "Given expression: " + given_expression << endl;
  }

  cout << endl << "Press enter to quit." << endl;
  getchar();
  return 0;
}