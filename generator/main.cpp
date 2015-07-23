#include <iostream>
#include <fstream>
#include <cmath>

#include <SyntX/util/parser/parser.h>

#include "Generator.h"

using namespace util::parser;
using namespace std;

int main(int argc, char* argv[]) {
  base_rule::set_build_ast(true);

  //Lexer rules
  rule ReleaseOp;
  ReleaseOp <<= character("R") | character("W");
  rule UntilOp;
  UntilOp <<= character("U");
  rule ImplicationOp;
  ImplicationOp <<= identifier("->") | identifier("=>") | identifier("-->") | identifier("imply") | identifier("i");
  rule EquivalenceOp;
  EquivalenceOp <<= identifier("<->") | identifier("<=>") | identifier("iff") | identifier("eq") | identifier("e");
  rule XorOp;
  XorOp <<= character("^") | identifier("xor");
  rule OrOp;
  OrOp <<= character("|") | identifier("||") | character("+") | identifier("or");
  rule AndOp;
  AndOp <<= character("&") | identifier("&&") | character("*") | identifier("and");
  rule WeakNextOp;
  WeakNextOp <<= identifier("wX") | identifier("wO");
  rule NextOp;
  NextOp <<= character("X") | character("O");
  rule FutureOp;
  FutureOp <<= character("F") | identifier("<>");
  rule GloballyOp;
  GloballyOp <<= character("G") | identifier("[]");
  rule NotOp;
  NotOp <<= character("!") | character("~") | identifier("not");

  rule Operator;
  Operator <<=
          ReleaseOp
          | UntilOp
          | ImplicationOp
          | EquivalenceOp
          | XorOp
          | OrOp
          | AndOp
          | WeakNextOp
          | NextOp
          | FutureOp
          | GloballyOp
          | NotOp
          ;


  rule LPAR;
  LPAR <<= character("(");
  rule RPAR;
  RPAR <<= character(")");
  rule AtomicProposition;
  AtomicProposition <<= range('0', '9');

  rule addition, OperatorEnd("OperatorEnd"), expression;
  addition <<= -OperatorEnd << *(-Operator << -OperatorEnd);
  OperatorEnd <<= -AtomicProposition | -expression;
  expression <<= -LPAR << -addition << -RPAR;

  //LANGUAGE END
  std::string input = "1 G ( 1 & 2 )\n";
  base_rule::match_range context(input.cbegin(), input.cend());
  base_rule::match_range result;
  std::shared_ptr<base_rule::node> root;

  if (addition.match(context, result, root)) {
    std::cout << "Matched: " << std::string(result.first, result.second) << std::endl;

    ast_draw printer;
    printer.to_formatted_string(root);
    printer.draw_to_file(root);

    printer.opimize_ast(root);
    printer.to_formatted_string(root);

  }
  else {
    std::cout << "Didn't match" << std::endl;
  }

  /*
  if (argc <= 1) {
    cout << "ROS Automatic monitor generator ALFA. Made by Gazder Bence" << endl;
    cout << "Please enter the expression!" << endl;
    getline(std::cin, input);
  }
  else
  {
    input = std::string(argv[1]);
  }
  Generator gen;

  if (gen.Parse(input)) {
    gen.Generate();
    cout << "Generation completed." << endl;
    cout << "Given expression: " + input << endl;
  }

  cout << endl << "Press enter to quit." << endl;
  */
  getchar();
  return 0;
}