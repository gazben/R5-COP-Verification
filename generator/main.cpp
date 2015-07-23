#include <iostream>
#include <fstream>
#include <cmath>

#include <SyntX/util/parser/parser.h>

#include "Generator.h"

using namespace util::parser;
using namespace std;

int main(int argc, char* argv[]) {
  base_rule::set_build_ast(true);

  //LANGUAGE
    
  //example
  /*
  rule addition, addend("addend"), expression;
  addition <<= -addend << *(-character("+") << -addend);
  addend <<= -range('0', '9') | -expression;
  expression <<= -character("(") << -addition << -character(")");
  */
  /////

  //LTL
  rule ltl;

  //Lexer rules
  rule ReleaseOp;
  ReleaseOp <<= character("R") | character("W");
  rule UntilOp;
  UntilOp <<= character("U");
  rule ImpicationOp;
  ImpicationOp <<= identifier("->") | identifier("=>") | identifier("-->") | identifier("imply") | identifier("i");
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
  NextOp <<= character("!") | character("~") | identifier("not");
  rule LPAR;
  LPAR <<= character("(");
  rule RPAR;
  RPAR <<= character(")");
  rule ExactProposition;
  //TODO
  rule AtomicProposition;
  AtomicProposition <<= range('0', '9');
    
  //Parser rules
  rule ParenthesizedLtl("ParenthesizedLtl");
  ParenthesizedLtl <<= -LPAR << -ltl << -RPAR;

  rule Proposition("Proposition");
  Proposition <<= -AtomicProposition | ltl;

  rule Not("Not");
  Not <<= -NotOp << -ltl;

  rule WeakNext("WeakNext");
  WeakNext <<= -WeakNextOp << -ltl;

  rule Next("Next");
  Next <<= -NextOp << -ltl;

  rule Future("Future");
  Future <<= -FutureOp << -ltl;

  rule Globally("Globally");
  Globally <<= -GloballyOp << -ltl;

  rule Release("Release");
  Release <<= -ltl << -ReleaseOp << -ltl;

  rule Until("Until");
  Until <<= -ltl << UntilOp << -ltl;

  rule And("And");
  And <<= -ltl << AndOp << -ltl;

  rule Or("Or");
  Or <<= -ltl << OrOp << -ltl;

  rule Xor("Xor");
  Xor <<= -ltl << XorOp << -ltl;

  ltl <<=
    ParenthesizedLtl |
    Proposition |
    Not |
    WeakNext |
    Next |
    Future |
    Globally |
    Release |
    Until |
    And |
    Or |
    Xor;

  /*
  rule ltlexpression;
  ltlexpression <<= 
    LPAR <<= ltlexpression <<= RPAR |
    NotOp <<= ltlexpression |
    WeakNextOp <<= ltlexpression |
    NextOp <<= ltlexpression |
    FutureOp <<= ltlexpression |
  */


  //ltlexpression <<= 

  //LANGUAGE END
  std::string input = "G( 1 & 2 ) \n";
  base_rule::match_range context(input.cbegin(), input.cend());
  base_rule::match_range result;
  std::shared_ptr<base_rule::node> root;

  if (ltl.match(context, result, root)) {
    std::cout << "Matched: " << std::string(result.first, result.second) << std::endl;

    ast_draw printer;
    printer.to_formatted_string(root);
    printer.draw_to_file(root);
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