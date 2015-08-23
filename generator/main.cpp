#include <iostream>
#include <fstream>
#include <cmath>

#include <SyntX/util/languages/ltl.h>
#include <SyntX/util/parser/parser.h>

#include "Generator.h"
#include "ConnectionNormalFormGenerator.h"
#include "ast_optimizer.h"
#include "BlockGenerator.h"

using namespace util::parser;
using namespace std;

int main(int argc, char* argv[]) {
  base_rule::set_build_ast(true);

  //std::string input = "G (((8 | 9) ^ 4) U (1 & 2))\n";
  std::string input = "G(1 => (2 U 3))";

  cout << "ROS Automatic monitor generator BETA. Made by Gazder Bence" << endl;
  //cout << "Please enter the expression!" << endl;
  //getline(std::cin, input);

  base_rule::match_range context(input.cbegin(), input.cend());
  base_rule::match_range result;
  std::shared_ptr<base_rule::node> root;

  if (ltl().match(context, result, root)) {
    std::cout << "Matched: " << std::string(result.first, result.second) << std::endl;

    ast_optimizer::optimize_ast(root);
    
    ConnectionNormalFormGenerator converter;
    ast_node* normalFormRoot = converter.convertToConnectionNormalForm(root);

    BlockGenerator block_generator(normalFormRoot);
    block_generator.createBlocks();

    Generator gen(&block_generator);
    gen.Generate();
    cout << "Generation completed." << endl;
    cout << "Given expression: " + input << endl;
  }
  else {
    std::cout << "Didn't match" << std::endl;
  }

  cout << endl << "Press any key to quit." << endl;
  
  getchar();

  return 0;
}