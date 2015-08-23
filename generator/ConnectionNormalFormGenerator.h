#ifndef ConnectionNormalFormGenerator_h__
#define ConnectionNormalFormGenerator_h__

#include <memory>
#include <SyntX/util/parser/parser.h>

#include "ast_node.h"
/*
This class takes an AST and builds up a new AST with ConnectionNormalForm
Example(25-26):
http://home.mit.bme.hu/~majzik/dl/monitor/Pallagi_Peter_szakdolgozat_vegleges.pdf
Rules:
  - Generally: G(exp) ? ?F(?exp)
  - Future: F(exp) ? true U exp
  - Implication: exp1 ? exp2 ? ?exp1 V exp2
  - Or:  exp1 V exp2 ? ?(?exp1 ? ?exp2)
  - Until: exp1 U exp2 ? exp2 V (exp1 ? X(exp1 U exp2))
  - Negate:  ?(?exp) ? exp

*/

class ConnectionNormalFormGenerator {
private:
  std::shared_ptr<base_rule::node> originalRoot;
  ast_node* rootNode;
  static int untilDeepness;

  ast_node* copyAST(std::shared_ptr<base_rule::node> node, ast_node* parent = nullptr);

  void convertGenerallyOperators(ast_node* node);
  void convertFutureOperators(ast_node* node);
  void convertImplicationOperators(ast_node* node);
  void convertOrOperators(ast_node* node);
  void convertUntilOperators(ast_node* node, size_t depth = 0);
  void convertNegateOperators(ast_node* node);

public:
  std::shared_ptr<base_rule::node> getOriginalRoot();
  void setOriginalRoot(std::shared_ptr<base_rule::node> val);
  void convertOneMOreUntilLevel(ast_node* root);
  ast_node* convertToConnectionNormalForm(std::shared_ptr<base_rule::node>& _root);
  ast_node * add_nextop_to_root(ast_node * node);
  void free_ast();
  int getUntilDeepness();

};

#endif // ConnectionNormalFormGenerator_h__
