#ifndef ConnectionNormalFormGenerator_h__
#define ConnectionNormalFormGenerator_h__

#include <memory>

#include "SyntX/util/parser/parser.h"
#include "ast_node.h"
/*
This class takes an AST and builds up a new AST with ConnectionNormalForm
Example(25-26):
http://home.mit.bme.hu/~majzik/dl/monitor/Pallagi_Peter_szakdolgozat_vegleges.pdf
*/

class ConnectionNormalFormGenerator
{
public:
  std::shared_ptr<base_rule::node> getOriginalRoot();
  void setOriginalRoot(std::shared_ptr<base_rule::node> val);
  void convertOneMOreUntilLevel(AstNode* root);
  AstNode* convertToConnectionNormalForm(std::shared_ptr<base_rule::node>& _root);
  AstNode * addNextOpToRoot(AstNode * node);
  void freeAst();
  int getUntilDeepness();

private:
  std::shared_ptr<base_rule::node> original_root;
  AstNode* root_node;
  static int until_node_deepness;

  AstNode* copyAST(std::shared_ptr<base_rule::node> node, AstNode* parent = nullptr);

  void convertGenerallyOperators(AstNode* node);
  void convertFutureOperators(AstNode* node);
  void convertImplicationOperators(AstNode* node);
  void convertOrOperators(AstNode* node);
  void convertUntilOperators(AstNode* node, size_t depth = 0);
  void convertNegateOperators(AstNode* node);
};

#endif // ConnectionNormalFormGenerator_h__
