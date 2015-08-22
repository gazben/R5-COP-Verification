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
  int untilDeepness = 1;

  ast_node* copyAST(std::shared_ptr<base_rule::node> node, ast_node* parent = nullptr) {
    ast_node* result = new ast_node(node->the_type, node->the_value);
    result->parent = parent;
    if (node->right_children())
      result->rightChildren = copyAST(node->right_children(), result);
    if (node->left_children())
      result->leftChildren = copyAST(node->left_children(), result);

    return result;
  }

  void convertGenerallyOperators(ast_node* node) {
    if (node == nullptr) {
      return;
    }

    if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Globally") {
      node->the_type = base_rule::node::type::named_rule;
      node->the_value = "Not";

      ast_node* future_node = new ast_node(base_rule::node::type::named_rule, "Future");
      future_node->parent = node;

      ast_node* not_node = new ast_node(base_rule::node::type::named_rule, "Not");
      not_node->parent = future_node;

      not_node->leftChildren = node->left_children();
      not_node->rightChildren = node->right_children();

      node->nullChildren();
      node->leftChildren = future_node;
      future_node->leftChildren = not_node;
    }

    convertGenerallyOperators(node->left_children());
    convertGenerallyOperators(node->right_children());
  }

  void convertFutureOperators(ast_node* node) {
    if (node == nullptr) {
      return;
    }

    if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Future") {
      node->the_type = base_rule::node::type::named_rule;
      node->the_value = "Until";

      ast_node* true_node = new ast_node(base_rule::node::type::value, "True");
      true_node->parent = node;

      node->rightChildren = node->left_children();
      node->leftChildren = true_node;
    }

    convertFutureOperators(node->left_children());
    convertFutureOperators(node->right_children());
  }

  void convertImplicationOperators(ast_node* node) {
    if (node == nullptr) {
      return;
    }

    if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Implication") {
      node->the_type = base_rule::node::type::named_rule;
      node->the_value = "Or";

      ast_node* not_node = new ast_node(base_rule::node::type::named_rule, "Not");
      not_node->parent = node;
      not_node->add_children(node->left_children());

      node->leftChildren = not_node;
    }

    convertImplicationOperators(node->left_children());
    convertImplicationOperators(node->right_children());
  }
  void convertOrOperators(ast_node* node) {
    if (node == nullptr) {
      return;
    }

    if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Or") {
      node->the_type = base_rule::node::type::named_rule;
      node->the_value = "Not";

      ast_node* and_node = new ast_node(base_rule::node::type::named_rule, "And");
      and_node->parent = node;

      ast_node* not_node_left = new ast_node(base_rule::node::type::named_rule, "Not");
      ast_node* not_node_right = new ast_node(base_rule::node::type::named_rule, "Not");
      and_node->leftChildren = not_node_left;
      and_node->rightChildren = not_node_right;

      not_node_left->parent = and_node;
      not_node_left->add_children(node->left_children());
      not_node_right->parent = and_node;
      not_node_right->add_children(node->right_children());

      node->nullChildren();
      node->leftChildren = and_node;
    }

    convertOrOperators(node->left_children());
    convertOrOperators(node->right_children());
  }

  void convertUntilOperators(ast_node* node, size_t depth = 0) {
    if (node == nullptr) {
      return;
    }


    if (node->the_type == base_rule::node::type::named_rule && 
      node->the_value == "Until"  &&
      //depth < 4
      node->convertedCount < untilDeepness
      ) 
    {
      node->the_type = base_rule::node::type::named_rule;
      node->the_value = "Or";

      ast_node* and_node = new ast_node(base_rule::node::type::named_rule, "And");
      ast_node* next_node = new ast_node(base_rule::node::type::named_rule, "Next");
      ast_node* until_node = new ast_node(base_rule::node::type::named_rule, "Until");

      and_node->parent = node;
      next_node->parent = and_node;
      until_node->parent = next_node;

      ast_node* childrenBuffer[2] = { node->left_children(), node->right_children() };

      node->leftChildren = childrenBuffer[1]->clone(node);
      node->rightChildren = and_node;

      and_node->leftChildren = childrenBuffer[0]->clone(and_node);
      and_node->rightChildren = next_node;
      next_node->leftChildren = until_node;
      until_node->leftChildren = childrenBuffer[0];
      until_node->rightChildren = childrenBuffer[1];

      until_node->convertedCount = node->convertedCount + 1;
      node->convertedCount = 0;
    }

    convertUntilOperators(node->left_children(), depth + 1);
    convertUntilOperators(node->right_children(), depth + 1);
  }
  void convertNegateOperators(ast_node* node) {
    if (node == nullptr) {
      return;
    }

    if (
      (node->the_type == base_rule::node::type::named_rule && node->the_value == "Not") &&
      (node->left_children()->the_type == base_rule::node::type::named_rule && node->left_children()->the_value == "Not"))
    {
      node->the_type = node->left_children()->left_children()->the_type;
      node->the_value = node->left_children()->left_children()->the_value;
      node->left_children()->left_children()->parent = node;

      ast_node* childrenTemp = node->left_children()->left_children();

      node->leftChildren = childrenTemp->left_children();
      node->rightChildren = childrenTemp->right_children();

      //TODO free up memory of NOT node
    }

    convertNegateOperators(node->left_children());
    convertNegateOperators(node->right_children());
  }

public:
  std::shared_ptr<base_rule::node> getOriginalRoot() { return originalRoot; }
  void setOriginalRoot(std::shared_ptr<base_rule::node> val) { originalRoot = val; }

  void convertOneMOreUntilLevel(ast_node* root) {
    untilDeepness++;
    convertUntilOperators(root);
  }

  auto convertToConnectionNormalForm(std::shared_ptr<base_rule::node>& _root) {
    originalRoot = _root;
    rootNode = copyAST(originalRoot);

    convertGenerallyOperators(rootNode);
    convertFutureOperators(rootNode);
    convertImplicationOperators(rootNode);
    convertUntilOperators(rootNode);
    convertOrOperators(rootNode);
    convertNegateOperators(rootNode);

    return rootNode;
  }

  void free_ast() {
    ast_node::free_ast(rootNode);
  }

  int getUntilDeepness() {
    return untilDeepness;
  }
};

#endif // ConnectionNormalFormGenerator_h__
