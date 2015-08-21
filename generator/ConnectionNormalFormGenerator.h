#ifndef ConnectionNormalFormGenerator_h__
#define ConnectionNormalFormGenerator_h__

#include <memory>
#include <SyntX/util/parser/parser.h>

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
  std::shared_ptr<base_rule::node> root;

  void convertGenerallyOperators(std::shared_ptr<base_rule::node> node) {
    if (node == nullptr) {
      return;
    }

    if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Globally") {
      node->the_type = base_rule::node::type::named_rule;
      node->the_value = "Not";

      std::shared_ptr<base_rule::node> future_node = std::make_shared<base_rule::node>(base_rule::node::type::named_rule, "Future");
      future_node->parent = node;

      std::shared_ptr<base_rule::node> not_node = std::make_shared<base_rule::node>(base_rule::node::type::named_rule, "Not");
      not_node->parent = future_node;

      not_node->children = node->children;
      node->children.clear();
      node->children.push_back(future_node);
      future_node->children.push_back(not_node);
    }

    convertGenerallyOperators(node->left_children());
    convertGenerallyOperators(node->right_children());
  }

  void convertFutureOperators(std::shared_ptr<base_rule::node> node) {
    if (node == nullptr) {
      return;
    }

    if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Future") {
      node->the_type = base_rule::node::type::named_rule;
      node->the_value = "Until";

      std::shared_ptr<base_rule::node> true_node = std::make_shared<base_rule::node>(base_rule::node::type::value, "True");
      true_node->parent = node;
      //TODO: error handling (if children.size() == 2)
      node->children.push_back(true_node);
    }

    convertFutureOperators(node->left_children());
    convertFutureOperators(node->right_children());
  }

  void convertImplicationOperators(std::shared_ptr<base_rule::node> node) {
    if (node == nullptr) {
      return;
    }

    if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Implication") {
      node->the_type = base_rule::node::type::named_rule;
      node->the_value = "Or";

      std::shared_ptr<base_rule::node> not_node = std::make_shared<base_rule::node>(base_rule::node::type::named_rule, "Not");
      not_node->parent = node;
      not_node->children.push_back(node->children[0]);

      node->children[0] = not_node;
    }

    convertImplicationOperators(node->left_children());
    convertImplicationOperators(node->right_children());
  }
  void convertOrOperators(std::shared_ptr<base_rule::node> node) {
    if (node == nullptr) {
      return;
    }

    if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Or") {
      node->the_type = base_rule::node::type::named_rule;
      node->the_value = "Not";

      std::shared_ptr<base_rule::node> and_node = std::make_shared<base_rule::node>(base_rule::node::type::named_rule, "And");
      and_node->parent = node;

      std::shared_ptr<base_rule::node> not_node_left = std::make_shared<base_rule::node>(base_rule::node::type::named_rule, "Not");
      std::shared_ptr<base_rule::node> not_node_right = std::make_shared<base_rule::node>(base_rule::node::type::named_rule, "Not");
      and_node->children.push_back(not_node_left);
      and_node->children.push_back(not_node_right);

      not_node_left->parent = and_node;
      not_node_left->children.push_back(node->children[0]);
      not_node_right->parent = and_node;
      not_node_right->children.push_back(node->children[1]);

      node->children.clear();
      node->children.push_back(and_node);
    }

    convertOrOperators(node->left_children());
    convertOrOperators(node->right_children());
  }

  void convertUntilOperators(std::shared_ptr<base_rule::node> node) {
    if (node == nullptr) {
      return;
    }
    convertUntilOperators(node->left_children()); //To avoid endless loop
    convertUntilOperators(node->right_children());

    if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Until") {
      node->the_type = base_rule::node::type::named_rule;
      node->the_value = "Or";

      std::shared_ptr<base_rule::node> and_node = std::make_shared<base_rule::node>(base_rule::node::type::named_rule, "And");
      std::shared_ptr<base_rule::node> next_node = std::make_shared<base_rule::node>(base_rule::node::type::named_rule, "Next");
      std::shared_ptr<base_rule::node> until_node = std::make_shared<base_rule::node>(base_rule::node::type::named_rule, "Not");

      and_node->parent = node;
      next_node->parent = and_node;
      until_node->parent = next_node;

      until_node->children = node->children;

      node->children.clear();
      node->children.push_back(until_node->children[1]);
      node->children.push_back(and_node);

      and_node->children.push_back(until_node->children[0]);
      and_node->children.push_back(next_node);
      next_node->children.push_back(until_node);
    }
  }
  void convertNegateOperators(std::shared_ptr<base_rule::node>& node) {
    if (node == nullptr) {
      return;
    }

    if ((node->the_type == base_rule::node::type::named_rule && node->the_value == "Not") &&
      (node->children[0]->the_type == base_rule::node::type::named_rule && node->children[0]->the_value == "Not")){

      node->the_type = node->children[0]->children[0]->the_type;
      node->the_value = node->children[0]->children[0]->the_value;
      node->children[0]->children[0]->parent = node;
      node->children = node->children[0]->children[0]->children;
    }

    convertNegateOperators(node->left_children());
    convertNegateOperators(node->right_children());
  }

public:
  std::shared_ptr<base_rule::node> getRoot() { return root; }
  void setRoot(std::shared_ptr<base_rule::node> val) { root = val; }

  std::shared_ptr<base_rule::node> convertToConnectionNormalForm(std::shared_ptr<base_rule::node> _root) {
    root = _root;

    convertGenerallyOperators(root);
    convertFutureOperators(root);
    convertImplicationOperators(root);
    convertOrOperators(root);
    convertUntilOperators(root);
    convertNegateOperators(root);

    return root;
  }
};

#endif // ConnectionNormalFormGenerator_h__
