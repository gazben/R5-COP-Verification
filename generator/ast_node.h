#ifndef ast_node_h__
#define ast_node_h__

#include <SyntX/util/parser/parser.h>
#include <string>
#include <memory>
/*
Structure used for the Connection Normal form generation.
*/
struct ast_node {
  ast_node* leftChildren;
  ast_node* rightChildren;
  ast_node* parent;
  base_rule::node::type the_type;
  std::string the_value;
  unsigned int blockID;
  unsigned int convertedCount;  //used for Until operators to determinate, how deap the converting

  ast_node();
  ast_node(std::string _value);
  ast_node(base_rule::node::type _type);
  ast_node(base_rule::node::type _type, std::string _value);

  ast_node* left_children();
  ast_node* right_children();

  ast_node* clone(ast_node* _parent = nullptr);
  ast_node* cloneUntilNext(ast_node* _parent = nullptr);


  static void free_ast(ast_node* node);
  void add_children(ast_node* node);
  void nullChildren();

  static std::string to_string(ast_node* node) {
    if (node == nullptr || node->the_value == "Next")
      return "";

    std::string text;
    switch (node->the_type) {
    case base_rule::node::type::value:
      text = node->the_value;
      break;
    case base_rule::node::type::alternation:
      text = "a";
      break;
    case base_rule::node::type::concatenation:
      text = "c";
      break;
    case base_rule::node::type::option:
      text = "o";
      break;
    case base_rule::node::type::repetition:
      text = "r";
      break;
    case base_rule::node::type::repetition_or_epsilon:
      text = "R";
      break;
    case base_rule::node::type::named_rule:
      text = node->the_value;
      break;
    }
    return text + to_string(node->leftChildren) + to_string(node->rightChildren);
  }
};
#endif // ast_node_h__