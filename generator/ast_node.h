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
  unsigned int convertedCount;  //used for Until operators to determinate, how deap is the converting
  unsigned int currentInterfaceID;
  static unsigned int globalInterfaceID;

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
  void deleteChildren();

  static std::string to_string(ast_node* node);
  std::string getFunctionString();
  void setInterfaceID();
};
#endif // ast_node_h__