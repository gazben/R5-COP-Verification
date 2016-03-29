#ifndef ast_node_h__
#define ast_node_h__

/* GLOBAL INCLUDES */
#include <string>
#include <memory>

/* LOCAL INCLUDES */
#include "generator_framework.h"
#include "syntx/parser.h"
/* INCLUDES END */


/*
Structure used for the Connection Normal form generation.
The structure represents a binary tree node.
*/
struct AstNode
{
  AstNode* left_children;
  AstNode* right_children;
  AstNode* parent;

  base_rule::node::type the_type;
  std::string the_value;

  unsigned int block_id;
  unsigned int converted_count;            //used for Until operators to determinate, how deap is the converting
  unsigned int current_interface_id;
  static unsigned int global_interface_id;

  AstNode();
  AstNode(std::string value);
  AstNode(base_rule::node::type type);
  AstNode(base_rule::node::type type, std::string value);

  AstNode* getLeftChildren();
  AstNode* getRightChildren();

  AstNode* clone(AstNode* _parent = nullptr);
  AstNode* cloneUntilNext(AstNode* _parent = nullptr);

  static void freeAst(AstNode* node);
  void addChildren(AstNode* node);
  void nullChildren();

  static std::string toString(AstNode* node);
  std::string getFunctionString();
  void setInterfaceID();
};
#endif // ast_node_h__