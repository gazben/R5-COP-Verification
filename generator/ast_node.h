#ifndef ast_node_h__
#define ast_node_h__

#include <SyntX/util/parser/parser.h>
#include <string>
#include <memory>

#include "Function.h"
/*
Structure used for the Connection Normal form generation.
*/
struct ast_node{
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

  static std::string to_string(ast_node* node);
  /*
  trilean EVAL_s1a(Property* _prop)
  {
  return
  NAND_3( NOT_3(_prop->isEventFired(EVENT_B)), NAND_3(_prop->isEventFired(EVENT_C), _prop->InputStates()[1])
  );
  }
  */

  std::string ast_node::getFunctionString()
  {
    std::string result;

    if( the_value == "And" )
      result += "AND_3";
    else if (the_value == "Or")
      result += "OR_3";
    else if (the_value == "Not")
      result += "NOT_3";
    else
      result += "VALUE";

    return result + "(" + ((leftChildren)? left_children()->getFunctionString():"") + ((rightChildren)?(", " + right_children()->getFunctionString()):"") + ")";
  }

  std::string ast_node::getDeclarationString()
  {
    throw std::logic_error("The method or operation is not implemented.");
  }

  std::string ast_node::getSignature()
  {
    throw std::logic_error("The method or operation is not implemented.");
  }

};
#endif // ast_node_h__