#include "ast_node.h"

unsigned int ast_node::globalInterfaceID = 0;

ast_node::ast_node(base_rule::node::type _type, std::string _value) :ast_node()
{
  the_type = _type;
  the_value = _value;
}

ast_node::ast_node(base_rule::node::type _type) :ast_node()
{
  the_type = _type;
}

ast_node::ast_node(std::string _value) : ast_node()
{
  the_value = _value;
}

ast_node::ast_node() : leftChildren(nullptr), rightChildren(nullptr), parent(nullptr), blockID(0), convertedCount(0), currentInterfaceID(0)
{
}

ast_node* ast_node::left_children()
{
  return leftChildren;
}

ast_node* ast_node::right_children()
{
  return rightChildren;
}

ast_node* ast_node::clone(ast_node* _parent /*= nullptr*/)
{
  ast_node* result = new ast_node(the_type, the_value);
  result->parent = _parent;
  if (right_children())
    result->rightChildren = right_children()->clone(result);
  if (left_children())
    result->leftChildren = left_children()->clone(result);

  return result;
}

ast_node* ast_node::cloneUntilNext(ast_node* _parent)
{
  if (the_type == base_rule::node::type::named_rule && the_value == "Next") {
    ast_node* result = new ast_node(the_type, the_value);
    result->setInterfaceID();
    result->globalInterfaceID++;
    result->parent = _parent;
    return result;
    //return nullptr;
  }

  ast_node* result = new ast_node(the_type, the_value);
  result->parent = _parent;
  if (left_children())
    result->leftChildren = left_children()->cloneUntilNext(result);
  if (right_children())
    result->rightChildren = right_children()->cloneUntilNext(result);
  return result;
}

void ast_node::free_ast(ast_node* node)
{
  if (node == nullptr)
    return;

  free_ast(node->left_children());
  free_ast(node->right_children());
  delete node;
}

void ast_node::add_children(ast_node* node)
{
  if (leftChildren == nullptr)
    leftChildren = node;
  else if (rightChildren == nullptr)
    rightChildren = node;
  else
    throw std::runtime_error("Too many children!");
}

void ast_node::deleteChildren()
{
  leftChildren = nullptr;
  rightChildren = nullptr;
}

std::string ast_node::to_string(ast_node* node)
{
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

void ast_node::setInterfaceID()
{
  currentInterfaceID = globalInterfaceID;
}

std::string ast_node::getFunctionString()
{
  std::string result;

  if (the_value == "And")
    result += "AND_3";
  else if (the_value == "Or")
    result += "OR_3";
  else if (the_value == "Not")
    result += "NOT_3";
  else if (the_value == "Next") {
    result += "_prop->inputStates[" + std::to_string(currentInterfaceID) + "]";
    return result;
  }
  else if (the_value == "1") {
    result += "_prop->isEventFired(EVENT_UP)";
    return result;
  }
  else if (the_value == "2") {
    result += "_prop->isEventFired(EVENT_DOWN)";
    return result;
  }
  else if (the_value == "3") {
    result += "_prop->isEventFired(EVENT_RIGHT)";
    return result;
  }
  else if (the_value == "True") {
    result += "TRUE";
    return result;
  }
  else if (the_value == "False") {
    result += "FALSE";
    return result;
  }
  else
    result += "VALUE";

  return result + "(" + ((leftChildren) ? left_children()->getFunctionString() : "") + ((rightChildren) ? (", " + right_children()->getFunctionString()) : "") + ")";
}