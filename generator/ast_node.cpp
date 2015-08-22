#include "ast_node.h"

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

ast_node::ast_node() : leftChildren(nullptr), rightChildren(nullptr), parent(nullptr), blockID(0), convertedCount(0)
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

ast_node * ast_node::cloneUntilNext(ast_node* _parent)
{
  if (the_type == base_rule::node::type::named_rule && the_value == "Next")
    return nullptr;

  ast_node* result = new ast_node(the_type, the_value);
  result->parent = _parent;
  if (right_children())
    result->rightChildren = right_children()->clone(result);
  if (left_children())
    result->leftChildren = left_children()->clone(result);

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

void ast_node::nullChildren()
{
  leftChildren = nullptr;
  rightChildren = nullptr;
}