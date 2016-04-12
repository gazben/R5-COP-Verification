#include "ast_node.h"

unsigned int AstNode::global_interface_id = 0;

AstNode::AstNode(base_rule::node::type _type, std::string _value) :AstNode()
{
  the_type = _type;
  the_value = _value;
}

AstNode::AstNode(base_rule::node::type _type) :AstNode()
{
  the_type = _type;
}

AstNode::AstNode(std::string _value) : AstNode()
{
  the_value = _value;
}

AstNode::AstNode() : left_children(nullptr), right_children(nullptr), parent(nullptr), block_id(0), converted_count(0), current_interface_id(0)
{
}

AstNode* AstNode::getLeftChildren()
{
  return left_children;
}

AstNode* AstNode::getRightChildren()
{
  return right_children;
}

AstNode* AstNode::clone(AstNode* _parent /*= nullptr*/)
{
  AstNode* result = new AstNode(the_type, the_value);
  result->parent = _parent;
  if (getRightChildren())
    result->right_children = getRightChildren()->clone(result);
  if (getLeftChildren())
    result->left_children = getLeftChildren()->clone(result);

  return result;
}

AstNode* AstNode::cloneUntilNext(AstNode* _parent)
{
  if (the_type == base_rule::node::type::named_rule && the_value == "Next") {
    AstNode* result = new AstNode(the_type, the_value);
    result->setInterfaceID();
    result->global_interface_id++;
    result->parent = _parent;
    return result;
  }

  AstNode* result = new AstNode(the_type, the_value);
  result->parent = _parent;
  if (getLeftChildren())
    result->left_children = getLeftChildren()->cloneUntilNext(result);
  if (getRightChildren())
    result->right_children = getRightChildren()->cloneUntilNext(result);
  return result;
}

void AstNode::freeAst(AstNode* node)
{
  if (node == nullptr)
    return;

  freeAst(node->getLeftChildren());
  freeAst(node->getRightChildren());
  delete node;
}

void AstNode::addChildren(AstNode* node)
{
  if (getLeftChildren() == nullptr)
    left_children = node;
  else if (getRightChildren() == nullptr)
    right_children = node;
  else
    throw std::runtime_error("Too many children!");
}

void AstNode::nullChildren()
{
  left_children = nullptr;
  right_children = nullptr;
}

std::string AstNode::toString(AstNode* node)
{
  if (node == nullptr || node->the_value == "Next")
    return "";

  std::string text;
  switch (node->the_type)
  {
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
  case base_rule::node::type::value:
  case base_rule::node::type::named_rule:
    if (node->getRightChildren() == nullptr && node->getLeftChildren() == nullptr) {
      if (node->the_value[0] == '\'' && node->the_value[node->the_value.size()-1] == '\'') {
        auto event_name = node->the_value;
        event_name.erase(0, 1);
        event_name.erase(event_name.size() - 1, 1);
        text = event_name;
      }
    } 
    else 
    {
      text = node->the_value;
    }
    break;
  }
  return text + toString(node->left_children) + toString(node->right_children);
}

void AstNode::setInterfaceID()
{
  current_interface_id = global_interface_id;
}

std::string AstNode::getFunctionString()
{
  std::string result;

  if (the_value == "And")
    result += "AND_3";
  else if (the_value == "Or")
    result += "OR_3";
  else if (the_value == "Not")
    result += "NOT_3";
  else if (the_value == "Next")
  {
    result += "property->input_states[" + std::to_string(current_interface_id) + "]";
    return result;
  }
  else if (the_value == "True")
  {
    result += "TRUE";
    return result;
  }
  else if (the_value == "False")
  {
    result += "FALSE";
    return result;
  }
  else
  {
    if (the_type != base_rule::node::type::repetition) {
      if (the_value[0] == '\'' && the_value[the_value.size() - 1] == '\'') {
        the_value.erase(0, 1);
        the_value.erase(the_value.size() - 1, 1);
      }
      result += "property->isEventFired(" + the_value + ")";
      if (the_value == "")
      {
        result += "0";
      }
      return result;
    }
  }

  return result + "(" + ((left_children) ? getLeftChildren()->getFunctionString() : "") + ((right_children) ? (", " + getRightChildren()->getFunctionString()) : "") + ")";
}