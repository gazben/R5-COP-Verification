#include "ast_optimizer.h"

std::shared_ptr<base_rule::node> AstOptimizer::optimizeAst(std::shared_ptr<base_rule::node> &node)
{
  removeAlternations(node);
  removeCharacterLeafs(node);
  removeOneChildrenRoots(node);
  rearrangeOperators(node);
  removeNodesMarkedForDeletion(node);
  removeOneChildrenRoots(node);
  removeLparRpar(node);
  removeAlternations(node);
  rearrangeOneOperandOperators(node);

  return node;
}

std::shared_ptr<base_rule::node> AstOptimizer::rearrangeOperators(std::shared_ptr<base_rule::node> node)
{
  if (node == nullptr)
    return nullptr;

  if (
    node->parent != nullptr &&
    node->parent->parent != nullptr &&
    node->the_type == base_rule::node::type::named_rule)
  {
    auto new_place = node->parent->parent;
    new_place->the_type = node->the_type;
    new_place->the_value = node->the_value;

    for (unsigned int i = 0; i < node->parent->children.size(); i++)
    {
      auto entry = node->parent->children[i];

      if (entry->the_type == base_rule::node::type::named_rule)
      {
        if (entry->children.size() == 0)
        {
          node->the_type = base_rule::node::type::deleted;
          node->parent->children.erase(node->parent->children.begin() + i);
        }
      }
      else if (entry->the_type == base_rule::node::type::value)
      {
        node->parent->the_type = entry->the_type;
        node->parent->the_value = entry->the_value;

        if (entry->children.size() == 0) {
          entry->the_type = base_rule::node::type::deleted;
        }
      }
    }
  }

  rearrangeOperators(node->left_children());
  rearrangeOperators(node->right_children());

  return node;
}

std::shared_ptr<base_rule::node> AstOptimizer::rearrangeOneOperandOperators(std::shared_ptr<base_rule::node> node)
{
  if (node == nullptr)
    return nullptr;

  if (
    node->parent != nullptr &&
    node->parent->parent == nullptr &&
    node->children.size() == 0 &&
    node->parent->the_type == base_rule::node::type::repetition_or_epsilon &&
    node->the_type == base_rule::node::type::named_rule)
  {
    auto new_place = node->parent;
    new_place->the_type = node->the_type;
    new_place->the_value = node->the_value;

    for (unsigned int i = 0; i < node->parent->children.size(); i++)
    {
      auto entry = node->parent->children[i];

      if (entry->the_type == base_rule::node::type::named_rule)
      {
        if (entry->children.size() == 0)
        {
          node->the_type = base_rule::node::type::deleted;
          node->parent->children.erase(node->parent->children.begin() + i);
        }
      }
      else if (entry->the_type == base_rule::node::type::value)
      {
        node->parent->the_type = entry->the_type;
        node->parent->the_value = entry->the_value;

        if (entry->children.size() == 0) {
          entry->the_type = base_rule::node::type::deleted;
        }
      }
    }
  }

  rearrangeOneOperandOperators(node->left_children());
  rearrangeOneOperandOperators(node->right_children());

  return node;
}

std::shared_ptr<base_rule::node> AstOptimizer::removeLparRpar(std::shared_ptr<base_rule::node> node)
{
  if (node == nullptr)
    return nullptr;

  for (unsigned int i = 0; i < node->children.size(); i++)
  {
    auto& entry = node->children[i];

    if (entry->the_value == "(" || entry->the_value == ")")
    {
      entry->parent->children.erase(entry->parent->children.begin() + i);
    }
  }

  removeLparRpar(node->left_children());
  removeLparRpar(node->right_children());

  return node;
}

std::shared_ptr<base_rule::node> AstOptimizer::removeCharacterLeafs(std::shared_ptr<base_rule::node> node)
{
  if (node == nullptr)
    return nullptr;

  if (node->children.size() == 1 && node->the_type == base_rule::node::type::named_rule)
  {
    node->children.clear();
  }

  for (auto &entry : node->children)
    removeCharacterLeafs(entry);

  return node;
}

std::shared_ptr<base_rule::node> AstOptimizer::removeAlternations(std::shared_ptr<base_rule::node> node)
{
  if (node == nullptr)
    return nullptr;

  for (unsigned int i = 0; i < node->children.size(); i++)
  {
    auto& entry = node->children[i];

    if (entry->children.size() == 1
      && ((entry->the_type == base_rule::node::type::alternation)
        || (entry->the_type == base_rule::node::type::concatenation)))
    {
      entry = entry->children.front();
      entry->parent = node;
      removeAlternations(node);
    }
  }

  for (auto &entry : node->children)
    removeAlternations(entry);

  return node;
}

std::shared_ptr<base_rule::node> AstOptimizer::removeOneChildrenRoots(std::shared_ptr<base_rule::node> node)
{
  if (node == nullptr)
    return nullptr;

  while ((node->children.size() == 1) && (node->children.front()->the_type != base_rule::node::type::value))
  {
    auto new_children = node->children.front()->children;
    node->children.clear();
    node->children = new_children;
    for (auto entry : new_children)
      entry->parent = node;
  }

  removeOneChildrenRoots(node->left_children());
  removeOneChildrenRoots(node->right_children());

  return node;
}

void AstOptimizer::removeNodesMarkedForDeletion(std::shared_ptr<base_rule::node> node)
{
  if (node == nullptr)
    return;

  removeNodesMarkedForDeletion(node->left_children());
  removeNodesMarkedForDeletion(node->right_children());

  while (node->parent != nullptr
    && node->parent->children.size() != 0
    && node->parent->left_children().get()->the_type == base_rule::node::type::deleted)
  {
    node->parent->children.erase(node->parent->children.begin());
  }
}