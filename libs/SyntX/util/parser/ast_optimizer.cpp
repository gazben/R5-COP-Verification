#include "ast_optimizer.h"



std::shared_ptr<base_rule::node> ast_optimizer::optimize_ast(std::shared_ptr<base_rule::node> &node)
{
  remove_alternations(node);
  remove_character_leafs(node);
  remove_one_children_roots(node);
  rearrange_operators(node);
  remove_nodes_marked_for_deletion(node);
  remove_one_children_roots(node);
  remove_lpar_rpar(node);
  remove_alternations(node);

  return node;
}

std::shared_ptr<base_rule::node> ast_optimizer::rearrange_operators(std::shared_ptr<base_rule::node> &node)
{
  if (node == nullptr)
    return nullptr;

  if (
    node->parent != nullptr &&
    node->parent->parent != nullptr &&
    node->the_type == base_rule::node::type::named_rule) {


    auto new_place = node->parent->parent;
    new_place->the_type = node->the_type;
    new_place->the_value = node->the_value;

    for (unsigned int i = 0; i < node->parent->children.size(); i++) {
      auto entry = node->parent->children[i];

      if (entry->the_type == base_rule::node::type::named_rule) {
        if (entry->children.size() == 0) {
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
          //entry->parent->children.erase(node->parent->children.begin() + i);
        }
      }
    }
  }

  rearrange_operators(node->left_children());
  rearrange_operators(node->right_children());

  return node;
}

std::shared_ptr<base_rule::node> ast_optimizer::remove_lpar_rpar(std::shared_ptr<base_rule::node> &node)
{
  if (node == nullptr)
    return nullptr;

  for (unsigned int i = 0; i < node->children.size(); i++) {
    auto& entry = node->children[i];

    if (entry->the_value == "(" || entry->the_value == ")") {
      entry->parent->children.erase(entry->parent->children.begin() + i);
    }
  }

  remove_lpar_rpar(node->left_children());
  remove_lpar_rpar(node->right_children());

  return node;
}

std::shared_ptr<base_rule::node> ast_optimizer::remove_character_leafs(std::shared_ptr<base_rule::node> &node)
{
  if (node == nullptr)
    return nullptr;

  if (node->children.size() == 1 && node->the_type == base_rule::node::type::named_rule) {
    node->children.clear();
  }

  for (auto &entry : node->children)
    remove_character_leafs(entry);

  return node;
}

std::shared_ptr<base_rule::node> ast_optimizer::remove_alternations(std::shared_ptr<base_rule::node> &node)
{
  if (node == nullptr)
    return nullptr;

  for (unsigned int i = 0; i < node->children.size(); i++) {
    auto& entry = node->children[i];

    if (entry->children.size() == 1 && ((entry->the_type == base_rule::node::type::alternation) || (entry->the_type == base_rule::node::type::concatenation))) {
      entry = entry->children.front();
      entry->parent = node;
      remove_alternations(node);
    }
  }


  for (auto &entry : node->children)
    remove_alternations(entry);

  return node;
}

std::shared_ptr<base_rule::node> ast_optimizer::remove_one_children_roots(std::shared_ptr<base_rule::node> &node)
{
  if (node == nullptr)
    return nullptr;

  while ((node->children.size() == 1) && (node->children.front()->the_type != base_rule::node::type::value)) {
    auto new_children = node->children.front()->children;
    node->children.clear();
    node->children = new_children;
    for (auto entry : new_children)
      entry->parent = node;
  }

  remove_one_children_roots(node->left_children());
  remove_one_children_roots(node->right_children());

  return node;
}

void ast_optimizer::remove_nodes_marked_for_deletion(std::shared_ptr<base_rule::node> &node)
{
  if (node == nullptr)
    return;

  remove_nodes_marked_for_deletion(node->left_children());
  remove_nodes_marked_for_deletion(node->right_children());

  while (node->parent != nullptr && node->parent->children.size() != 0 && node->parent->left_children().get()->the_type == base_rule::node::type::deleted) {
    node->parent->children.erase(node->parent->children.begin());
  }
}
