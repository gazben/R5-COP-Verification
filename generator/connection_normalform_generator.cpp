#include "generator.h"

int ConnectionNormalFormGenerator::until_node_deepness = 2;

AstNode* ConnectionNormalFormGenerator::copyAST(std::shared_ptr<base_rule::node> node, AstNode* parent /*= nullptr*/)
{
  AstNode* result = new AstNode(node->the_type, node->the_value);
  result->parent = parent;
  if (node->right_children())
    result->right_children = copyAST(node->right_children(), result);
  if (node->left_children())
    result->left_children = copyAST(node->left_children(), result);

  return result;
}

/*
        exp1
         |
exp1    not
 |       |
 G   =>  F
 |       |
exp2    not
         |
        exp2
*/
void ConnectionNormalFormGenerator::convertGenerallyOperators(AstNode* node)
{
  if (node == nullptr)
  {
    return;
  }

  if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Globally")
  {
    node->the_type = base_rule::node::type::named_rule;
    node->the_value = "Not";

    AstNode* future_node = new AstNode(base_rule::node::type::named_rule, "Future");
    future_node->parent = node;

    AstNode* not_node = new AstNode(base_rule::node::type::named_rule, "Not");
    not_node->parent = future_node;

    not_node->left_children = node->getLeftChildren();
    not_node->right_children = node->getRightChildren();

    node->nullChildren();
    node->left_children = future_node;
    future_node->left_children = not_node;
  }

  convertGenerallyOperators(node->getLeftChildren());
  convertGenerallyOperators(node->getRightChildren());
}

/*
exp1      exp1
 |         |
 F   =>    U
 |       /   \
exp2   true  exp2
*/
void ConnectionNormalFormGenerator::convertFutureOperators(AstNode* node)
{
  if (node == nullptr) {
    return;
  }

  if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Future") {
    node->the_type = base_rule::node::type::named_rule;
    node->the_value = "Until";

    AstNode* true_node = new AstNode(base_rule::node::type::value, "True");
    true_node->parent = node;

    node->right_children = node->getLeftChildren();
    node->left_children = true_node;
  }

  convertFutureOperators(node->getLeftChildren());
  convertFutureOperators(node->getRightChildren());
}

/*
               exp
   exp          |
    |           Or
   Impl    =>  /  \
   /   \      not  exp2
 exp1  exp2    |
              exp1
*/
void ConnectionNormalFormGenerator::convertImplicationOperators(AstNode* node)
{
  if (node == nullptr) {
    return;
  }

  if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Implication") {
    node->the_type = base_rule::node::type::named_rule;
    node->the_value = "Or";

    AstNode* not_node = new AstNode(base_rule::node::type::named_rule, "Not");
    not_node->parent = node;
    not_node->addChildren(node->getLeftChildren());

    node->left_children = not_node;
  }

  convertImplicationOperators(node->getLeftChildren());
  convertImplicationOperators(node->getRightChildren());
}

/*
               exp
  exp           |
   |           not
  Impl    =>    |
 /   \         and
exp1  exp2    /   \
            not   not
             |     |
            exp1  exp2
*/
void ConnectionNormalFormGenerator::convertOrOperators(AstNode* node)
{
  if (node == nullptr)
  {
    return;
  }

  if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Or") {
    node->the_type = base_rule::node::type::named_rule;
    node->the_value = "Not";

    AstNode* and_node = new AstNode(base_rule::node::type::named_rule, "And");
    and_node->parent = node;

    AstNode* not_node_left = new AstNode(base_rule::node::type::named_rule, "Not");
    AstNode* not_node_right = new AstNode(base_rule::node::type::named_rule, "Not");
    and_node->left_children = not_node_left;
    and_node->right_children = not_node_right;

    not_node_left->parent = and_node;
    not_node_left->addChildren(node->getLeftChildren());
    not_node_right->parent = and_node;
    not_node_right->addChildren(node->getRightChildren());

    node->nullChildren();
    node->left_children = and_node;
  }

  convertOrOperators(node->getLeftChildren());
  convertOrOperators(node->getRightChildren());
}

/*
                  exp
                   |
                   or
   exp           /    \
    |          exp2   and
    U     =>         /   \
  /   \            exp1  next
exp1  exp2                |
                          U
                        /   \
                      exp1  exp2
*/
void ConnectionNormalFormGenerator::convertUntilOperators(AstNode* node, size_t depth /*= 0*/)
{
  if (node == nullptr) {
    return;
  }

  if (node->the_type == base_rule::node::type::named_rule &&
    node->the_value == "Until"  &&
    node->converted_count < until_node_deepness
    )
  {
    node->the_type = base_rule::node::type::named_rule;
    node->the_value = "Or";

    AstNode* and_node = new AstNode(base_rule::node::type::named_rule, "And");
    AstNode* next_node = new AstNode(base_rule::node::type::named_rule, "Next");
    AstNode* until_node = new AstNode(base_rule::node::type::named_rule, "Until");

    and_node->parent = node;
    next_node->parent = and_node;
    until_node->parent = next_node;

    AstNode* childrenBuffer[2] = { node->getLeftChildren(), node->getRightChildren() };

    node->left_children = childrenBuffer[1]->clone(node);
    node->right_children = and_node;

    and_node->left_children = childrenBuffer[0]->clone(and_node);
    and_node->right_children = next_node;
    next_node->left_children = until_node;
    until_node->left_children = childrenBuffer[0];
    until_node->right_children = childrenBuffer[1];

    until_node->converted_count = node->converted_count + 1;
    node->converted_count = 0;
  }

  convertUntilOperators(node->getLeftChildren(), depth + 1);
  convertUntilOperators(node->getRightChildren(), depth + 1);
}

/*
  exp1
   |
  not     exp1
   |   =>  |
  not     exp2
   |
  exp2
*/
void ConnectionNormalFormGenerator::convertNegateOperators(AstNode* node)
{
  if (node == nullptr) {
    return;
  }

  if (
    (node->the_type == base_rule::node::type::named_rule && node->the_value == "Not") &&
    (node->getLeftChildren()->the_type == base_rule::node::type::named_rule && node->getLeftChildren()->the_value == "Not"))
  {
    node->the_type = node->getLeftChildren()->getLeftChildren()->the_type;
    node->the_value = node->getLeftChildren()->getLeftChildren()->the_value;
    node->getLeftChildren()->getLeftChildren()->parent = node;

    AstNode* childrenTemp = node->getLeftChildren()->getLeftChildren();

    node->left_children = childrenTemp->getLeftChildren();
    node->right_children = childrenTemp->getRightChildren();

    //TODO free up memory of NOT node
  }

  convertNegateOperators(node->getLeftChildren());
  convertNegateOperators(node->getRightChildren());
}

std::shared_ptr<base_rule::node> ConnectionNormalFormGenerator::getOriginalRoot()
{
  return original_root;
}

void ConnectionNormalFormGenerator::setOriginalRoot(std::shared_ptr<base_rule::node> val)
{
  original_root = val;
}

void ConnectionNormalFormGenerator::convertOneMOreUntilLevel(AstNode* root)
{
  until_node_deepness++;
  convertUntilOperators(root);
  convertOrOperators(root);
  convertNegateOperators(root);
}

AstNode* ConnectionNormalFormGenerator::convertToConnectionNormalForm(std::shared_ptr<base_rule::node>& _root)
{
  original_root = _root;
  root_node = copyAST(original_root);

  convertGenerallyOperators(root_node);
  convertFutureOperators(root_node);
  convertImplicationOperators(root_node);
  convertUntilOperators(root_node);
  convertOrOperators(root_node);
  convertNegateOperators(root_node);
  root_node = addNextOpToRoot(root_node);

  return root_node;
}

void ConnectionNormalFormGenerator::freeAst()
{
  AstNode::freeAst(root_node);
}

int ConnectionNormalFormGenerator::getUntilDeepness()
{
  return until_node_deepness;
}

AstNode*  ConnectionNormalFormGenerator::addNextOpToRoot(AstNode* node) {
  if (node == nullptr)
    return nullptr;

  AstNode* root_Next_node = new AstNode(base_rule::node::type::named_rule, "Next");
  root_Next_node->left_children = node;
  root_Next_node->parent = nullptr;
  node->parent = root_Next_node;

  return root_Next_node;
}