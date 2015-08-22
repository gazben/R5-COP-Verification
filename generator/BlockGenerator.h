#ifndef BlockGenerator_h__
#define BlockGenerator_h__

#include <string>
#include <vector>
#include <SyntX/util/parser/parser.h>

#include "ast_node.h"

struct block {
  std::vector<std::pair<std::string, ast_node*>> blockRoots;
  unsigned int nextStateInterfaceSize;

  void init() {
  }
};

class BlockGenerator {
private:
  ast_node* rootNode;
  std::vector<block> evalBlocks;

  int blockLevel = 0;

  void cutAST(ast_node* node = nullptr) {
    if (node == nullptr)
      return;

    if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Next") {
      auto root = node->leftChildren->cloneUntilNext();
      evalBlocks[node->blockID - 1].blockRoots.push_back(std::pair<std::string, ast_node*>(ast_node::to_string(root), root));
    
    }

    cutAST(node->leftChildren);
    cutAST(node->rightChildren);

  }
public:

  BlockGenerator(ast_node* root) :rootNode(root) {
  }

  ast_node* RootNode() const { return rootNode; }
  void RootNode(ast_node* val) { rootNode = val; }

  static void markBlocks(ast_node* node) {
    if (node == nullptr)
      return;

    static int currentBlockID = 0;

    if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Next") {
      currentBlockID++;
    }

    node->blockID = currentBlockID;

    markBlocks(node->leftChildren);
    markBlocks(node->rightChildren);

    if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Next") {
      currentBlockID--;
    }
  }

  int getHeight(ast_node* node)
  {
    if (node == nullptr)
      return 0;

    int lheight = getHeight(node->leftChildren);
    int rheight = getHeight(node->rightChildren);

    if (lheight > rheight) {
      if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Next")
        return(lheight + 1);
      else
        return(lheight);
    }

    else {
      if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Next")
        return(rheight + 1);
      else
        return(rheight);
    }

  }

  void createBlocks() {
    evalBlocks.resize(getHeight(rootNode));
    cutAST(rootNode);

    ast_draw<decltype(rootNode)> printer(rootNode);
    printer.to_formatted_string(rootNode);

    getchar();
  }
};

#endif // BlockGenerator_h__
