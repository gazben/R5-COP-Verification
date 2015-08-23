#ifndef BlockGenerator_h__
#define BlockGenerator_h__

#include <string>
#include <vector>
#include <tuple>
#include <SyntX/util/parser/parser.h>

#include "ast_node.h"
#include "ConnectionNormalFormGenerator.h"

using block_data = std::vector<
  std::tuple<
  std::string, // previousStateInterface
  ast_node*, // pointer to the block
  std::vector<std::string> // nextState interface
  >
>;

struct block {
  block_data blockRoots;
  std::vector<ast_node*> nextStateRoots;
};

class BlockGenerator {
private:
  ast_node* rootNode;
  std::vector<block> evalBlocks;
  ConnectionNormalFormGenerator generator;
  std::vector<std::string> nextStateInterfaceBuffer;
  std::vector<ast_node*> nextStateRootBuffer;

  void cutAST(ast_node* node = nullptr) {
    if (node == nullptr)
      return;

    if ((node->the_type == base_rule::node::type::named_rule && node->the_value == "Next")
      && node->blockID <= (generator.getUntilDeepness() - 1)
      )
    {
      auto root = node->leftChildren->cloneUntilNext();
      if (evalBlocks.size() < node->blockID)
        evalBlocks.resize(node->blockID);

      evalBlocks[node->blockID - 1].blockRoots.push_back(std::tuple< std::string, ast_node*, std::vector<std::string> >(ast_node::to_string(root), root,
        [&]()->auto {
        nextStateInterfaceBuffer.clear();
        return getNextStateInterface(node->leftChildren);
      }()
        ));
    }
    else if ((node->the_type == base_rule::node::type::named_rule && node->the_value == "Next")
      && (node->blockID == generator.getUntilDeepness())) {
      evalBlocks[node->parent->blockID - 1].nextStateRoots.push_back(node);
    }

    cutAST(node->leftChildren);
    cutAST(node->rightChildren);
  }

  void cutNextBlock(std::vector<ast_node*> blockRoots) {
    for (int rootIndex = 0; rootIndex < blockRoots.size(); rootIndex++) {
    }
  }

  std::vector<std::string> getNextStateInterface(ast_node* node) {
    if (node == nullptr)
      return std::vector<std::string>();

    if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Next") {
      nextStateInterfaceBuffer.push_back(ast_node::to_string(node->leftChildren));
    }
    else {
      getNextStateInterface(node->leftChildren);
      getNextStateInterface(node->rightChildren);
    }

    return nextStateInterfaceBuffer;
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
    cutAST(rootNode);

    ast_draw<decltype(rootNode)> printer(rootNode);
    printer.to_formatted_string(rootNode);

    getchar();
  }
};

#endif // BlockGenerator_h__
