#ifndef BlockGenerator_h__
#define BlockGenerator_h__

#include <string>
#include <vector>
#include <SyntX/util/parser/parser.h>

#include "ast_node.h"

struct block {
  unsigned int blockID;
  std::vector<ast_node*> blockRoots;
  unsigned int nextStateInterfaceSize;

  block(unsigned int _blockID):blockID(_blockID){
  }

};

class BlockGenerator {
private:
  ast_node* rootNode;
  std::vector<block> evalBlocks;

public:

  BlockGenerator(ast_node* root):rootNode(root) {
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

  std::string generateFunctions() {
  }
};

#endif // BlockGenerator_h__
