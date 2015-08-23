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
  ast_node*, // pointer to the root
  std::vector<std::string> // nextState interface string
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
  unsigned int currentBlockNumber = 0;
  bool isNextBlockIdenticalToPrev(std::vector<std::string> previousState, std::vector<std::string> nextState);
  void cutAST(ast_node* node = nullptr);
  void RootNode(ast_node* val);
  void cutNextBlock(std::vector<ast_node*> blockRoots);
  static void markBlocks(ast_node* node);
  std::vector<std::string> getNextStateInterface(ast_node* node);
  std::vector<std::string> getPreviousStateInterface(int blockNumber);
  std::vector<std::string> getNextStateInterface(int blockNumber);
  ast_node* RootNode() const;
  int getHeight(ast_node* node);

public:

  BlockGenerator(ast_node* root);
  void createBlocks();
};

#endif // BlockGenerator_h__
