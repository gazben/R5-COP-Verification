#ifndef BlockGenerator_h__
#define BlockGenerator_h__

#include <string>
#include <vector>
#include <tuple>
#include <SyntX/util/parser/parser.h>

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
  int blockID;

  std::string getConstructDeclaration();
  std::string getConstructString();
  std::vector<std::string> getSignatures();
  std::string getDeclarationString();
  std::string getFunctionString();

  std::vector<std::string> getNextStateInterface() {
    std::vector<std::string> result;
    for (auto entry : nextStateRoots) {
      result.push_back(ast_node::to_string(entry->leftChildren));
    }
    return result;
  }

  std::vector<std::string> getPreviousStateInterface()
  {
    std::vector<std::string> result;
    for (auto& entry : blockRoots) {
      result.push_back(std::get<0>(entry));
    }
    return result;
  }
};

class BlockGenerator {
private:
  ast_node* rootNode;

  ConnectionNormalFormGenerator generator;
  std::vector<std::string> nextStateInterfaceBuffer;
  std::vector<ast_node*> nextStateRootBuffer;
  unsigned int currentBlockNumber = 0;
  void cutAST(ast_node* node = nullptr);
  void RootNode(ast_node* val);
  void cutNextBlock(std::vector<ast_node*> blockRoots);
  static void markBlocks(ast_node* node);
  std::vector<std::string> getNextStateInterface(ast_node* node);

  ast_node* RootNode() const;
  int getHeight(ast_node* node);

public:
  std::vector<block> evalBlocks;
  static bool isNextBlockIdenticalToPrev(std::vector<std::string> previousState, std::vector<std::string> nextState);

  std::vector<std::string> getPreviousStateInterface(int blockNumber);
  std::vector<std::string> getNextStateInterface(int blockNumber);

  BlockGenerator(ast_node* root);
  void createBlocks();

  std::string getFunctionDeclarations();
  std::string getFunctionStrings();
  std::string getConstructFunctionStrings();
};

#endif // BlockGenerator_h__
