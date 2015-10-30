#ifndef BlockGenerator_h__
#define BlockGenerator_h__

#include <string>
#include <vector>
#include <tuple>
#include <SyntX/util/parser/parser.h>
#include <boost/log/trivial.hpp>

#include "ConnectionNormalFormGenerator.h"

// "Top" interface of a block
using previousStateInterface = std::vector<
  std::tuple
  <
  std::string, // previousStateInterface
  ast_node*, // pointer to the root
  std::vector<std::string> // nextState interface string
  >
>;
// "Buttom" interface of a block
using nextStateInterface = std::vector<ast_node*>;

//Represents a block from the AST (from next node to next node)
struct block {
  int blockID;
  previousStateInterface blockRoots;
  nextStateInterface nextStateRoots;
  
  //Returns the generated construct function declaration.
  std::string getConstructDeclaration();
  //Returns the generated construct function body.
  std::string getConstructBody();
  //Returns the function declarations for the block roots. 
  std::vector<std::string> getBlockRootEvalFunctionDeclarations();
  
  //Returns the function declarations of the block.
  std::string getDeclarationString();
  //Returns the function definitions of the block.
  std::string getFunctionString();

  std::vector<std::string> getNextStateInterfaceString();
  std::vector<std::string> getPreviousStateInterfaceString();
};


//Creates the next node to next node blocks from the given AST
class BlockGenerator {
public:
  BlockGenerator(ast_node* root);
  std::vector<block> evalBlocks;

  //Returns true if the two given interfaces are equal
  static bool isNextBlockIdenticalToPrev(std::vector<std::string> previousState, std::vector<std::string> nextState);

  std::vector<std::string> getPreviousStateInterface(int blockNumber);
  std::vector<std::string> getNextStateInterface(int blockNumber);

  //Create the evaluation blocks from the AST root node. Fill the evalBlocks vector.
  void createBlocks();

  //CODE generation functions
  std::string getFunctionDeclarations();
  std::string getFunctions();
  std::string getConstructFunctions();

private:
  ConnectionNormalFormGenerator generator;
  
  ast_node* astRootNode;
  ast_node* getAstRootNode() const;
  void setAstRootNode(ast_node* val);

  int getHeight(ast_node* node);

  std::vector<std::string> nextStateInterfaceBuffer;
  std::vector<ast_node*> nextStateRootBuffer;
  std::vector<std::string> getNextStateInterface(ast_node* node);
  
  void cutAST(ast_node* node = nullptr);
  void cutNextBlock(std::vector<ast_node*> blockRoots);
  //Mark the given AST with the proper blockID
  void markBlocks(ast_node* node);
};

#endif // BlockGenerator_h__
