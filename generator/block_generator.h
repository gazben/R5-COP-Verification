#ifndef BlockGenerator_h__
#define BlockGenerator_h__

#include <string>
#include <vector>
#include <tuple>
#include <boost/log/trivial.hpp>

#include "syntx/parser.h"
#include "connection_normalform_generator.h"

// "Top" interface of a block
using PreviousStateInterface = std::vector<
  std::tuple
  <
  std::string, // previousStateInterface
  AstNode*, // pointer to the root
  std::vector<std::string> // nextState interface string
  >
>;
// "Buttom" interface of a block
using NextStateInterface = std::vector<AstNode*>;

//Represents a block from the AST (from next node to next node)
struct block {
  int block_id;
  PreviousStateInterface block_roots;
  NextStateInterface next_state_roots;

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
class BlockGenerator
{
public:
  std::vector<block> eval_blocks;

  //Returns true if the two given interfaces are equal
  static bool isNextBlockIdenticalToPrev(std::vector<std::string> previousState, std::vector<std::string> nextState);

  std::vector<std::string> getPreviousStateInterface(int blockNumber);
  std::vector<std::string> getNextStateInterface(int blockNumber);

  //Create the evaluation blocks from the AST root node. Fill the evalBlocks vector.
  void createBlocks();

  //Code generation functions
  std::string getFunctionDeclarations();
  std::string getFunctions();
  std::string getConstructFunctions();
  std::string getEventsWithCodes();

  AstNode* getAstRootNode() const;

  void setAstRootNode(AstNode* rootNode);
private:
  ConnectionNormalFormGenerator generator;
  AstNode* ast_root_rode;
  int getHeight(AstNode* node);
  void fillEventBlocks(AstNode* root);
  std::vector<std::string> event_names;


  std::vector<std::string> nextStateInterfaceBuffer;
  std::vector<AstNode*> nextStateRootBuffer;
  std::vector<std::string> getNextStateInterface(AstNode* node);

  void cutAST(AstNode* node = nullptr);
  void cutNextBlock(std::vector<AstNode*> blockRoots);
  //Mark the given AST with the proper blockID
  void markBlocks(AstNode* node);
};

#endif // BlockGenerator_h__
