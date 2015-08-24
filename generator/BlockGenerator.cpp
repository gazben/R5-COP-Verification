#include "BlockGenerator.h"

void BlockGenerator::RootNode(ast_node* val)
{
  rootNode = val;
}

ast_node* BlockGenerator::RootNode() const
{
  return rootNode;
}

void BlockGenerator::markBlocks(ast_node* node)
{
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

int BlockGenerator::getHeight(ast_node* node)
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

void BlockGenerator::cutNextBlock(std::vector<ast_node*> blockRoots)
{
  for (auto rootEntry : blockRoots) {
    cutAST(rootEntry);
  }
}

std::vector<std::string> BlockGenerator::getPreviousStateInterface(int blockNumber)
{
  std::vector<std::string> result;
  for (auto& entry : evalBlocks[blockNumber - 1].blockRoots) {
    result.push_back(std::get<0>(entry));
  }
  return result;
}

void BlockGenerator::cutAST(ast_node* node /*= nullptr*/)
{
  if (node == nullptr)
    return;

  if ((node->the_type == base_rule::node::type::named_rule && node->the_value == "Next")
    && node->blockID <= (generator.getUntilDeepness() - 1)
    )
  {
    ast_node::globalInterfaceID = 0;
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

std::vector<std::string> BlockGenerator::getNextStateInterface(int blockNumber)
{
  std::vector<std::string> result;
  for (auto entry : evalBlocks[blockNumber - 1].nextStateRoots) {
    result.push_back(ast_node::to_string(entry->leftChildren));
  }
  return result;
}

std::vector<std::string> BlockGenerator::getNextStateInterface(ast_node* node)
{
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

bool BlockGenerator::isNextBlockIdenticalToPrev(std::vector<std::string> previousState, std::vector<std::string> nextState)
{
  for (const auto& prevEntry : previousState) {  //blockRoot to_string()
    bool result = false;
    for (const auto& nextEntry : nextState) {  //nextStateRoots to_string
      if (prevEntry == nextEntry)
        result = true;
    }
    if (result == false)
      return false;
  }

  return true;
}

BlockGenerator::BlockGenerator(ast_node* root) :rootNode(root)
{
}

void BlockGenerator::createBlocks()
{
  std::vector<ast_node*> rootTemp;
  rootTemp.push_back(rootNode);
  markBlocks(rootNode);
  cutNextBlock(rootTemp);
  currentBlockNumber++;

  while (!isNextBlockIdenticalToPrev(getPreviousStateInterface(currentBlockNumber), getNextStateInterface(currentBlockNumber))) {
    generator.convertOneMOreUntilLevel(rootNode);
    markBlocks(rootNode);
    cutNextBlock(evalBlocks[currentBlockNumber - 1].nextStateRoots);
    currentBlockNumber++;
  }

  for (int i = 0; i < evalBlocks.size(); i++)
    evalBlocks[i].blockID = i;

  ast_draw<decltype(rootNode)> printer(rootNode);
  printer.to_formatted_string(rootNode);
}

std::string BlockGenerator::getFunctionDeclarations()
{
  std::string result;

  for (auto& blockEntry : evalBlocks)
    result += (blockEntry.getDeclarationString() + "\n");

  return result;
}

std::string BlockGenerator::getFunctionStrings()
{
  std::string result;

  for (auto& blockEntry : evalBlocks)
    result += (blockEntry.getFunctionString() + "\n");

  return result;
}

std::string BlockGenerator::getConstructFunctionStrings()
{
  std::string result;

  for (auto& blockEntry : evalBlocks) {
    result += (blockEntry.getConstructString() + "\n");
  }
  return result;
}

std::string block::getConstructString()
{
  std::string constructBlockString;

  //constructBlockString += getSignature();
  constructBlockString += "Property* construct_block" + std::to_string(blockID) + "(Property* _rootNode)";
  constructBlockString += "{ \n";

  std::vector<std::string> evalFunctions;
  for (auto& blockEntry : blockRoots) {
    evalFunctions.push_back("EVAL_" + std::get<0>(blockEntry));
  }

  for (auto& evalEntry : evalFunctions) {
    constructBlockString += "_rootNode->evalFunctions.push_back(" + evalEntry + "); \n";
  }

  constructBlockString += "_rootNode->constructChildrenNodeFunc = construct_block" +
    ((BlockGenerator::isNextBlockIdenticalToPrev(getPreviousStateInterface(), getNextStateInterface())) ? (std::to_string(blockID)) : (std::to_string(blockID + 1))) +
    ";\n";
  constructBlockString += "_rootNode->outputStates.resize(" + std::to_string(blockRoots.size()) + ");" + "\n";
  constructBlockString += "_rootNode->inputStates.resize(" + std::to_string(nextStateRoots.size()) + ");" + "\n";
  constructBlockString += "return _rootNode;\n}\n";

  return constructBlockString;
}

std::vector<std::string> block::getSignatures()
{
  //return "trilean EVAL_s1a(Property* _prop)";    std::string result;
  std::vector<std::string> result;

  for (auto& blockEntry : blockRoots) {
    result.push_back("trilean EVAL_" + std::get<0>(blockEntry) + "(Property* _prop)");
  }
  return result;
}

std::string block::getDeclarationString()
{
  std::string result;

  for (auto& resultEntry : getSignatures()) {
    result += (resultEntry + ";\n");
  }
  return result;
}

std::string block::getFunctionString()
{
  std::vector<std::string> signatures = getSignatures();
  std::vector<std::string> functionBodys;
  std::vector<std::string> results;

  for (auto& blockEntry : blockRoots) {
    functionBodys.push_back(std::get<1>(blockEntry)->getFunctionString());
  }

  for (int i = 0; i < blockRoots.size(); i++) {
    results.push_back(signatures[i] + "{ return " + functionBodys[i] + ";}");
  }

  std::string result;
  for (auto& entry : results) {
    result += (entry + "\n");
  }

  return result;
}