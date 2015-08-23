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

  ast_draw<decltype(rootNode)> printer(rootNode);
  printer.to_formatted_string(rootNode);
}