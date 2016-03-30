#include "block_generator.h"

void BlockGenerator::setAstRootNode(AstNode* rootNode)
{
  ast_root_rode = rootNode;
}

AstNode* BlockGenerator::getAstRootNode() const
{
  return ast_root_rode;
}

void BlockGenerator::markBlocks(AstNode* node)
{
  if (node == nullptr)
    return;

  static int currentBlockID = 0;

  if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Next")
  {
    currentBlockID++;
  }

  node->block_id = currentBlockID;
  markBlocks(node->left_children);
  markBlocks(node->right_children);

  if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Next")
  {
    currentBlockID--;
  }
}

int BlockGenerator::getHeight(AstNode* node)
{
  if (node == nullptr)
    return 0;

  int lheight = getHeight(node->left_children);
  int rheight = getHeight(node->right_children);
  int maxHeight = (lheight > rheight) ? lheight : rheight;

  if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Next")
    return(maxHeight + 1);
  else
    return(maxHeight);
}

std::string BlockGenerator::getEventsWithCodes()
{
  fillEventBlocks(ast_root_rode);
  std::sort(event_names.begin(), event_names.end());
  auto last = std::unique(event_names.begin(), event_names.end());
  event_names.erase(last, event_names.end());

  std::string result;
  unsigned long long int actual_code = 2; //1 is reserved for stop

  int max_event_count = 62;
  if (event_names.size() > max_event_count) {
    BOOST_LOG_TRIVIAL(fatal) << "Maximum event count reached (max: " 
      + std::to_string(max_event_count) + " current: " + std::to_string(event_names.size()) + ")";
    monitor_generator::terminate();
  } else if (!event_names.empty()) {
    for (auto event_entry : event_names) {
      BOOST_LOG_TRIVIAL(info) << "event \"" + event_entry + "\" entry generated with value " + std::to_string(actual_code);
      result += "const StateRegisterType " + event_entry + " = " + std::to_string(actual_code) + ";\n";
      actual_code = actual_code << 1;
    }
  } else {
    BOOST_LOG_TRIVIAL(warning) << "Event container is empty. No event codes are generated";
  }

  return result;
}

void BlockGenerator::fillEventBlocks(AstNode* root)
{
  if (root == nullptr) 
    return;   

  if (root->getRightChildren() == nullptr && root->getLeftChildren() == nullptr) {
    if (root->the_value != "True" && root->the_value != "False") {
      //remove the leading and the trailing ' characters. Ex.: 'event1' => event1
      auto& event_name = root->the_value.erase(0,1);    
      event_name.erase(event_name.size() - 1, 1);
      //save the event name
      event_names.push_back(event_name);
    }
    return;
  }

  fillEventBlocks(root->getLeftChildren());
  fillEventBlocks(root->getRightChildren());  
  
  return;
}

void BlockGenerator::cutNextBlock(std::vector<AstNode*> blockRoots)
{
  for (auto rootEntry : blockRoots)
  {
    cutAST(rootEntry);
  }
}

std::vector<std::string> BlockGenerator::getPreviousStateInterface(int blockNumber)
{
  return eval_blocks[blockNumber - 1].getPreviousStateInterfaceString();
}

std::vector<std::string> BlockGenerator::getNextStateInterface(int blockNumber)
{
  return eval_blocks[blockNumber - 1].getNextStateInterfaceString();
}

void BlockGenerator::cutAST(AstNode* node /*= nullptr*/)
{
  if (node == nullptr)
    return;

  if ((node->the_type == base_rule::node::type::named_rule && node->the_value == "Next")
    && (node->block_id <= (generator.getUntilDeepness() - 1))
    )
  {
    AstNode::global_interface_id = 0;
    auto root = node->left_children->cloneUntilNext();
    if (eval_blocks.size() < node->block_id)
      eval_blocks.resize(node->block_id);

    eval_blocks[node->block_id - 1].block_roots.push_back(std::tuple< std::string, AstNode*, std::vector<std::string> >(AstNode::toString(root), root,
      [&]()->auto
    {
      nextStateInterfaceBuffer.clear();
      return getNextStateInterface(node->left_children);
    }()
      ));
  }
  else if ((node->the_type == base_rule::node::type::named_rule && node->the_value == "Next")
    && (node->block_id == generator.getUntilDeepness()))
  {
    eval_blocks[node->parent->block_id - 1].next_state_roots.push_back(node);
  }

  cutAST(node->left_children);
  cutAST(node->right_children);
}

std::vector<std::string> BlockGenerator::getNextStateInterface(AstNode* node)
{
  if (node == nullptr)
    return std::vector<std::string>();

  if (node->the_type == base_rule::node::type::named_rule && node->the_value == "Next")
  {
    nextStateInterfaceBuffer.push_back(AstNode::toString(node->left_children));
  }
  else
  {
    getNextStateInterface(node->left_children);
    getNextStateInterface(node->right_children);
  }

  return nextStateInterfaceBuffer;
}

bool BlockGenerator::isNextBlockIdenticalToPrev(std::vector<std::string> previousState, std::vector<std::string> nextState)
{
  for (const auto& prevEntry : previousState)
  {
    bool result = false;
    for (const auto& nextEntry : nextState)
    {
      if (prevEntry == nextEntry)
        result = true;
    }
    if (result == false)
      return false;
  }

  return true;
}

void BlockGenerator::createBlocks()
{
  unsigned int currentBlockNumber = 0;

  std::vector<AstNode*> rootTemp;
  rootTemp.push_back(ast_root_rode);
  markBlocks(ast_root_rode);
  cutNextBlock(rootTemp);
  currentBlockNumber++;

  while (!isNextBlockIdenticalToPrev(
    getPreviousStateInterface(currentBlockNumber), getNextStateInterface(currentBlockNumber))
    )
  {
    generator.convertOneMOreUntilLevel(ast_root_rode);
    markBlocks(ast_root_rode);
    cutNextBlock(eval_blocks[currentBlockNumber - 1].next_state_roots);
    currentBlockNumber++;
  }

  for (unsigned int i = 0; i < eval_blocks.size(); i++)
    eval_blocks[i].block_id = i;
}

std::string BlockGenerator::getFunctionDeclarations()
{
  std::string result;

  for (auto& blockEntry : eval_blocks)
    result += (blockEntry.getDeclarationString() + "\n");

  for (auto& blockEntry : eval_blocks)
    result += (blockEntry.getConstructDeclaration() + "\n");

  return result;
}

std::string BlockGenerator::getFunctions()
{
  std::string result;

  for (auto& blockEntry : eval_blocks)
    result += (blockEntry.getFunctionString() + "\n");

  return result;
}

std::string BlockGenerator::getConstructFunctions()
{
  std::string result;

  for (auto& blockEntry : eval_blocks)
  {
    result += (blockEntry.getConstructBody() + "\n");
  }
  return result;
}

std::string block::getConstructDeclaration() {
  return "Property* construct_block" + std::to_string(block_id) + "(Property* _rootNode);";
}

std::string block::getConstructBody()
{
  std::string constructBlockString;

  constructBlockString += "Property* construct_block" + std::to_string(block_id) + "(Property* _rootNode)";
  constructBlockString += "{ \n";

  std::vector<std::string> evalFunctions;
  for (auto& blockEntry : block_roots)
  {
    evalFunctions.push_back("EVAL_" + std::get<0>(blockEntry));
  }

  for (auto& evalEntry : evalFunctions)
  {
    constructBlockString += "_rootNode->evalFunctions.push_back(" + evalEntry + "); \n";
  }

  constructBlockString += "_rootNode->constructChildrenNodeFunc = construct_block" +
    ((BlockGenerator::isNextBlockIdenticalToPrev(getPreviousStateInterfaceString(), getNextStateInterfaceString())) ? (std::to_string(block_id)) : (std::to_string(block_id + 1))) +
    ";\n";
  constructBlockString += "_rootNode->outputStates.resize(" + std::to_string(block_roots.size()) + ");" + "\n";
  constructBlockString += "_rootNode->inputStates.resize(" + std::to_string(next_state_roots.size()) + ");" + "\n";
  constructBlockString += "return _rootNode;\n}\n";

  return constructBlockString;
}

std::vector<std::string> block::getBlockRootEvalFunctionDeclarations()
{
  //return "trilean EVAL_s1a(Property* _prop)";
  std::vector<std::string> result;

  for (auto& blockEntry : block_roots)
  {
    result.push_back("trilean EVAL_" + std::get<0>(blockEntry) + "(Property* _prop)");
  }
  return result;
}

std::string block::getDeclarationString()
{
  std::string result;

  for (auto& resultEntry : getBlockRootEvalFunctionDeclarations())
  {
    result += (resultEntry + ";\n");
  }
  return result;
}

std::string block::getFunctionString()
{
  std::vector<std::string> signatures = getBlockRootEvalFunctionDeclarations();
  std::vector<std::string> functionBodys;
  std::vector<std::string> results;

  for (auto& blockEntry : block_roots)
  {
    functionBodys.push_back(std::get<1>(blockEntry)->getFunctionString());
  }

  for (unsigned int i = 0; i < block_roots.size(); i++)
  {
    results.push_back(signatures[i] + "{ return " + functionBodys[i] + ";}");
  }

  std::string result;
  for (auto& entry : results)
  {
    result += (entry + "\n");
  }

  return result;
}

std::vector<std::string> block::getNextStateInterfaceString()
{
  std::vector<std::string> result;
  for (auto entry : next_state_roots)
  {
    result.push_back(AstNode::toString(entry->left_children));
  }
  return result;
}

std::vector<std::string> block::getPreviousStateInterfaceString()
{
  std::vector<std::string> result;
  for (auto& entry : block_roots)
  {
    result.push_back(std::get<0>(entry));
  }
  return result;
}