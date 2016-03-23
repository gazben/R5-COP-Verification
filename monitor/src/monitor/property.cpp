#include "property.h"

//Static field init
unsigned int Property::current_max_id = 0;
unsigned int Property::level = 0;
Property* Property::current_block = nullptr;
bool Property::evaluated = false;

trilean Property::isEventFired(StateRegisterType eventCode)
{
  return (state_register_ptr->stateRegisterValue & eventCode) ? TRUE : FALSE;
}

trilean Property::evaluate()
{
  //Initial block
  if (current_block == nullptr)
    current_block = this;
  
  //Get the current state register for uninitialized block
  if (current_block->state_register_ptr == nullptr) {
    current_block->state_register_ptr = StateRegister::getStatePointer();
  }

  //STOP signal handling
  if (current_block->isEventFired(EVENT_END) == TRUE) {
    ROS_INFO_STREAM("--END signal found. All input values are FALSE--");
    current_block = current_block->root_node;
    current_block->freeChildrenNode();

    for (auto& entry : current_block->input_states) {
      entry = trilean(OutputState::FALSE);
    }
    level--;
  }

  //Begin normal evaluation
  ROS_INFO_STREAM("--Block evaluation--");
  ROS_INFO_STREAM("-Current block state-");
  printBlock(current_block);

  trilean result = UNKNOWN;
  bool isChanged = false;
  ROS_INFO_STREAM("-Evaluating-");
  for (unsigned int i = 0; i < current_block->output_states.size(); i++)
  {
    trilean tempOutputResult = current_block->eval_functions[i](current_block);
    if (tempOutputResult != current_block->output_states[i])
    {
      isChanged = true;
      current_block->output_states[i] = tempOutputResult;
    }
  }

  //Output of the descendant node changed. We can go up in the stack.
  if (isChanged) {
    ROS_INFO_STREAM("-Block changed-");

    if (current_block->root_node != nullptr) {
      //Parent node exist. Move up.
      ROS_INFO_STREAM("-Current block state-");
      printBlock(current_block);

      current_block = current_block->root_node;

      if (current_block->input_states.size() != current_block->children_node->output_states.size()) {
        ROS_INFO_STREAM("Invalid input/output size on block: " + std::to_string(current_block->id) + " and " + std::to_string(current_block->children_node->id));
        ROS_INFO_STREAM("The system will use the smaller input. This can result in wrong result!");
      }
      for (unsigned int i = 0;
      i < ((current_block->input_states.size() < current_block->children_node->output_states.size()) ? current_block->input_states.size() : current_block->children_node->output_states.size());
        i++) {
        current_block->input_states[i] = current_block->children_node->output_states[i];
      }
      current_block->freeChildrenNode();

      level--;

      current_block->evaluate();
    }
    else {
      //No parent node -> GOAL REACHED
      ROS_INFO_STREAM("-No parent node. Goal reached!-");
      evaluated = true;
      result = current_block->output_states[0];
      ROS_INFO_STREAM("Result: " + trilean::tostring(result));
      current_block->freeChildrenNode();
      ROS_INFO_STREAM("Executing given command: " + ((result == TRUE) ? true_command : false_command));

      if (result == TRUE) {
        system(true_command.c_str());
      }
      if (result == FALSE) {
        system(false_command.c_str());
      }
    }
  }
  else {
    level++;
    //No change happened we go deeper
    ROS_INFO_STREAM("No change. Going deeper!");

    current_block->constructChildrenBlock();
    current_block = current_block->children_node;
  }

  return result;
}

void Property::freeChildrenNode()
{
  delete current_block->children_node;
  current_block->children_node = nullptr;
}

Property* Property::constructChildrenBlock()
{
  Property* childrenBlockTemp = new Property();
  childrenBlockTemp->root_node = this;
  children_node = construct_children_node_func(childrenBlockTemp);
  return childrenBlockTemp;
}

Property::~Property()
{
  delete children_node;

  if (root_node != nullptr)
    root_node->children_node = nullptr;
}

Property::Property()
  :children_node(nullptr),
  root_node(nullptr),
  state_register_ptr(nullptr),
  construct_children_node_func(nullptr)
{
  id = current_max_id;
  current_max_id++;
  ROS_INFO_STREAM("BLOCK CREATED | ID " + std::to_string(id));
}

void Property::printBlock(Property *block) {
  std::string tempOut;
  for (auto& entry : block->output_states) {
    tempOut += (entry == OutputState::FALSE) ? "F" : (entry == OutputState::TRUE) ? "T" : "U";
    tempOut += " ";
  }
  std::string tempIn;
  ROS_INFO_STREAM("Out: " + tempOut);
  ROS_INFO_STREAM("ID: " + std::to_string(block->id) + " level: " + std::to_string(level)
    + " Statereg: " + std::to_string(block->state_register_ptr->stateRegisterValue));
  for (trilean& entry : block->input_states) {
    tempIn += (entry == OutputState::FALSE) ? "F" : (entry == OutputState::TRUE) ? "T" : "U";
    tempIn += " ";
  }
  ROS_INFO_STREAM("In: " + tempIn);
}
