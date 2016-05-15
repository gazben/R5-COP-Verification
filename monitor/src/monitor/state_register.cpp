#include "state_register.h"

/*Static init*/
StateRegisterType StateRegister::state_register = 0;
StateRegister *StateRegister::root_state = nullptr;

StateRegister *StateRegister::getStatePointer(StateRegisterType state_register_copy /*= stateRegister*/)
{
  if (root_state == nullptr){
    root_state = insertState();
    return root_state;
  }

  StateRegister * temp = root_state;

  while (temp != nullptr && temp->state_register_value != state_register_copy) {
    if (state_register_copy < temp->state_register_value){
      if (temp->left_node == nullptr){
        temp->left_node = insertState(state_register_copy);
        return temp->left_node;
      }
      temp = temp->left_node;
    }
    else{
      if (temp->right_node == nullptr){
        temp->right_node = insertState(state_register_copy);
        return temp->right_node;
      }
      temp = temp->right_node;
    }
  }
  return temp;
}

void StateRegister::freeState(StateRegister *root /*= rootState*/)
{
  if (root_state == nullptr)
    return;

  delete root->left_node;
  delete root->right_node;
  delete root;
}

StateRegister::~StateRegister()
{
}

StateRegister::StateRegister()
{
  left_node = nullptr;
  right_node = nullptr;
  state_register_value = state_register;
}

StateRegister *StateRegister::insertState(StateRegisterType stateReg /*= stateRegister*/, StateRegister * root /*= rootState*/)
{
  if (root == nullptr) {
    StateRegister *temp = new StateRegister();
    temp->left_node = temp->right_node = nullptr;
    temp->state_register_value = stateReg;
    return temp;
  }
  else if (stateReg < root->state_register_value){
    root->left_node = insertState(stateReg, root->left_node);
    return root->left_node;
  }
  else if (stateReg > root->state_register_value){
    root->right_node = insertState(stateReg, root->right_node);
    return root->right_node;
  }
  else
    return root;
}

void StateRegister::clearEvents() {
  state_register = 0;
}

bool StateRegister::isEventCurrentlyFired(StateRegisterType eventCode)
{
  return (state_register & eventCode) ? true : false;
}
