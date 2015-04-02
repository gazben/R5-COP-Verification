#ifndef StateRegister_h__
#define StateRegister_h__

/* GLOBAL INCLUDES */
#include <stdlib.h>

/* LOCAL INCLUDES */
#include "Events.h"
/* INCLUDES END */

class StateRegisterState{
private:
	//Global stateRegister
	static SR_regtype stateRegister;
	static StateRegisterState* rootState;

	SR_regtype stateRegisterValue;

	StateRegisterState* rightNode;
	StateRegisterState* leftNode;

	static StateRegisterState* insertState(SR_regtype stateReg = stateRegister, StateRegisterState* root = rootState){
		if (root == nullptr) {
			StateRegisterState *temp = new StateRegisterState();
			temp->leftNode = temp->rightNode = nullptr;
			temp->stateRegisterValue = stateReg;
			return temp;
		}
		else if (stateReg < root->stateRegisterValue){
			root->leftNode = insertState(stateReg, root->leftNode);
			return root->leftNode;
		}
		else if (stateReg > root->stateRegisterValue){
			root->rightNode = insertState(stateReg, root->rightNode);
			return root->rightNode;
		}
		else
			return root;
	}

public:

	StateRegisterState(){
		leftNode = nullptr;
		rightNode = nullptr;
		stateRegisterValue = stateRegister;
	}

	~StateRegisterState(){
	}

	SR_regtype StateRegisterValue() const { 
		return stateRegisterValue;
	}

	static void freeState(StateRegisterState *root = rootState) {
		if (rootState == nullptr)
			return;

		delete root->leftNode;
		delete root->rightNode;
		delete root;
	}

	static StateRegisterState* getStatePointer(SR_regtype StateRegisterCopy = stateRegister){
		if (rootState == nullptr){
			rootState = insertState();
			return rootState;
		}

		StateRegisterState* temp = rootState;

		while (temp != nullptr && temp->stateRegisterValue != StateRegisterCopy) {
			if (StateRegisterCopy < temp->stateRegisterValue){
				if (temp->leftNode == nullptr){
					temp->leftNode = insertState(StateRegisterCopy);
					return temp->leftNode;
				}
				temp = temp->leftNode;
			}
			else{
				if (temp->rightNode == nullptr){
					temp->rightNode = insertState(StateRegisterCopy);
					return temp->rightNode;
				}
				temp = temp->rightNode;
			}
		}
		return temp;
	}

	friend class Eventhandler;
};

#endif // StateRegister_h__
