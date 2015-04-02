#ifndef Property_h__
#define Property_h__

/* GLOBAL INCLUDES */
#include <functional>
#include <vector>
#include <exception>

/* LOCAL INCLUDES */
#include "EventHandler.h"
/* INCLUDES END */

/* FUNCTION TYPE DEFINITIONS */
using propertyEvalFunc = std::function < trilean(class Property*(void)) >;
using constructChildrenBlock = std::function < class Property*(class Property*) >;

class Property{
protected:
	Property* rootNode;
	Property* childrenNode;

	StateRegisterState* stateRegisterPtr;

	std::vector<trilean> inputStates;
	std::vector<trilean> outputStates;
	std::vector <propertyEvalFunc*> evalFunctions;
	constructChildrenBlock constructChildrenNode;

public:

	Property(unsigned int _inputSize, unsigned int _outputSize)
		:childrenNode(nullptr),
		rootNode(nullptr),
		stateRegisterPtr(nullptr),
		constructChildrenNode(nullptr)
	{
		evalFunctions.resize(_outputSize);
		inputStates.resize(_inputSize);

		stateRegisterPtr = StateRegisterState::getStatePointer();
	}

	~Property(){
		delete childrenNode;

		if (rootNode != nullptr)
			rootNode->childrenNode = nullptr;
	}

	std::vector<propertyEvalFunc> EvalFunctions() const {
		return evalFunctions;
	}

	void EvalFunctions(std::vector<propertyEvalFunc> func) {
		if (func.size() != evalFunctions.size()){
			throw std::runtime_error("Invalid eval function size!");
		}
		evalFunctions = func;
	}

	void freeChildrenNode(){
		delete childrenNode;

		if (rootNode != nullptr)
			rootNode->childrenNode = nullptr;
	}

	std::vector<trilean> OutputStates() const {
		return outputStates;
	}

	std::vector<trilean> InputStates() const {
		return inputStates;
	}

	//Getter-setters
	Property* RootNode() const {
		return rootNode;
	}
	void RootNode(class Property* val) { rootNode = val; }

	Property* ChildrenNode() const { return childrenNode; }
	void ChildrenNode(Property* val) { childrenNode = val; }

	trilean isEventFired(SR_regtype eventCode){
		return (stateRegisterPtr->StateRegisterValue() & eventCode) ? FALSE : TRUE;
	}

	static trilean Evaluate(Property* root){
		Property* currentNode = root;
		trilean result = UNKNOWN;

		bool isChanged = 0;

		while (result == UNKNOWN){
			isChanged = false;

			for (int i = 0; i < currentNode->OutputStates().size(); i++){
				trilean tempOutputResult = currentNode->evalFunctions[i](currentNode);
				if (tempOutputResult != currentNode->outputStates[i]){
					isChanged = true;
					currentNode->outputStates[i] = tempOutputResult;
					break;
				}
			}

			//Output of the descendant node changed. We can go up in the stack.
			if (isChanged){
				//Free the current node.
				if (currentNode->rootNode != nullptr){
					currentNode = currentNode->rootNode;

					//give the output to the input
					if (currentNode->inputStates.size() != currentNode->childrenNode->outputStates.size()){
						throw std::runtime_error("Invalid eval function size!");
					}

					//COPY right now, optimise later!
					for (int i = 0; i < currentNode->inputStates.size(); i++){
						currentNode->inputStates[i] = currentNode->childrenNode->outputStates[i];
					}

					delete currentNode->childrenNode;
				}
				else{
					//GOAL REACHED
					result = currentNode->outputStates[0];
					root->freeChildrenNode();
				}
			}
			//No change happened we go deeper
			else{
				currentNode->constructChildrenNode(currentNode);
				currentNode = currentNode->childrenNode;
			}
		}

		return result;
	}
};

//////////////////////////////////////////////////////////////////////////
namespace EvalFunctions{
	trilean EVAL_s0(Property& _prop){
		return
			AND_3(
			NAND_3(
			_prop.isEventFired(EVENT_A),
			AND_3(
			NOT_3(_prop.isEventFired(EVENT_B)),
			NAND_3(
			_prop.isEventFired(EVENT_C),
			_prop.InputStates()[0])
			)
			),
			NAND_3(TRUE, _prop.InputStates()[1])
			);
	}

	trilean EVAL_s1a(Property& _prop){
		return
			NAND_3(
			NOT_3(_prop.isEventFired(EVENT_B)),
			NAND_3(_prop.isEventFired(EVENT_C), _prop.InputStates()[1])
			);
	}
}

//////////////////////////////////////////////////////////////////////////
namespace ConstructFunctions{
	Property* PROP_constructS0(Property* _rootNode){
		Property* newProppertyNode;

		newProppertyNode = PROP_createEmptyPropertyNode();
		newProppertyNode = PROP_initProperty(newProppertyNode, 2, 1);

		if (_rootNode != NULL){
			newProppertyNode->rootNode = _rootNode;
			_rootNode->descendantNode = newProppertyNode;
		}

		newProppertyNode->evalFunctions[0] = EVAL_s0;
		newProppertyNode->constructDescendantNode = PROP_constructS1;

		return newProppertyNode;
	}

	Property* PROP_constructS1(Property* _rootNode){
		Property* newProppertyNode;

		newProppertyNode = PROP_createEmptyPropertyNode();
		newProppertyNode = PROP_initProperty(newProppertyNode, 2, 2);

		if (_rootNode != NULL){
			newProppertyNode->rootNode = _rootNode;
			_rootNode->descendantNode = newProppertyNode;
		}

		newProppertyNode->evalFunctions[0] = EVAL_s1a;
		newProppertyNode->evalFunctions[1] = EVAL_s0;
		newProppertyNode->constructDescendantNode = PROP_constructS1;

		return newProppertyNode;
	}
}

#endif // Property_h__
