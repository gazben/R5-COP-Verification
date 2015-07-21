#ifndef Construct_h__
#define Construct_h__

#include "Function.h"

/*
Construct.
Replace the //--CONSTRUCTFUNCTIONS-- substring with the generated string
Add the function to the //--DECLARATIONS--

Just set the input and output sizes, evalfunctions and construct children node function.
Example:
Property* constructS0(Property* _rootNode)
{
_rootNode->evalFunctions.push_back(EVAL_s0);
_rootNode->constructChildrenNodeFunc = constructS1;
_rootNode->outputStates.resize(1);
_rootNode->inputStates.resize(2);
return _rootNode;
}
*/

class ConstructFunction :public Function {
public:
  ConstructFunction(
    std::vector<std::string> _evalFunctions,
    std::string _constructChildrenNodeFunc,
    unsigned int _outputStatesSize,
    unsigned int _inputStatesSize
    );

  /*
  Naming convention: construct_ID_OUT_IN
  */
  std::string getFunctionString();

  std::string getDeclarationString();

private:
  std::vector<std::string> evalFunctions;
  std::string constructChildrenNodeFunc;
  unsigned int outputStatesSize;
  unsigned int inputStatesSize;

  std::string getSignature();
};

#endif // Construct_h__
