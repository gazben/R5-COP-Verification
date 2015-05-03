#ifndef Construct_h__
#define Construct_h__

#include <string>
#include <vector>

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
using namespace std;

class ConstructFunction{
public:
  ConstructFunction(
    vector<string> _evalFunctions,
    string _constructChildrenNodeFunc,
    unsigned int _outputStatesSize,
    unsigned int _inputStatesSize
    );

  /*
  Naming convention: construct_ID_OUT_IN
  */
  string getFunctionString();

  string getDeclarationString();

private:
  vector<string> evalFunctions;
  string constructChildrenNodeFunc;
  unsigned int outputStatesSize;
  unsigned int inputStatesSize;

  static unsigned int ID_counter;
  unsigned int ID;

  string getSignature();
};

#endif // Construct_h__
