#include "Construct.h"

unsigned int ConstructFunction::ID_counter = 0;

string ConstructFunction::getDeclarationString()
{
  return getSignature() + ";";
}

string ConstructFunction::getFunctionString()
{
  string functionString;
  functionString += getSignature();
  functionString += "{ \n";

  for (string& evalEntry : evalFunctions){
    functionString += "_rootNode->evalFunctions.push_back(" + evalEntry + "); \n";
  }

  functionString += "_rootNode->constructChildrenNodeFunc =" + constructChildrenNodeFunc + ";" + "\n";
  functionString += "_rootNode->outputStates.resize(" + to_string(outputStatesSize) + ");" + "\n";
  functionString += "_rootNode->inputStates.resize(" + to_string(inputStatesSize) + ");" + "\n";
  functionString += "return _rootNode;\n}\n";
  return functionString;
}

ConstructFunction::ConstructFunction(
  vector<string> _evalFunctions,
  string _constructChildrenNodeFunc,
  unsigned int _outputStatesSize,
  unsigned int _inputStatesSize
  )
  :evalFunctions(_evalFunctions),
  constructChildrenNodeFunc(_constructChildrenNodeFunc),
  outputStatesSize(_outputStatesSize),
  inputStatesSize(_inputStatesSize)
{
}

string ConstructFunction::getSignature()
{
  return
    "Property* construct_" +
    to_string(ID) + "_" +
    to_string(outputStatesSize) + "_" +
    to_string(inputStatesSize) +
    "(Property* _rootNode)";
}