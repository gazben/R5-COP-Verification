#ifndef Eval_h__
#define Eval_h__

#include "Function.h"

/*
Evaluate.
Replace the //--EVALFUNCTIONS-- substring with the generated string!

Eval function example:
trilean EVAL_s1a(Property* _prop)
{
return
NAND_3(
NOT_3(_prop->isEventFired(EVENT_B)),
NAND_3(_prop->isEventFired(EVENT_C), _prop->InputStates()[1])
);
}
*/

class EvalFunction : public Function{
public:
  virtual string getFunctionString()
  {
    throw std::logic_error("The method or operation is not implemented.");
  }

  virtual string getDeclarationString()
  {
    throw std::logic_error("The method or operation is not implemented.");
  }

private:
  virtual string getSignature()
  {
    throw std::logic_error("The method or operation is not implemented.");
  }
};

#endif // Eval_h__
