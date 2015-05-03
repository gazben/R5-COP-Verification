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

class EvalFunction : Function{


};

#endif // Eval_h__
