#include <SyntX/util/languages/ltl.h>

bool ltl::match(base_rule::match_range& context, base_rule::match_range& result, std::shared_ptr<base_rule::node>& root) {
  rule ReleaseOp("Release");
  ReleaseOp <<= character("R") | character("W");
  rule UntilOp("Until");
  UntilOp <<= character("U");
  rule ImplicationOp("Implication");
  ImplicationOp <<= identifier("=>") | identifier("imply");
  rule EquivalenceOp("Equivalence");
  EquivalenceOp <<= identifier("<->") | identifier("eq") | identifier("e");
  rule XorOp("Xor");
  XorOp <<= character("^") | identifier("xor");
  rule OrOp("Or");
  OrOp <<= character("|") | identifier("||") | character("+") | identifier("or");
  rule AndOp("And");
  AndOp <<= character("&") | identifier("&&") | character("*") | identifier("and");
  rule NextOp("Next");
  NextOp <<= character("X");
  rule FutureOp("Future");
  FutureOp <<= character("F");
  rule GloballyOp("Globally");
  GloballyOp <<= character("G");
  rule NotOp("Not");
  NotOp <<= character("!") | character("~");
  rule LPAR;
  LPAR <<= character("(");
  rule RPAR;
  RPAR <<= character(")");
  rule AtomicProposition;
  AtomicProposition <<= range('0', '9');

  rule OperatorTwoOp;
  OperatorTwoOp <<=
    ReleaseOp
    | UntilOp
    | ImplicationOp
    | EquivalenceOp
    | XorOp
    | OrOp
    | AndOp
    ;
  
  rule OperatorOneOp;
  OperatorOneOp <<=
    FutureOp
    | GloballyOp
    | NotOp
    | NextOp
    ;

  rule operatorActions, operationOneOp, operationTwoOp, OperatorEnd, expression;

  operationTwoOp <<= -OperatorEnd << -OperatorTwoOp << -OperatorEnd;
  operationOneOp <<= -OperatorOneOp << -OperatorEnd;

  expression <<= -LPAR << -operatorActions << -RPAR;
  OperatorEnd <<= -AtomicProposition | -expression;
  operatorActions <<= *(-operationTwoOp | -operationOneOp);

  return operatorActions.match(context, result, root);
}