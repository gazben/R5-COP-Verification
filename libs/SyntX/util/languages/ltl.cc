#include <SyntX/util/languages/ltl.h>


bool ltl::match(base_rule::match_range& context, base_rule::match_range& result, std::shared_ptr<base_rule::node>& root){
    rule ReleaseOp("Release");
    ReleaseOp <<= character("R") | character("W");
    rule UntilOp("Until");
    UntilOp <<= character("U");
    rule ImplicationOp("Implication");
    ImplicationOp <<= identifier("->") | identifier("=>") | identifier("-->") | identifier("imply") | identifier("i");
    rule EquivalenceOp("Equivalence");
    EquivalenceOp <<= identifier("<->") | identifier("<=>") | identifier("iff") | identifier("eq") | identifier("e");
    rule XorOp("Xor");
    XorOp <<= character("^") | identifier("xor");
    rule OrOp("Or");
    OrOp <<= character("|") | identifier("||") | character("+") | identifier("or");
    rule AndOp("And");
    AndOp <<= character("&") | identifier("&&") | character("*") | identifier("and");
    rule WeakNextOp("WeakNext");
    WeakNextOp <<= identifier("wX") | identifier("wO");
    rule NextOp("Next");
    NextOp <<= character("X") | character("O");
    rule FutureOp("Future");
    FutureOp <<= character("F") | identifier("<>");
    rule GloballyOp("Globally");
    GloballyOp <<= character("G") | identifier("[]");
    rule NotOp("Not");
    NotOp <<= character("!") | character("~") | identifier("not");

    rule Operator;
    Operator <<=
            ReleaseOp
            | UntilOp
            | GloballyOp
            | EquivalenceOp
            | XorOp
            | OrOp
            | AndOp
            | WeakNextOp
            | NextOp
            | FutureOp
            | ImplicationOp
            | NotOp
            ;


    rule LPAR;
    LPAR <<= character("(");
    rule RPAR;
    RPAR <<= character(")");
    rule AtomicProposition;
    AtomicProposition <<= range('0', '9');

    rule operatorActions, operationOneOp, operationTwoOp, OperatorEnd, expression;

    operationTwoOp <<= -OperatorEnd << *(-Operator << -OperatorEnd);
    operationOneOp <<= -Operator << *expression;

    OperatorEnd <<= -AtomicProposition | -expression;
    operatorActions <<= -operationOneOp | -operationTwoOp;

    expression <<= -LPAR << -operatorActions << -RPAR;



    return operatorActions.match(context, result, root);
}