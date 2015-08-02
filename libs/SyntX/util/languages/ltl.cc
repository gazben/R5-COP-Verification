#include <SyntX/util/languages/ltl.h>


bool ltl::match(base_rule::match_range& context, base_rule::match_range& result, std::shared_ptr<base_rule::node>& root){
    rule ReleaseOp;
    ReleaseOp <<= character("R") | character("W");
    rule UntilOp;
    UntilOp <<= character("U");
    rule ImplicationOp;
    ImplicationOp <<= identifier("->") | identifier("=>") | identifier("-->") | identifier("imply") | identifier("i");
    rule EquivalenceOp;
    EquivalenceOp <<= identifier("<->") | identifier("<=>") | identifier("iff") | identifier("eq") | identifier("e");
    rule XorOp;
    XorOp <<= character("^") | identifier("xor");
    rule OrOp;
    OrOp <<= character("|") | identifier("||") | character("+") | identifier("or");
    rule AndOp;
    AndOp <<= character("&") | identifier("&&") | character("*") | identifier("and");
    rule WeakNextOp;
    WeakNextOp <<= identifier("wX") | identifier("wO");
    rule NextOp;
    NextOp <<= character("X") | character("O");
    rule FutureOp;
    FutureOp <<= character("F") | identifier("<>");
    rule GloballyOp;
    GloballyOp <<= character("G") | identifier("[]");
    rule NotOp;
    NotOp <<= character("!") | character("~") | identifier("not");

    rule Operator;
    Operator <<=
            ReleaseOp
            | UntilOp
            | ImplicationOp
            | EquivalenceOp
            | XorOp
            | OrOp
            | AndOp
            | WeakNextOp
            | NextOp
            | FutureOp
            | GloballyOp
            | NotOp
            ;


    rule LPAR;
    LPAR <<= character("(");
    rule RPAR;
    RPAR <<= character(")");
    rule AtomicProposition;
    AtomicProposition <<= range('0', '9');

    rule operation, OperatorEnd("OperatorEnd"), expression;
    operation <<= -OperatorEnd << *(-Operator << -OperatorEnd);
    OperatorEnd <<= -AtomicProposition | -expression;
    expression <<= -LPAR << -operation << -RPAR;

    return operation.match(context, result, root);
}