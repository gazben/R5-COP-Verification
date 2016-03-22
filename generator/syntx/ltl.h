#ifndef LTL_H
#define LTL_H

#include "parser.h"
#include "language.h"

class ltl : language {
public:
  virtual bool match(base_rule::match_range& context, base_rule::match_range& result, std::shared_ptr<base_rule::node>& root);
};
#endif