#ifndef LANGUAGE_H
#define LANGUAGE_H

#include "parser.h"

using namespace util::parser;

class language {
public:
  virtual bool match(
    base_rule::match_range& context, 
    base_rule::match_range& result, 
    std::shared_ptr<base_rule::node>& root
    ) = 0;
};

#endif