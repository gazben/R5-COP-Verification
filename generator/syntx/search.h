/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, Gergely Nagy
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _UTIL_PARSER_SEARCH_
#define _UTIL_PARSER_SEARCH_

#include <functional>

#include "base_rule.h"

namespace util {
  namespace parser {
    /**
     * Searches for a match of the given rule in the context.
     * Steps through the context until a match is found or the end of the context
     * is reached.
     * @param the_context the input text of the search
     * @param the_rule the main rule of the search
     * @param the_result points to the match
     * @return true if a successful match was found
     */
    bool search(base_rule::match_range the_context, base_rule &the_rule, base_rule::match_range &the_result);

    /**
     * Searches for all the matches of the given rule in the context.
     * Steps through the context and evry time there is a match, it calls
     * the given \ref semantic_action. It doesn't look for overlapping matches
     * (i.e. it jumps to end of a matched range when it continues to search).
     * This is merely to speed up the searches.
     * @param the_context the input text of the search
     * @param the_rule the main rule of the search
     * @param an_action the \ref semantic_action that is called at every search -- this is an optional parameter
     * @return true if at least one successful match was found
     */
    bool search_all(base_rule::match_range the_context, base_rule &the_rule, base_rule::semantic_action const &an_action = base_rule::semantic_action());
  }
}

#endif // _UTIL_PARSER_SEARCH_
