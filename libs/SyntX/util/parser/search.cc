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

#include <string>

#include <SyntX/util/parser/search.h>

namespace util {
  namespace parser {
    bool search(base_rule::match_range the_context, base_rule &the_rule, base_rule::match_range &the_result) {
      bool matched = false;

      for (; the_context.first != the_context.second && !matched; ++the_context.first) {
        base_rule::match_range local(the_context);

        matched = the_rule.match(local, the_result);
      }

      return matched;
    }

    bool search_all(base_rule::match_range the_context, base_rule &the_rule, base_rule::semantic_action const &an_action) {
      bool success = false;

      for (; the_context.first != the_context.second; ++the_context.first) {
        base_rule::match_range local(the_context), the_result;

        bool matched = the_rule.match(local, the_result);

        if (matched) {
          success = true;

          if (an_action) an_action(std::string(the_result.first, the_result.second));

          the_context.first = the_result.second;
        }
      }

      return success;
    }
  }
}