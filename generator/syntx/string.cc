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

#include <tuple>
#include <sstream>

#include "string.h"

namespace util {
  namespace parser {
    bool string::test(base_rule::match_range &context, base_rule::match_range &the_match_range, std::shared_ptr<base_rule::node> &ast_root) {
      if (context.first == context.second) return false;

      base_rule::match_range local_context = context;

      if (*local_context.first == delimiter) {
        ++local_context.first;

        while (local_context.first != local_context.second && *local_context.first != delimiter) {
          if (*local_context.first == escape_character) ++local_context.first;
          ++local_context.first;
        }

        ++local_context.first;

        the_match_range.first = context.first;
        the_match_range.second = local_context.first;
        context = local_context;

        if (get_build_ast()) ast_root = std::make_shared<base_rule::node>(std::string(the_match_range.first, the_match_range.second));

        return true;
      }
      else return false;
    }

    void string::insert_failure_entry(std::string::const_iterator const &position) const {
      std::stringstream stream;
      stream << "a string with delimiter: {" + std::string(1,delimiter) + "}";
      the_failure_log.insert(base_rule::failure_entry(position, base_rule::rule_type::terminal_rule, stream.str()));
    }
  }
}