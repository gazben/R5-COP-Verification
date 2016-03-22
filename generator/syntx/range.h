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

#ifndef _UTIL_PARSER_RANGE_
#define _UTIL_PARSER_RANGE_

#include <memory>
#include <string>
#include <sstream>
#include <stdexcept>

#include "base_rule.h"

namespace util {
  namespace parser {
    /**
     * Rule that matches one character of a given range.
     */
    class range : public base_rule {
    private:
      char range_first;	/**< The lower endpoint of the range. */
      char range_last;	/**< The upper endpoint of the range. */

      /**
       * @copydoc util::parser::base_rule::test
       */
      virtual bool test(base_rule::match_range &context, base_rule::match_range &the_match_range, std::shared_ptr<base_rule::node> &ast_root = base_rule::dont_build_ast) override;

    public:
      /**
       * Constructor.
       * @param range_first the lower endpoint of the range
       * @param range_last the upper endpoint of the range
       * @throw std::invalid_argument
       */
      range(char range_first, char range_last) :
        range_first(range_first),
        range_last(range_last) {
        if (range_first > range_last) {
          std::stringstream str;
          str << "Range's lower bound (" << range_first << ") greater than upper bound (" << range_last << ").";
          throw std::invalid_argument(str.str());
        }
      }

      /**
       * @copydoc util::parser::base_rule::clone
       */
      virtual std::shared_ptr<base_rule> clone() const override {
        return std::shared_ptr<base_rule>(new range(*this));
      }

    protected:
      /**
       * @copydoc util::parser::base_rule::insert_failure_entry
       */
      virtual void insert_failure_entry(std::string::const_iterator const &position) const override;
    };
  }
}

#endif // _UTIL_PARSER_RANGE_
