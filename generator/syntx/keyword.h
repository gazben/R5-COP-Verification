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

#ifndef _UTIL_PARSER_KEYWORD_
#define _UTIL_PARSER_KEYWORD_

#include <memory>
#include <string>

#include "base_rule.h"

namespace util {
  namespace parser {
    /**
     * Rule that matches a given keyword.
     * It tries to match the given keyword and then it looks at
     * the subsequent character which has to be one that is not allowed in
     * keywords (not a letter, a number or part of the given extra character set).
     * The reason for acting this way is that otherwise the rule would match any
     * word starting with the given keyword.
     */
    class keyword : public base_rule {
    private:
      std::string the_word;			/**< The keyword to match. */
      std::string extra_characters;	/**< The set of extra characters allowed in the keyword. */

      /**
       * @copydoc util::parser::base_rule::test
       */
      virtual bool test(base_rule::match_range &context, base_rule::match_range &the_match_range, std::shared_ptr<base_rule::node> &ast_root = base_rule::dont_build_ast) override;

    public:
      /**
       * Constructor.
       * @param the_word the keyword to match
       * @param extra_characters the set of extra characters that are allowed in the keyword
       */
      keyword(std::string const &the_word, std::string const &extra_characters = "_") :
        the_word(the_word),
        extra_characters(extra_characters) {}

      /**
       * @copydoc util::parser::base_rule::clone
       */
      virtual std::shared_ptr<base_rule> clone() const override {
        return std::shared_ptr<base_rule>(new keyword(*this));
      }

    protected:
      /**
       * @copydoc util::parser::base_rule::insert_failure_entry
       */
      virtual void insert_failure_entry(std::string::const_iterator const &position) const override;
    };
  }
}

#endif //_UTIL_PARSER_KEYWORD_
