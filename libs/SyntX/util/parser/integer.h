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

#ifndef _UTIL_PARSER_INTEGER_
#define _UTIL_PARSER_INTEGER_

#include <memory>
#include <string>

#include <SyntX/util/parser/base_rule.h>

namespace util {
  namespace parser {
    /**
     * Rule that matches an integer number.
     * Whether the number should be signed or unsigned can be chosen at
     * construction using \ref integer::type.
     */
    class integer : public base_rule {
    public:
      /**
       * The type of the integer: signed or unsigned.
       */
      enum class type {
        signed_int, /**< The type that represents signed integers. */
        unsigned_int /**< The type that represents unsigned integers. */
      };

    private:
      type the_integer_type; /**< The integer type. */

    public:
      /**
       * Constructor.
       * @param the_integer_type the \ref type of the integer (signed or unsigned)
       */
      integer(type the_integer_type = type::signed_int) : the_integer_type(the_integer_type) {}

      /**
       * @copydoc util::parser::base_rule::test
       */
      virtual bool test(base_rule::match_range &context, base_rule::match_range &the_match_range, std::shared_ptr<base_rule::node> &ast_root = base_rule::dont_build_ast) override;

      /**
       * @copydoc util::parser::base_rule::clone
       */
      virtual std::shared_ptr<base_rule> clone() const override {
        return std::shared_ptr<base_rule>(new integer(*this));
      }

    protected:
      /**
       * @copydoc util::parser::base_rule::insert_failure_entry
       */
      virtual void insert_failure_entry(std::string::const_iterator const &position) const override;
    };
  }
}

#endif //_UTIL_PARSER_INTEGER_
