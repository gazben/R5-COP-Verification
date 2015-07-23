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

#ifndef _UTIL_PARSER_WHITESPACE_NOT_NEWLINE_
#define _UTIL_PARSER_WHITESPACE_NOT_NEWLINE_

#include <memory>
#include <string>

#include <SyntX/util/parser/base_rule.h>

namespace util {
	namespace parser {
		/**
		 * Rule that tries to match the given rule after having consumed every whitespace before the
		 * rule except for the newline character.
		 */
		class whitespace_not_newline : public base_rule {
			private:
				std::shared_ptr<base_rule> a_rule;		/**< The rule to be matched after consuming the whitespaces. */
				static char whitespace_characters[];	/**< The characters considered as whitespace (except for the newline character). */

				/**
				 * @copydoc util::parser::base_rule::test
				 */
				virtual bool test(base_rule::match_range &context, base_rule::match_range &the_match_range, std::shared_ptr<base_rule::node> &ast_root = base_rule::dont_build_ast) override;

			public:
				/**
				 * Constructor.
				 * @param a_rule rule to be matched after consuming the whitespaces
				 */
				whitespace_not_newline(base_rule const &a_rule) : a_rule(a_rule.clone()) {}

				/**
				 * @copydoc util::parser::base_rule::clone
				 */
				virtual std::shared_ptr<base_rule> clone() const override {
					return std::shared_ptr<base_rule>(new whitespace_not_newline(*this));
				}

			private:
				/**
				 * Determines whether the given character is a whitespace or not.
				 * @param c the character to be examined
				 */
				bool is_whitespace_not_newline(char c);
		};
	}
}

#endif // _UTIL_PARSER_WHITESPACE_NOT_NEWLINE_

