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

#ifndef _UTIL_PARSER_REPETITIONOREPSILON_
#define _UTIL_PARSER_REPETITIONOREPSILON_

#include <memory>
#include <string>

#include <Syntx/util/parser/base_rule.h>

namespace util {
	namespace parser {
		/**
		 * Rule that matches a given rule an unlimited number of times
		 * and returns true any way.
		 */
		class repetition_or_epsilon : public base_rule {
			private:
				std::shared_ptr<base_rule> repeated_rule; /**< Rule to be matched zero or more times. */

				/**
				 * @copydoc util::parser::base_rule::test
				 */
				virtual bool test(base_rule::match_range &context, base_rule::match_range &the_match_range, std::shared_ptr<base_rule::node> &ast_root = base_rule::dont_build_ast) override;

			public:
				/**
				 * Constructor.
				 * @param repeated_rule rule to be matched zero or more times
				 */
				repetition_or_epsilon(base_rule const &repeated_rule) : repeated_rule(repeated_rule.clone()) {}

				/**
				 * @copydoc util::parser::base_rule::clone
				 */
				virtual std::shared_ptr<base_rule> clone() const override {
					return std::shared_ptr<base_rule>(new repetition_or_epsilon(*this));
				}
		};
	}
}

#endif // _UTIL_PARSER_REPETITIONOREPSILON_
