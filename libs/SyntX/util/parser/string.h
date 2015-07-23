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

#ifndef _UTIL_PARSER_STRING_
#define _UTIL_PARSER_STRING_

#include <memory>
#include <sstream>

#include <SyntX/util/parser/base_rule.h>

namespace util {
	namespace parser {
		/**
		 * Matches a string literal of a formal language.
		 * The delimiter character and the character used as an escape character
		 * can be given.
		 */
		class string : public base_rule {
			private:
				char delimiter; /**< The string delimiter character. */
				char escape_character; /**< The escape_character. */

				/**
				 * @copydoc util::parser::base_rule::test
				 */
				virtual bool test(base_rule::match_range &context, base_rule::match_range &the_match_range, std::shared_ptr<base_rule::node> &ast_root = base_rule::dont_build_ast) override;
				
			public:
				/**
				 * Constructor.
				 * @param delimiter the delimiter character
				 * @param escape_character the escape character
				 */
				string(char delimiter = '\"', char escape_character = '\\') :
					delimiter(delimiter),
					escape_character(escape_character) {}

				/**
				 * @copydoc util::parser::base_rule::clone
				 */
				virtual std::shared_ptr<base_rule> clone() const override {
					return std::shared_ptr<base_rule>(new string(*this));
				}

			protected:
				/**
				 * @copydoc util::parser::base_rule::insert_failure_entry
				 */
				virtual void insert_failure_entry(std::string::const_iterator const &position) const override;
		};
	}
}

#endif //_UTIL_PARSER_STRING_
