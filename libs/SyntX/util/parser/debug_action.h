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

#ifndef _UTIL_PARSER_DEBUGACTION_
#define _UTIL_PARSER_DEBUGACTION_

#include <string>
#include <iostream>

namespace util {
	namespace parser {
		/**
		 * A functor that can be used to debug grammars.
		 * It prints the given strings at every match together with
		 * the consumed text.
		 */
		class debug_action {
			private:
				std::string text_before;	/**< String to be printed before the matched text. */
				std::string text_after;		/**< String to be printed after the matched text. */
				std::ostream &stream;		/**< The stream onto which the action should print.*/

			public:
				/**
				 * Constructor.
				 * @param text_before the string to be printed before the matched text
				 * @param text_after the string to be printed after the matched text
				 * @param stream the stream onto which the action should print
				 */
				debug_action(std::string const &text_before = "", std::string const &text_after = "", std::ostream &stream = std::cout) :
					text_before(text_before),
					text_after(text_after),
					stream(stream) {}

				/**
				 * The function call operator that is used when the \ref base_rule
				 * activates the action after a successful match.
				 * @param match the consumed text
				 */
				void operator()(std::string const &match) {
					stream << text_before << match << text_after;
				}
		};
	}
}
#endif // _UTIL_PARSER_DEBUGACTION_
