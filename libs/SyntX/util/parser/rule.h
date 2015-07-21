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

#ifndef _UTIL_PARSER_RULE_
#define _UTIL_PARSER_RULE_

#include <string>
#include <memory>
#include <utility>
#include <exception>

#include <Syntx/util/parser/base_rule.h>

namespace util {
	namespace parser {
		/**
		 * A subclass of \ref base_rule that is used as the type of user-defined composite rules.
		 * This rule stores a double indirection to the composite rule. This is because the rule
		 * might become part of other composites before beeing defined, so it has to contain something
		 * that can be copied and doesn't change when the rule is defined. This thing is a pointer to a pointer
		 * that is assigned the address of the actual rule at definition. The address of the inner pointer
		 * doesn't change (only it contents) so it can be assigned to the outer pointer at contruction time and
		 * can be copied an arbitrary number of times.
		 */
		class rule : public base_rule {
			public:
				/**
				 * Exception thrown when a rule's inner rule is not initialized and the \ref base_rule::match
				 * function is called.
				 */
				class undefined_rule : public std::exception {
					public:
						/**
						 * Returns the error message of the exception.
						 * @return the error message of the exception
						 */
						virtual char const *what() const noexcept override {
							return "A rule was left undefined -- no inner rule was assigned to it.";
						}
				};

			private:
				std::shared_ptr< std::shared_ptr<base_rule> > the_rule; /**< The address of the pointer storing the address of the composite rule referred to by this object. */
				std::string rule_name; /**< The name of the rule which is used in the AST. */

				/**
				 * @copydoc util::parser::base_rule::test
				 */
				virtual bool test(match_range &context, match_range &the_match_range, std::shared_ptr<base_rule::node> &ast_root = base_rule::dont_build_ast) override;

			public:	
				/**
				 * Constructor.
				 * @param a_rule pointer to the composite rule referred to by this object
				 */
				rule(std::shared_ptr<base_rule> a_rule = std::shared_ptr<base_rule>()) : the_rule(new std::shared_ptr<base_rule>(a_rule)) {}

				/**
				 * Constructs an empty rule with a name.
				 * @param a_name the name of the rule
				 */
				rule(std::string const &a_name) :
					the_rule(new std::shared_ptr<base_rule>()),
					rule_name(a_name) {}

				/**
				 * Sets the address of the composite rule.
				 * @param a_rule the address of the composite rule
				 */
				void set_rule(std::shared_ptr<base_rule> a_rule) {
					*the_rule = a_rule;
				}

				/**
				 * Gets the address of the composite rule.
				 * @return a_rule the address of the composite rule
				 */
				std::shared_ptr<base_rule> get_rule() {return *the_rule;}

				/**
				 * @copydoc util::parser::base_rule::clone
				 */
				virtual std::shared_ptr<base_rule> clone() const override {
					return std::shared_ptr<base_rule>(new rule(*this));
				}

				/**
				 * Assigns a \ref base_rule to the \ref rule.
				 */
				rule &operator <<=(base_rule const &rhs);

			protected:
				/**
				 * @copydoc util::parser::base_rule::insert_failure_entry
				 */
				virtual void insert_failure_entry(std::string::const_iterator const &position) const override;
		};

		/**
		 * Creates a \ref concatenation rule.
		 * @param first the first rule to be matched
		 * @param second the second rule to be matched
		 */
		rule operator <<(base_rule const &first, base_rule const &second);

		/**
		 * Creates a \ref alternation rule.
		 * @param first the first alternative
		 * @param second the second alternative
		 */
		rule operator |(base_rule const &first, base_rule const &second);

		/**
		 * Creates a \ref option rule.
		 * @param optional_rule the optional rule
		 */
		rule operator !(base_rule const &optional_rule);

		/**
		 * Creates a \ref repetition rule.
		 * @param repeated_rule the repeated rule
		 */
		rule operator +(base_rule const &repeated_rule);

		/**
		 * Creates a \ref repetition_or_epsilon rule.
		 * @param repeated_rule the repeated rule
		 */
		rule operator *(base_rule const &repeated_rule);

		/**
		 * Creates a rule that is preceeded by whitespace consumption using the \ref whitespace rule.
		 * @param a_rule the rule before which whitespaces are to be consumed
		 * @note This means that every whitespace is consumed before the given rule.
		 */
		rule operator -(base_rule const &a_rule);

		/**
		 * Creates a rule that is preceeded by whitespace consumption using the \ref whitespace_not_newline rule.
		 * @param a_rule the rule before which whitespaces are to be consumed
		 * @note This means that every whitespace is consumed before the given rule except for the newline character.
		 */
		rule operator ~(base_rule const &a_rule);
	}
}

#endif // _UTIL_PARSER_RULE_

