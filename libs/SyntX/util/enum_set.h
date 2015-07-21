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

#ifndef _UTIL_ENUMSET_
#define _UTIL_ENUMSET_

#include <unordered_set>
#include <initializer_list>

namespace util {
	/**
	 * Contains a set of constants from the same enumeration type.
	 * Enables to store a list of settings that would traditionally be
	 * stored as the OR-ed value of constants.
	 */
	template <typename enum_type>
		class enum_set {
			private:
				/**
				 * A hash helper class for the unordered_set. 
				 */
				class enum_hash {
					public:
						/**
						 * The function call operator that calculates the hash.
						 * @param value the enum value that needs a hash
						 * @return the hash value of the argument
						 */
						size_t operator()(enum_type const &value) const {
							return static_cast<size_t>(value);
						}
				};

				std::unordered_set<enum_type, enum_hash> the_enum_set; /**< The set that contains the values.*/

			public:
				/**
				 * Default constructor.
				 */
				enum_set() {}

				/**
				 * Constructor that takes an initializer that may contain
				 * an arbitrary number of constants.
				 * @param enum_list a list of constants that the \ref enum_set should hold.
				 */
				enum_set(std::initializer_list<enum_type> enum_list) {
					for (auto an_enum: enum_list)
						the_enum_set.insert(an_enum);
				}

				/**
				 * Tests whether a constant is in the set or not.
				 * @param a_value the value to be tested
				 * @return true if the value is in the set
				 */
				bool has(enum_type a_value) const {
					return the_enum_set.find(a_value) != the_enum_set.end();
				}

				/**
				 * Inserts a new value to the set.
				 * @param a_value the value to be inserted 
				 */
				void insert(enum_type a_value) {
					the_enum_set.insert(a_value);
				}

				/**
				 * Inserts a list of new values to the set.
				 * @param enum_list the values to be inserted 
				 */
				void insert(std::initializer_list<enum_type> enum_list) {
					for (auto an_enum: enum_list) {
						the_enum_set.insert(an_enum);
					}
				}

				/**
				 * Removes a value from the set.
				 * @param a_value the value to be erased
				 */
				void remove(enum_type a_value) {
					the_enum_set.erase(a_value);
				}

				/**
				 * Removes a list of new values from the set.
				 * @param enum_list the values to be removed 
				 */
				void remove(std::initializer_list<enum_type> enum_list) {
					for (auto an_enum: enum_list) {
						the_enum_set.erase(an_enum);
					}
				}
		};

	/**
	 * Constructs an \ref enum_set from an initialiyer list of enum constants.
	 * @param enum_list list of values
	 * @return an \ref enum_set holding the given values
	 */
	template <typename enum_type>
		enum_set<enum_type> make_enum_set(std::initializer_list<enum_type> enum_list) {
			return enum_set<enum_type>(enum_list);
		}
}
#endif // _UTIL_ENUMSET_
