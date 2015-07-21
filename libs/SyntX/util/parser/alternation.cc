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

#include <SyntX/util/parser/alternation.h>

namespace util {
	namespace parser {
		bool alternation::test(base_rule::match_range &context, base_rule::match_range &the_match_range, std::shared_ptr<base_rule::node> &ast_root) {
			base_rule::match_range first_range, second_range;
			base_rule::match_range local_context = context;
			std::shared_ptr<node> child_node;

			if (first->match(local_context, first_range, child_node)) {
				the_match_range = first_range;
				context = local_context;

				if (get_build_ast()) {
					ast_root = std::make_shared<base_rule::node>(base_rule::node::type::alternation);
					ast_root->children.push_back(child_node);
				}

				return true;
			}
			
			local_context = context;

			if (second->match(local_context, second_range, child_node)) {
				the_match_range = second_range;
				context = local_context;

				if (get_build_ast()) {
					ast_root = std::make_shared<base_rule::node>(base_rule::node::type::alternation);
					ast_root->children.push_back(child_node);
				}

				return true;
			}
			
			return false;
		}
	}
}

