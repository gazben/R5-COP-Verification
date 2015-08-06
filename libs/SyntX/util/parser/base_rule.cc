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
#include <iostream>

#include <cctype>
#include <fstream>
#include <sstream>
#include <iterator>
#include <algorithm>

#include <SyntX/util/parser/base_rule.h>

namespace util {
  namespace parser {
    std::shared_ptr<base_rule::node> base_rule::dont_build_ast;
    bool base_rule::build_ast = false;
    base_rule::failure_log base_rule::the_failure_log;

    bool base_rule::match(match_range &context, match_range &the_match_range, std::shared_ptr<node> &ast_root) {
      match_range a_range;

      if (test(context, a_range, ast_root)) {
        the_match_range = a_range;

        if (the_semantic_action) {
          std::string the_matched_substring(the_match_range.first, the_match_range.second);
          the_semantic_action(the_matched_substring);
        }

        return true;
      }
      else {
        insert_failure_entry(context.first);
        return false;
      }
    }

    base_rule &base_rule::operator[](semantic_action const &a_semantic_action) {
      the_semantic_action = a_semantic_action;
      return *this;
    }

    bool base_rule::file_to_string(std::string const &filename, std::string &the_contents) {
      std::fstream source(filename, std::ios::in);

      if (!source) return false;

      source >> std::noskipws;

      source.seekg(0, source.end);
      size_t length = source.tellg();
      source.seekg(0, source.beg);

      the_contents.reserve(length);
      std::copy(
        std::istream_iterator<char>(source),
        std::istream_iterator<char>(),
        std::inserter(
          the_contents,
          the_contents.end()
          )
        );

      return true;
    }

    char const *base_rule::empty_failure_log::what() const noexcept {
      return "The failure log is empty.";
    }

    std::string base_rule::get_error_message(match_range const &context, util::enum_set<rule_type> const &error_level) {
      if (the_failure_log.empty()) return "";

      auto const &first_entry = *(the_failure_log.get_log().crbegin());

      size_t characters_before = 0;
      std::string::const_iterator i_before = first_entry.the_position;
      std::string::const_iterator i_after = first_entry.the_position;

      while (i_before > context.first && *i_before != '\n') { ++characters_before; --i_before; }

      size_t line_number = 1;
      if (*i_before == '\n') {
        std::string::const_iterator line_iterator = i_before;
        while (line_iterator > context.first) {
          if (*line_iterator == '\n') ++line_number;
          --line_iterator;
        }

        ++i_before; //so i_before points to the first character of the line with the error
      }

      while (i_after < context.second && *i_after != '\n') ++i_after;

      std::stringstream message;

      message << "An error occured here (line " << line_number << "):" << std::endl;
      message << std::string(i_before, i_after) << std::endl;

      for (auto spaces = i_before; spaces < first_entry.the_position; ++spaces) {
        if (*spaces == '\t') message << "\t";
        else if (*spaces != '\n' && *spaces != '\r') message << " ";
      }

      message << "^" << std::endl << "The parser was expecting" << std::endl << "\t";

      auto &error_entries = the_failure_log.get_log();
      std::sort(error_entries.begin(), error_entries.end(),
        [](base_rule::failure_entry const &lhs, base_rule::failure_entry const &rhs) -> bool {
        return
          lhs.the_message < rhs.the_message;
      }
      );
      auto it = std::unique(error_entries.begin(), error_entries.end(),
        [](base_rule::failure_entry const &lhs, base_rule::failure_entry const &rhs) -> bool {
        return
          lhs.the_message == rhs.the_message;
      }
      );
      error_entries.resize(std::distance(error_entries.begin(), it));

      bool is_first = true;
      for (auto const &entry : error_entries) {
        if (error_level.has(entry.the_type)) {
          if (is_first) is_first = false;
          else message << std::endl << "\tor ";

          if (entry.the_type == rule_type::semi_terminal_rule) message << "[SEMI-TERMINAL]: ";
          if (entry.the_type == rule_type::named_rule) message << "[NAMED RULE]: ";
          message << entry.the_message;
        }
      }

      return message.str();
    }

    void base_rule::failure_log::insert(failure_entry const &an_entry) {
      if (!the_log.empty()) {
        auto the_max_element =
          std::max_element(
            the_log.cbegin(), the_log.cend(),
            [](failure_entry const &rhs, failure_entry const &lhs) {
          return rhs.the_position < lhs.the_position;
        }
        );

        if (the_max_element->the_position < an_entry.the_position) {
          the_log.clear();
          the_log.push_back(an_entry);
        }
        else if (the_max_element->the_position == an_entry.the_position) {
          the_log.push_back(an_entry);
        }
      }
      else {
        the_log.push_back(an_entry);
      }
    }
  }
}