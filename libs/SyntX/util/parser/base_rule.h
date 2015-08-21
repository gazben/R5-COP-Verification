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

#ifndef _UTIL_PARSER_BASERULE_
#define _UTIL_PARSER_BASERULE_

#include <vector>
#include <string>
#include <memory>
#include <vector>
#include <utility>
#include <exception>
#include <functional>

#include <SyntX/util/enum_set.h>

namespace util {
  namespace parser {
    /**
     * The base class of the rules.
     * The operation to be done when there is a match are defined in this class.
     */
    class base_rule {
    public:
      using semantic_action = std::function<void(std::string const &)>; /**< The function type of semantic actions. */
      using match_range = std::pair< std::string::const_iterator, std::string::const_iterator >; /**< The type of ranges representing match results and ranges to match. */

      /**
       * Represents a node of the AST (Abstract Syntax Tree) that is automatically built while parsing if \ref build_ast is set to true.
       */
      struct node {
        /**
         * The type of the node.
         */
        enum class type {
          /**
           * The node represents a value (a leaf in the AST).
           */
          value,
          /**
           * The node represents an \ref util::parser::alternation -- its child is the alternative that matched.
           */
          alternation,
          /**
           * The node represents a \ref util::parser::concatenation -- its children are the two elements that appear one after the other.
           */
          concatenation,
          /**
           * The node represents an \ref util::parser::option -- its child is the optional element or a null pointer.
           */
          option,
          /**
           * The node represents a \ref util::parser::repetition -- its children are the elements of the type that is repeated.
           */
          repetition,
          /**
           * The node represents a \ref util::parser::repetition_or_epsilon -- its children are the elements of the type that is repeated or a null pointer.
           */
          repetition_or_epsilon,
          /**
           * A node representing a named rule.
           */
          named_rule, 
          /**
          * The node represent a deleted node waiting, for deletion.
          */
          deleted
        };

        type the_type; /**< The \ref util::parser::base_rule::node::type of the node. */
        std::string the_value; /**< The value of the node. */
        std::vector<std::shared_ptr<node>> children; /**< The children of the node. */
        std::shared_ptr<node> parent; /**< The parent of the node. */

        std::shared_ptr<node> left_children() {
          if (children.size() == 0)
            return nullptr;
          else
            return children[0];
        }

        std::shared_ptr<node> right_children() {
          if (children.size() == 0 || children.size() == 1)
            return nullptr;
          else 
            return children[1];
        }


        /**
         * Constructs a value node.
         * @param a_value the value of the node
         */
        node(std::string a_value) : the_type(type::value), the_value(a_value) {}

        /**
         * Constructs a node of the given type (tipically used for tyeps other than value).
         * @param a_type the type of the node
         */
        node(type a_type) : the_type(a_type) {}

        /**
        * Constructs a node of the given type (tipically used for tyeps other than value), and the value.
        * @param a_type the type of the node
        * @param a_value the value of the node
        */
        node(type a_type, std::string a_value) : the_type(a_type), the_value(a_value) {}
      };

      /**
       * The type of a rule.
       * Used in error entries.
       */
      enum class rule_type {
        terminal_rule, /**< A terminal rule operating at the character level. */
        semi_terminal_rule, /**< A rule that operates at the character level, but uses other character level rules inside. */
        named_rule /**< A named rule created by the user of the framework. */
      };

      /**
       * An entry in the failure log holdin the position of the
       * error, the type of the rule that failed and an error message
       * describing what the parser expected at that position.
       */
      struct failure_entry {
        std::string::const_iterator the_position; /**< The position of the error. */
        rule_type the_type; /**< The tye of the rule that failed. */
        std::string the_message; /**< The error message describing what the parser expected to find at the given position. */

        /**
         * Default constructor.
         */
        failure_entry() {}

        /**
         * Constructor.
         * @param a_position the position of the error
         * @param a_type the tye of the rule that failed
         * @param a_message the error message describing what the parser expected to find at the given position
         */
        failure_entry(std::string::const_iterator a_position, rule_type a_type, std::string const &a_message) :
          the_position(a_position),
          the_type(a_type),
          the_message(a_message) {}

        /**
         * Less than operator.
         * @param other another object
         * @return true if this one is less than the other
         */
        bool operator <(failure_entry const &other) const {
          return
            the_position < other.the_position &&
            static_cast<size_t>(the_type) < static_cast<size_t>(other.the_type) &&
            the_message < other.the_message;
        }

        /**
         * Equal to operator.
         * @param other another object
         * @return true if this one is equal to the other
         */
        bool operator ==(failure_entry const &other) const {
          return
            the_position == other.the_position &&
            static_cast<size_t>(the_type) == static_cast<size_t>(other.the_type) &&
            the_message == other.the_message;
        }
      };

      /**
       * The failure log that holds the components that the error message is constructed of.
       */
      class failure_log {
      public:
        using container_type = std::vector<failure_entry>; /**< The type of the container that is used to store the log.*/

      private:
        container_type the_log; /**< The log.*/

      public:
        /**
         * Inserts a new entry to the log.
         * @param an_entry the new entry
         */
        void insert(failure_entry const &an_entry);

        /**
         * Gets the container that holds the log.
         * @return the container that holds the log
         */
        container_type const &get_log() const { return the_log; }

        /**
         * Gets the container that holds the log.
         * @return the container that holds the log
         */
        container_type &get_log() { return the_log; }

        /**
         * Clears the log.
         * @note It is necessary to do that between parses.
         */
        void clear_log() { the_log.clear(); }

        /**
         * Checks whether the log is empty.
         * @return true if the log is empty
         */
        bool empty() const { return the_log.empty(); }
      };

      /**
       * Exception thrown when \ref base_rule::get_failure_cause is called on an empty failure log.
       */
      class empty_failure_log : public std::exception {
      public:
        /**
         * Destructor.
         */
        ~empty_failure_log() noexcept {}

        /**
         * Yields the error message of the exception.
         * @return the error message of the exception.
         */
        char const *what() const noexcept;
      };

    private:
      semantic_action the_semantic_action; /**< The semantic action assigned to this rule. */
      static bool build_ast; /**< Stores whether an AST is to be built. */

      /**
       * The virtual method that defines the conditions of matching.
       * This method is defined by every descendent of the @ref base_rule class.
       * @param context the range to be matched (the first iterator is incremented if the rule matches,
       * the second points to the end of the text)
       * @param the_match_range the beginning and end of the matched range -- this variable
       * is only altered if there is a match
       * @param ast_root the pointer to the root of the AST that is built by the rule
       * @return true if there is a match
       * @note This method may throw an exception (\ref rule::test throws \ref rule::undefined_rule).
       */
      virtual bool test(match_range &context, match_range &the_match_range, std::shared_ptr<node> &ast_root = dont_build_ast) = 0;

    public:
      /**
       * Destructor.
       */
      virtual ~base_rule() {}

      /**
       * Contains the operations performed when a rule is tried on the current position.
       * The method first tries to apply the rule at the current position using the virtual
       * \ref test method. If there is a match, the_match_range is set to represent the
       * range consumed by the rule and \ref the_semantic_action is called (if one has been
       * assigned to this rule).
       * @param context the range to be matched
       * @param the_match_range the consumed range
       * @param ast_root the pointer to the root of the AST that is built by the rule
       * @return true if there was a match
       */
      bool match(match_range &context, match_range &the_match_range, std::shared_ptr<node> &ast_root = dont_build_ast);

      /**
       * Assigns a \ref semantic_action to the rule.
       * @param a_semantic_action the \ref semantic_action to be assigned to the rule
       * @return reference of the rule
       */
      base_rule &operator[](semantic_action const &a_semantic_action);

      /**
       * Returns a dynamically allocated copy of this object as base_rule pointer.
       * @return a dynamically allocated copy of this object as base_rule pointer
       */
      virtual std::shared_ptr<base_rule> clone() const = 0;

      /**
       * Reads the contents of a text file to a std::string.
       * @param filename the file to be read
       * @param the_contents the std::string to be filled
       * @return true if the operation was successful
       */
      static bool file_to_string(std::string const &filename, std::string &the_contents);

      /**
       * Sets \ref build_ast that stores whether an AST is built during the parsing process.
       * @param a_value the new value of \ref build_ast.
       */
      static void set_build_ast(bool a_value) { build_ast = a_value; }

      /**
       * Returns the current value of \ref build_ast.
       * @return the current value of \ref build_ast
       */
      static bool get_build_ast() { return build_ast; }

      /**
       * Generates an error message based on the failure log.
       * @param context the bounds of the parsed text
       * @note The context is needed as the bounds of the text
       * are lost during the parsing process.
       * @param error_level the type of rules to include in the error message
       * @return an error message based on the failure log
       */
      static std::string get_error_message(match_range const &context, util::enum_set<rule_type> const &error_level = enum_set<rule_type>({ rule_type::terminal_rule }));

      /**
       * Clears the \ref failure_log.
       * @note This method has to be called between parses. The failure_log is a
       * static variable and rules simply write it, so entries from two subsequent
       * parses could get mixed up causing trouble.
       */
      static void clear_failure_log() { the_failure_log.clear_log(); }

      /**
       * Gets the \ref failure_log.
       * @return the \ref failure_log
       */
      static failure_log::container_type const &get_failure_log() {
        return the_failure_log.get_log();
      }

    protected:
      /**
       * Rules corresponding to terminal symbols enter data in the failure log if they fail to match.
       * Only the terminal rules need to act so it is defined as an empty method so as not to
       * litter the code of other rules.
       * @param position the position where the rule failed
       */
      virtual void insert_failure_entry(std::string::const_iterator const &position) const {}

    protected:
      static std::shared_ptr<node> dont_build_ast; /**< The default value of AST root (used when the tree is not built). */
      static failure_log the_failure_log; /**< The log where terminal symbols enter data when they fail.*/
    };
  }
}

#endif // _UTIL_PARSER_BASERULE_
