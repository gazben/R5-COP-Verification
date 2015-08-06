#ifndef ast_draw_h__
#define ast_draw_h__

#include <memory>
#include <fstream>
#include <iostream>
#include <cmath>

#include "base_rule.h"
using namespace util::parser;

class ast_draw {
private:
  std::shared_ptr<base_rule::node> root;
  size_t svg_width = 0;
  size_t svg_height = 0;
  std::string svg_file_content;

  std::shared_ptr<base_rule::node> rearrange_operators(std::shared_ptr<base_rule::node> &node);
  std::shared_ptr<base_rule::node> remove_character_leafs(std::shared_ptr<base_rule::node> &node);
  std::shared_ptr<base_rule::node> remove_alternations(std::shared_ptr<base_rule::node> &node);
  std::shared_ptr<base_rule::node> remove_one_children_roots(std::shared_ptr<base_rule::node> &node);

  void remove_nodes_marked_for_deletion(std::shared_ptr<base_rule::node> &node);
  struct vector {
    vector(double _x = 0, double _y = 0)
      :x(_x), y(_y)
    {
    }
    double x;
    double y;
  };

  size_t count_depth(std::shared_ptr<base_rule::node> const &node, size_t depth = 0);

  void create_svg(std::shared_ptr<base_rule::node> const &node, size_t depth, size_t level_pos = 1, vector parentLinePoint = vector());
public:

  ast_draw(std::shared_ptr < base_rule::node> = nullptr);

  void set_root(std::shared_ptr < base_rule::node>);
  void draw_to_file(std::shared_ptr<base_rule::node> const &node, std::string path = "ast.html");
  void draw_to_file(std::string path = "ast.html");
  void to_formatted_string(std::shared_ptr<base_rule::node> const &node, size_t depth = 0);
  void to_formatted_string(size_t depth = 0);
  std::shared_ptr<base_rule::node> optimize_ast(std::shared_ptr<base_rule::node> &node);
  std::shared_ptr<base_rule::node> optimize_ast();
};

#endif // ast_draw_h__
