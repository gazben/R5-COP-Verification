#ifndef ast_draw_h__
#define ast_draw_h__

#include <memory>
#include <fstream>
#include <iostream>
#include <cmath>

#include "base_rule.h"
using namespace util::parser;

class ast_draw{
private:
  std::shared_ptr<base_rule::node> root;
  size_t svg_width = 0;
  size_t svg_height = 0;
  std::string svg_file_content;

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

  ast_draw();

  void draw_to_file(std::shared_ptr<base_rule::node> const &node, std::string path = "ast.html");
  void to_formatted_string(std::shared_ptr<base_rule::node> const &node, size_t depth = 0);
  std::shared_ptr<base_rule::node> opimize_ast(std::shared_ptr<base_rule::node> &node);
};

#endif // ast_draw_h__
