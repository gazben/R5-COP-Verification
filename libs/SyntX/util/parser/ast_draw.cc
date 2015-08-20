#include "ast_draw.h"

size_t ast_draw::count_depth(std::shared_ptr<base_rule::node> const &node, size_t depth /*= 0*/)
{
  static size_t max_depth = 0;

  if (depth > max_depth)
    max_depth = depth;

  for (auto &a_node : node->children) {
    count_depth(a_node, depth + 1);
  }

  return max_depth;
}

std::string ast_draw::node_to_string(std::shared_ptr<base_rule::node> const &node) {
  std::string text;
  switch (node->the_type) {
  case base_rule::node::type::value:
    text = node->the_value;
    break;
  case base_rule::node::type::alternation:
    text = "alternation";
    break;
  case base_rule::node::type::concatenation:
    text = "concatenation";
    break;
  case base_rule::node::type::option:
    text = "option";
    break;
  case base_rule::node::type::repetition:
    text = "repetition";
    break;
  case base_rule::node::type::repetition_or_epsilon:
    text = "repetition_or_epsilon";
    break;
  case base_rule::node::type::named_rule:
    text = "Op_" + node->the_value;
    break;
  }
  return text;
}

void ast_draw::create_svg(std::shared_ptr<base_rule::node> const &node, size_t depth, size_t level_pos /*= 1*/, vector parentLinePoint /*= vector()*/)
{
  if (node) {
    std::string text = node_to_string(node);

    size_t x_pos;
    size_t y_pos;
    size_t offset = level_pos;
    if (offset % 2 == 0)
      offset += 1;

    x_pos = ((svg_width / (std::pow(2, depth + 1))) * offset);
    y_pos = depth * 120 + 70;

    //SYMBOL
    double svg_ellipse_rx = 50 + text.length() * 1.6;
    double svg_ellipse_ry = 35;

    svg_file_content +=
      "<ellipse cx=\"" +
      std::to_string(x_pos + (text.length() * 3.4)) +
      "\" cy=\"" +
      std::to_string(y_pos) +
      "\" rx=\"" + std::to_string(svg_ellipse_rx) +
      "\" ry=\"" + std::to_string(svg_ellipse_ry) +
      "\" style=\""
      "stroke:#ff0000;stroke-width: 2;stroke: black;fill: none;\"/>";
    svg_file_content += "<text x=\"" + std::to_string(x_pos) + "\" y=\"" + std::to_string(y_pos) + "\"" + " fill=\"black\">";
    svg_file_content += text + "</text>";
    svg_file_content += "\n";
    if (depth != 0) {
      svg_file_content +=
        "<line x1=\"" + std::to_string(parentLinePoint.x) +
        "\" y1=\"" + std::to_string(parentLinePoint.y) +
        "\" x2=\"" + std::to_string(x_pos + svg_ellipse_rx / 2.0) +
        "\" y2=\"" + std::to_string(y_pos - svg_ellipse_ry / 1.1) +
        "\" style=\"stroke: black;stroke-width:3\" />";
    }

    for (unsigned int i = 0; i < node->children.size(); i++) {
      size_t temp_level_pos = level_pos * 2;
      vector linePoint;
      if (i == 0) {
        temp_level_pos--;
        linePoint.x = x_pos;
        linePoint.y = y_pos + svg_ellipse_ry;
      }
      else {
        temp_level_pos++;
        linePoint.x = x_pos + svg_ellipse_rx;
        linePoint.y = y_pos + svg_ellipse_ry;
      }

      create_svg(node->children[i], depth + 1, temp_level_pos, linePoint);
    }
  }
}

void ast_draw::draw_to_file(std::shared_ptr<base_rule::node> const &node, std::string path /*= "ast.html"*/)
{
  using namespace std;

  double depth = count_depth(root);
  svg_width = pow(2, depth) * 70;
  svg_height = depth * 150 + 70;

  create_svg(root, 0);

  std::ofstream svg_file;
  svg_file.open(path);

  //svg_file << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>" << endl;
  svg_file << "<html><head></head><body>" << endl;
  svg_file << "<svg width=\"" << to_string(svg_width) << "\"  " << "height=\"" << to_string(svg_height) << "\">" << std::endl;

  svg_file << svg_file_content << endl;

  svg_file << "</svg>" << endl;
  svg_file << "</body></html>";
  svg_file.close();

  std::cout << "SVG_Height: " << svg_height << endl;
  std::cout << "SVG_Width: " << svg_width << endl;
  std::cout << "AST_depth: " << depth << endl;
}

void ast_draw::draw_to_file(std::string path /*= "ast.html"*/)
{
  draw_to_file(root);
}

void ast_draw::to_formatted_string(std::shared_ptr<base_rule::node> const &node, size_t depth /*= 0*/)
{
  if (node) {
    std::string text = node_to_string(node);
    for (size_t i = 0; i < depth; ++i) std::cout << "  ";
    std::cout << text << std::endl;

    for (auto &a_node : node->children)
      to_formatted_string(a_node, depth + 1);
  }
}

void ast_draw::to_formatted_string(size_t depth /*= 0*/)
{
  to_formatted_string(root);
}

ast_draw::ast_draw(std::shared_ptr < base_rule::node> _root) {
  root = _root;
}

void ast_draw::set_root(std::shared_ptr < base_rule::node> _root)
{
  root = _root;
}