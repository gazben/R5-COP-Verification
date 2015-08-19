#ifndef ast_optimizer_h__
#define ast_optimizer_h__

#include <memory>
#include <SyntX/util/parser/parser.h>

class ast_optimizer {
public:
  static std::shared_ptr<base_rule::node> optimize_ast(std::shared_ptr<base_rule::node> &node);
private:
  static std::shared_ptr<base_rule::node> rearrange_operators(std::shared_ptr<base_rule::node> &node);
  static std::shared_ptr<base_rule::node> rearrange_oneop_operators(std::shared_ptr<base_rule::node> &node);
  static std::shared_ptr<base_rule::node> remove_lpar_rpar(std::shared_ptr<base_rule::node> &node);
  static std::shared_ptr<base_rule::node> remove_character_leafs(std::shared_ptr<base_rule::node> &node);
  static std::shared_ptr<base_rule::node> remove_alternations(std::shared_ptr<base_rule::node> &node);
  static std::shared_ptr<base_rule::node> remove_one_children_roots(std::shared_ptr<base_rule::node> &node);
  static void remove_nodes_marked_for_deletion(std::shared_ptr<base_rule::node> &node);
};


#endif // ast_optimizer_h__