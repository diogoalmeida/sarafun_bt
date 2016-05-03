#ifndef __PARSE_TREE__
#define __PARSE_TREE__

#include <fstream>
#include <json/json.hpp>
#include <behavior_tree_core/BehaviorTree.h>
#include <ros/package.h>

using json = nlohmann::json;

namespace bt_parser {
class Parser {
public:
  Parser(std::string filepath);
  ~Parser();

  BT::TreeNode *parseTree();

private:
  std::ifstream file_;
  json tree_j_;

  BT::TreeNode *parseTree(json node);
  void verifyNode(json node);
};
}

#endif
