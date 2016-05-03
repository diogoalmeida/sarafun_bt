#include <sarafun_tree/parse_tree.h>

namespace bt_parser {

Parser::Parser(std::string filepath) {
  file_.open(filepath);
  file_ >> tree_j_;
}

BT::TreeNode *Parser::parseTree() {
  std::string root_id = tree_j_["root"];

  json root_node = tree_j_["nodes"][root_id];

  return parseTree(root_node);
}

BT::TreeNode *Parser::parseTree(json node) {
  std::string type = node["type"];
  std::string id = node["id"];
  bool is_leaf = false;
  BT::TreeNode *bt_node;

  if (type == std::string("Selector")) {
    bt_node = new BT::SelectorNode(id);
  } else if (type == std::string("SelectorStar")) {
    bt_node = new BT::SelectorStarNode(id);
  } else if (type == std::string("Sequence")) {
    bt_node = new BT::SequenceNode(id);
  } else if (type == std::string("SequenceStar")) {
    bt_node = new BT::SequenceStarNode(id);
  } else if (type == std::string("Action")) {
    is_leaf = true;
    std::string name = node["name"];
    bt_node = new BT::ROSAction(name);
  }

  if (!is_leaf) {
    std::vector<std::string> children = node["children"];

    for (std::vector<std::string>::iterator i = children.begin();
         i != children.end(); i++) {
      bt_node->AddChild(parseTree(tree_j_["nodes"][*i]));
    }
  }

  return bt_node;
}
}
