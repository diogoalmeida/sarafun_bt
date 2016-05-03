#include <sarafun_tree/parse_tree.h>

namespace bt_parser {

Parser::Parser(std::string filepath) { file_.open(filepath); }
Parser::~Parser() { file_.close(); }

void Parser::verifyNode(json node) {
  if (!node.count("id") || !node.count("type") || !node.count("name")) {
    std::string error_message("The BT JSON file nodes must all have the 'id', "
                              "'type' and 'name' members!");
    throw std::logic_error(error_message);
  }
}

BT::TreeNode *Parser::parseTree() {
  try {
    file_ >> tree_j_;

    if (!tree_j_.count("root") || !tree_j_.count("nodes")) {
      throw std::logic_error(
          std::string("The tree file must declare 'root' and 'nodes' members "
                      "at the base level!"));
    }
    std::string root_id = tree_j_["root"];

    json root_node = tree_j_["nodes"][root_id];

    return parseTree(root_node);
  } catch (const std::exception &e) {
    ROS_ERROR("Error parsing BT json file: %s.", e.what());
  }
  return nullptr;
}

BT::TreeNode *Parser::parseTree(json node) {
  verifyNode(node);

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
