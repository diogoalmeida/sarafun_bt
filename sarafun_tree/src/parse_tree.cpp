#include <sarafun_tree/parse_tree.h>

namespace bt_parser {

Parser::Parser(std::string filepath) { file_.open(filepath); }
Parser::~Parser() { file_.close(); }

void Parser::verifyNode(const json &node) {
  if (!node.count("name")) {
    std::string id = node["id"];
    std::string error_message("The BT JSON file nodes must all have the 'name' "
                              "member! Offending node has id: ");

    error_message = error_message + id;

    throw std::logic_error(error_message);
  }
  if (!node.count("id") || !node.count("type")) {
    std::string name = node["name"];
    std::string error_message("The BT JSON file nodes must all have the 'id', "
                              "'type' members! Offending node has name: ");

    error_message = error_message + name;

    throw std::logic_error(error_message);
  }
}

BT::TreeNode *Parser::parseTree() {
  try {
    current_id_ = "root";
    file_ >> tree_j_;

    if (!tree_j_.count("root") || !tree_j_.count("nodes")) {
      throw std::logic_error(
          std::string("The tree file must declare 'root' and 'nodes' members "
                      "at the base level!"));
    }
    std::string root_id = tree_j_["root"];

    json root_node = tree_j_["nodes"][root_id];

    return parseTree(root_node);

  } catch (const std::invalid_argument &e) {
    ROS_ERROR("Error parsing BT json file: %s. Current id: %s", e.what(),
              current_id_.c_str());
  } catch (const std::logic_error &e) {
    ROS_ERROR("Error with BT Node: %s.", e.what());
  } catch (const std::exception &e) {
    ROS_ERROR("Unknown exception: %s.", e.what());
  }
  return nullptr;
}

BT::TreeNode *Parser::parseTree(const json &node) {
  verifyNode(node);

  std::string type = node["type"];
  std::string id = node["id"];
  current_id_ = id;

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
  } else if (type == std::string("Condition")) {
    is_leaf = true;
    std::string name = node["name"];
    bt_node = new BT::ROSCondition(name);
  } else if (type == std::string("Loader")) {
    is_leaf = true;
    std::string name = node["name"];
    bt_node = new BT::ROSLoaderNode(name);
    storeParameters(dynamic_cast<BT::ROSLoaderNode*>(bt_node), node); // get the parameters to be loaded when this node is tick'ed
  } else {
    std::string error_message("BT input file includes an unknown node type: ");
    error_message = error_message + type;
    throw std::logic_error(error_message);
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

void Parser::storeParameters(BT::ROSLoaderNode *bt_node, const json &node){
  json params = node["parameters"];
  std::string domain, string_value;
  int int_value;
  double double_value;

  for (json::iterator it = params.begin(); it != params.end(); ++it)
  {
    domain = it.key();

    if(it.value().is_string())
    {
      string_value = it.value();
      bt_node->addParameter(domain, string_value);
    }
    else if (it.value().is_number())
    {
      if (it.value().is_number_integer())
      {
        int_value = it.value();
        bt_node->addParameter(domain, int_value);
      }
      else
      {
        double_value = it.value();
        bt_node->addParameter(domain, double_value);
      }
    }
    else
    {
      std::string error_message("Load node has unsupported parameter type: "); // TODO: Add human-readable type
      throw std::logic_error(error_message);
    }
  }
}
}
