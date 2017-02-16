#ifndef __PARSE_TREE__
#define __PARSE_TREE__

#include <fstream>
#include <json/json.hpp>
#include <behavior_tree_core/BehaviorTree.h>
#include <ros/package.h>

using json = nlohmann::json;

namespace bt_parser {

/**
  Implements a behavior tree parser, that reads a JSON file describing
  the tree and generates the executable tree structure.
**/
class Parser {
public:
  /**
    @param filepath The directory of the JSON tree description.
  **/
  Parser(std::string filepath);
  ~Parser();

  /*
    Parses the JSON tree description.

    @return A pointer to the tree root.
    @throw logic_error
  */
  BT::TreeNode *parseTree();

private:
  std::ifstream file_;
  json tree_j_;
  std::string current_id_;

  /*
    Parses a subtree of the JSON description.

    @param node A json node object with a subtree to be parsed.
    @return A pointer to the subtree root.
    @throw logic_error
  */
  BT::TreeNode *parseTree(json node);

  /**
    Verifies the sanity of the given JSON node describing a BT subtree.

    @param node A json node object with a BT subtree.
    @throw logic_error
  **/
  void verifyNode(json node);
};
}
#endif
