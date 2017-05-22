#ifndef __TREE_FROM_KF__
#define __TREE_FROM_KF__

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <json/json.hpp>
#include <sarafun_msgs/KeyframeMsg.h>

using json = nlohmann::json;
namespace tree_generator {

const int NUM_TYPES = 7;
enum node_type {SEQ=0, SEL, SEQSTAR, SELSTAR, ACTION, CONDITION, LOADER};

/**
  This class constructs one pre-defined BT subtree,
  given the semantic information contained in a keyframe.

  The files describing the subtrees are defined in the /data/subtrees directory,
  and must have names matching the labels loaded in the parameter server.
**/
class SubTreeFromKF {
public:
  SubTreeFromKF();
  ~SubTreeFromKF(){}

  /**
    Recovers a BT subtree with a label that matches the keyframe.

    @param label The KF label that identifies the subtree.
    @return True in case the label exists and the file is recovered. False otherwise.
  **/
  bool loadLabel(std::string label);

  /*
    Creates the subtree for the loaded label, adjusting the id's
    based on the given indices. Updates the given indices.

    @param indices Vector of indices that define the number of actions and conditions
    of a given label that have occurred.
    @return A JSON node with the JSON description of the subtree defined by the
    KD label
    @throw logic_error
  */
  json createSubTree(std::vector<int> &indices);

  /**
  Make sure that the given json tree has nodes with proper indices.

  @param node The JSON node with the tree description.
  @param indices The vector of indices.
  @return The modified json NODE.
  @throw logic_error
  **/
  json modifyId(json node, std::vector<int> &indices);

private:
  ros::NodeHandle nh_;
  json subtree_;
  std::ifstream file_;
  bool has_label_;
  std::string current_id_;
};

/**
  Creates a JSON behavior tree description from a sequence of keyframe labels.
  It works by aggregating different subtrees defined for each label.
**/
class TreeFromKF
{
public:
  TreeFromKF();
  ~TreeFromKF(){}

  /**
    Creates a JSON BT description from a sequence of keyframe labels.

    @param keyframes_list The list of received keyframe messages.
    @return The JSON node representing the JSON BT description.
  **/
  json createTree(const std::vector<sarafun_msgs::KeyframeMsg> &keyframes_list);

private:
  std::vector<int> indices_; // to keep track of the id's

  json tree_;
  SubTreeFromKF subtree_parser_;

  void addChildren(const json &tree, std::map<std::string, json> &children_map);

  /**
    Merges two trees under a sequence node.

    @param tree_a The tree to be executed first.
    @param tree_b The tree to be executed second.
    @return The JSON node with the two trees united under a sequence node.
  **/
  json mergeTrees(const json &tree_a, const json &tree_b);
};

/**
  Catches and replaces possible hifens '-' in a KF label with an underscore.

  @param The KF label.
**/
void replaceWithUnderscore(std::string &label);
}

#endif
