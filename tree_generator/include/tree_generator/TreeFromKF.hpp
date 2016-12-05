#ifndef __TREE_FROM_KF__
#define __TREE_FROM_KF__

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <json/json.hpp>
#include <sarafun_msgs/KeyframeList.h>
#include <sarafun_msgs/KeyframeMsg.h>

using json = nlohmann::json;
namespace tree_generator {

enum node_type {SEQ=0, SEL, SEQSTAR, SELSTAR, ACTION, CONDITION};

/*
  This class constructs one pre-defined BT subtree,
  given the semantic information contained in a keyframe
*/
class SubTreeFromKF {
public:
  SubTreeFromKF();
  ~SubTreeFromKF(){}

  bool loadLabel(std::string label);
  json createSubTree(std::vector<int> &indices);

private:
  ros::NodeHandle nh_;
  json subtree_;
  std::ifstream file_;
  bool has_label_;
  std::string current_id_;

  json modifyId(json node, std::vector<int> &indices);
};

/*
  Class responsible from creating a JSON file from the
  given keyframe sequence
*/
class TreeFromKF
{
public:
  TreeFromKF();
  ~TreeFromKF(){}

  json createTree(const sarafun_msgs::KeyframeList &keyframes);

private:
  std::vector<int> indices_; // to keep track of the id's

  json tree_;
  SubTreeFromKF subtree_parser_;
  void addChildren(const json &tree, std::map<std::string, json> &children_map);
};

void replaceWithUnderscore(std::string &label);
}

#endif
