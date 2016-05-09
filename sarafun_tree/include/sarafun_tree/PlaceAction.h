#ifndef __PLACE_ACTION__
#define __PLACE_ACTION__
#include <ros/ros.h>
#include <behavior_tree_leaves/ActionTemplate.h>
#include <sarafun_manipulation/PlaceAction.h>

namespace sarafun {
using sarafun_manipulation;
class PlaceAction : ExecuteAction<PlaceAction, PlaceGoal> {
public:
  PlaceAction(std::string node_name, std::string action_name, bt_name)
      : ExecuteAction<PlaceAction, PlaceGoal>(node_name, action_name, bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~PlaceAction() {}

  void preemptionRoutine();
  int executionRoutine();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
