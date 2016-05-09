#ifndef __INSERTION_WITH_DEFORMATION_ACTION__
#define __INSERTION_WITH_DEFORMATION_ACTION__
#include <ros/ros.h>
#include <behavior_tree_leaves/ActionTemplate.h>
#include <sarafun_assembly/InsertionAction.h>

namespace sarafun {
using sarafun_assembly;
class InsertionWithDeformationAction
    : ExecuteAction<InsertionAction, InsertionGoal> {
public:
  InsertionWithDeformationAction(std::string node_name, std::string action_name,
                                 bt_name)
      : ExecuteAction<InsertionAction, InsertionGoal>(node_name, action_name,
                                                      bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~InsertionWithDeformationAction() {}

  bool fillGoal(InsertionGoal &goal);
  int getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
