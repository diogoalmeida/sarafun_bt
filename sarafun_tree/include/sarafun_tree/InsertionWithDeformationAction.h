#ifndef __INSERTION_WITH_DEFORMATION_ACTION__
#define __INSERTION_WITH_DEFORMATION_ACTION__
#include <ros/ros.h>
#include <behavior_tree_leaves/ActionTemplate.h>

namespace sarafun {
class InsertionWithDeformationAction : ActionTemplate {
public:
  InsertionWithDeformationAction(std::string node_name, std::string action_name)
      : node_name_(node_name), ActionTemplate(action_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~InsertionWithDeformationAction() {}

  void preemptionRoutine();
  int executionRoutine();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
};
}
#endif
