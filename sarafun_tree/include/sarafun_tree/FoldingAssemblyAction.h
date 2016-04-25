#ifndef __FOLDING_ASSEMBLY_ACTION__
#define __FOLDING_ASSEMBLY_ACTION__
#include <ros/ros.h>
#include <behavior_tree_leaves/ActionTemplate.h>

namespace sarafun {
class FoldingAssemblyAction : ActionTemplate {
public:
  FoldingAssemblyAction(std::string node_name, std::string action_name)
      : node_name_(node_name), ActionTemplate(action_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~FoldingAssemblyAction() {}

  void preemptionRoutine();
  int executionRoutine();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
};
}
#endif
