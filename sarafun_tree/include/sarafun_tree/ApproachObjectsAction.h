#ifndef __APPROACH_OBJECTS_ACTION__
#define __APPROACH_OBJECTS_ACTION__
#include <ros/ros.h>
#include <behavior_tree_leaves/ActionTemplate.h>

namespace sarafun {
class ApproachObjectsAction : ActionTemplate {
public:
  ApproachObjectsAction(std::string node_name, std::string action_name)
      : node_name_(node_name), ActionTemplate(action_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~ApproachObjectsAction() {}

  void preemptionRoutine();
  int executionRoutine();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
};
}
#endif
