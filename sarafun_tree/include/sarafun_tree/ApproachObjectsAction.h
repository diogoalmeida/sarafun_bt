#ifndef __APPROACH_OBJECTS_ACTION__
#define __APPROACH_OBJECTS_ACTION__
#include <ros/ros.h>
#include <sarafun_tree/ExecuteAction.h>
#include <sarafun_manipulation/ApproachAction.h>

namespace sarafun {
using sarafun_manipulation;
class ApproachObjectsAction : ExecuteAction<ApproachAction, ApproachGoal> {
public:
  ApproachObjectsAction(std::string node_name, std::string action_name, bt_name)
      : ExecuteAction<ApproachAction, ApproachGoal>(node_name, action_name,
                                                    bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~ApproachObjectsAction() {}

  bool fillGoal(ApproachGoal &goal);
  int getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
#endif
