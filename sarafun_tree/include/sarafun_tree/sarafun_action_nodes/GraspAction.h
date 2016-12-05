#ifndef __GRASP_ACTION__
#define __GRASP_ACTION__

#include <ros/ros.h>
#include <sarafun_tree/ExecuteAction.h>
#include <sarafun_msgs/GraspKeyframeAction.h>

namespace sarafun {
class GraspAction
    : ExecuteAction<sarafun_msgs::GraspKeyframeAction,
                    sarafun_msgs::GraspKeyframeGoal> {
public:
  GraspAction(std::string node_name, std::string action_name,
                        std::string bt_name)
      : ExecuteAction<
            sarafun_msgs::GraspKeyframeAction,
            sarafun_msgs::GraspKeyframeGoal>::ExecuteAction(node_name,
                                                               action_name,
                                                               bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~GraspAction() {}

  bool fillGoal(sarafun_msgs::GraspKeyframeGoal &goal);
  double getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
#endif
