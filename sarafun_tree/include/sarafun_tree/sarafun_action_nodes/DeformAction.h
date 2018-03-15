#ifndef __DEFORM_ACTION__
#define __DEFORM_ACTION__

#include <ros/ros.h>
#include <sarafun_tree/ExecuteAction.h>
#include <sarafun_msgs/DeformationKeyframeAction.h>

namespace sarafun {
class DeformAction
    : ExecuteAction<sarafun_msgs::DeformationKeyframeAction,
                    sarafun_msgs::DeformationKeyframeGoal> {
public:
  DeformAction(std::string node_name, std::string action_name,
                        std::string bt_name)
      : ExecuteAction<
            sarafun_msgs::DeformationKeyframeAction,
            sarafun_msgs::DeformationKeyframeGoal>::ExecuteAction(node_name,
                                                               action_name,
                                                               bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~DeformAction() {}

  bool fillGoal(sarafun_msgs::DeformationKeyframeGoal &goal);
  double getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
#endif
