#ifndef __ALIGN_ACTION__
#define __ALIGN_ACTION__

#include <ros/ros.h>
#include <sarafun_tree/ExecuteAction.h>
#include <sarafun_msgs/AlignKeyframeAction.h>

namespace sarafun {
class AlignAction
    : ExecuteAction<sarafun_msgs::AlignKeyframeAction,
                    sarafun_msgs::AlignKeyframeGoal> {
public:
  AlignAction(std::string node_name, std::string action_name,
                        std::string bt_name)
      : ExecuteAction<
            sarafun_msgs::AlignKeyframeAction,
            sarafun_msgs::AlignKeyframeGoal>::ExecuteAction(node_name,
                                                               action_name,
                                                               bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~AlignAction() {}

  bool fillGoal(sarafun_msgs::AlignKeyframeGoal &goal);
  double getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
#endif
