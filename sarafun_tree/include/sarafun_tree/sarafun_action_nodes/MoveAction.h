#ifndef __MOVE_ACTION__
#define __MOVE_ACTION__

#include <ros/ros.h>
#include <sarafun_tree/ExecuteAction.h>
#include <sarafun_msgs/MoveKeyframeAction.h>

namespace sarafun {
class MoveAction
    : ExecuteAction<sarafun_msgs::MoveKeyframeAction,
                    sarafun_msgs::MoveKeyframeGoal> {
public:
  MoveAction(std::string node_name, std::string action_name,
                        std::string bt_name)
      : ExecuteAction<
            sarafun_msgs::MoveKeyframeAction,
            sarafun_msgs::MoveKeyframeGoal>::ExecuteAction(node_name,
                                                               action_name,
                                                               bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~MoveAction() {}

  bool fillGoal(sarafun_msgs::MoveKeyframeGoal &goal);
  double getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
#endif
