#ifndef __RETRACT_ACTION__
#define __RETRACT_ACTION__

#include <ros/ros.h>
#include <sarafun_tree/ExecuteAction.h>
#include <sarafun_msgs/RetractKeyframeAction.h>

namespace sarafun {
class RetractAction
    : ExecuteAction<sarafun_msgs::RetractKeyframeAction,
                    sarafun_msgs::RetractKeyframeGoal> {
public:
  RetractAction(std::string node_name, std::string action_name,
                        std::string bt_name)
      : ExecuteAction<
            sarafun_msgs::RetractKeyframeAction,
            sarafun_msgs::RetractKeyframeGoal>::ExecuteAction(node_name,
                                                               action_name,
                                                               bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~RetractAction() {}

  bool fillGoal(sarafun_msgs::RetractKeyframeGoal &goal);
  double getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
#endif
