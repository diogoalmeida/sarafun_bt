#ifndef __PICKUP_ACTION__
#define __PICKUP_ACTION__

#include <ros/ros.h>
#include <sarafun_tree/ExecuteAction.h>
#include <sarafun_msgs/PickUpKeyframeAction.h>

namespace sarafun {
class PickupAction
    : ExecuteAction<sarafun_msgs::PickUpKeyframeAction,
                    sarafun_msgs::PickUpKeyframeGoal> {
public:
  PickupAction(std::string node_name, std::string action_name,
                        std::string bt_name)
      : ExecuteAction<
            sarafun_msgs::PickUpKeyframeAction,
            sarafun_msgs::PickUpKeyframeGoal>::ExecuteAction(node_name,
                                                               action_name,
                                                               bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~PickupAction() {}

  bool fillGoal(sarafun_msgs::PickUpKeyframeGoal &goal);
  double getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
#endif
