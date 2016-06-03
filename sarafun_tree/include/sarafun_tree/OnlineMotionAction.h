#ifndef __ONLINE_MOTION_ACTION__
#define __ONLINE_MOTION_ACTION__

#include <ros/ros.h>
#include <sarafun_tree/ExecuteAction.h>
#include <sarafun_hqp_omg/OnlineMotionAction.h>

namespace sarafun {
class OnlineMotionAction : ExecuteAction<sarafun_hqp_omg::OnlineMotionAction,
                                       sarafun_hqp_omg::OnlineMotionGoal> {
public:
  OnlineMotionAction(std::string node_name, std::string action_name,
                   std::string bt_name)
      : ExecuteAction<
            sarafun_hqp_omg::OnlineMotionAction,
            sarafun_hqp_omg::OnlineMotionGoal>::ExecuteAction(node_name,
                                                           action_name,
                                                           bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~OnlineMotionAction() {}

  bool fillGoal(sarafun_hqp_omg::OnlineMotionGoal &goal);
  double getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
#endif
