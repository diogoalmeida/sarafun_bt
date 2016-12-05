#ifndef __INITIAL_ACTION__
#define __INITIAL_ACTION__

#include <ros/ros.h>
#include <sarafun_tree/ExecuteAction.h>
#include <sarafun_msgs/InitialKeyframeAction.h>

namespace sarafun {
class InitialAction
    : ExecuteAction<sarafun_msgs::InitialKeyframeAction,
                    sarafun_msgs::InitialKeyframeGoal> {
public:
  InitialAction(std::string node_name, std::string action_name,
                        std::string bt_name)
      : ExecuteAction<
            sarafun_msgs::InitialKeyframeAction,
            sarafun_msgs::InitialKeyframeGoal>::ExecuteAction(node_name,
                                                               action_name,
                                                               bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~InitialAction() {}

  bool fillGoal(sarafun_msgs::InitialKeyframeGoal &goal);
  double getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
#endif
