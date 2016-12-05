#ifndef __CONTACT_ACTION__
#define __CONTACT_ACTION__

#include <ros/ros.h>
#include <sarafun_tree/ExecuteAction.h>
#include <sarafun_msgs/ContactKeyframeAction.h>

namespace sarafun {
class ContactAction
    : ExecuteAction<sarafun_msgs::ContactKeyframeAction,
                    sarafun_msgs::ContactKeyframeGoal> {
public:
  ContactAction(std::string node_name, std::string action_name,
                        std::string bt_name)
      : ExecuteAction<
            sarafun_msgs::ContactKeyframeAction,
            sarafun_msgs::ContactKeyframeGoal>::ExecuteAction(node_name,
                                                               action_name,
                                                               bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~ContactAction() {}

  bool fillGoal(sarafun_msgs::ContactKeyframeGoal &goal);
  double getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
#endif
