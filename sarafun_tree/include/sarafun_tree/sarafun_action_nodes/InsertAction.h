#ifndef __INSERT_ACTION__
#define __INSERT_ACTION__

#include <ros/ros.h>
#include <sarafun_tree/ExecuteAction.h>
#include <sarafun_msgs/InsertionKeyframeAction.h>

namespace sarafun {
class InsertAction
    : ExecuteAction<sarafun_msgs::InsertionKeyframeAction,
                    sarafun_msgs::InsertionKeyframeGoal> {
public:
  InsertAction(std::string node_name, std::string action_name,
                        std::string bt_name)
      : ExecuteAction<
            sarafun_msgs::InsertionKeyframeAction,
            sarafun_msgs::InsertionKeyframeGoal>::ExecuteAction(node_name,
                                                               action_name,
                                                               bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~InsertAction() {}

  bool fillGoal(sarafun_msgs::InsertionKeyframeGoal &goal);
  double getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
#endif
