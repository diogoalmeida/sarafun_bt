#ifndef __GRAB_OBJECT_ACTION__
#define __GRAB_OBJECT_ACTION__

#include <ros/ros.h>
#include <sarafun_tree/ExecuteAction.h>
#include <sarafun_manipulation/GrabAction.h>

namespace sarafun {
class GrabObjectAction : ExecuteAction<sarafun_manipulation::GrabAction,
                                       sarafun_manipulation::GrabGoal> {
public:
  GrabObjectAction(std::string node_name, std::string action_name,
                   std::string bt_name)
      : ExecuteAction<
            sarafun_manipulation::GrabAction,
            sarafun_manipulation::GrabGoal>::ExecuteAction(node_name,
                                                           action_name,
                                                           bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~GrabObjectAction() {}

  bool fillGoal(sarafun_manipulation::GrabGoal &goal);
  double getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
#endif
