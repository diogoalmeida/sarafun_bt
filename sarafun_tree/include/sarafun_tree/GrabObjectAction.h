#ifndef __GRAB_OBJECT_ACTION__
#define __GRAB_OBJECT_ACTION__
#include <ros/ros.h>
#include <sarafun_tree/ExecuteAction.h>
#include <sarafun_manipulation/GrabAction.h>

namespace sarafun {
using sarafun_manipulation;
class GrabObjectAction : ExecuteAction<GrabAction, GrabGoal> {
public:
  GrabObjectAction(std::string node_name, std::string action_name, bt_name)
      : ExecuteAction<GrabAction, GrabGoal>(node_name, action_name, bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    bt_name_(bt_name) node_handle_ = ros::NodeHandle(node_name);
  }
  ~GrabObjectAction() {}

  bool fillGoal(GrabGoal &goal);
  int getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
