#ifndef __PLACE_ACTION__
#define __PLACE_ACTION__

#include <ros/ros.h>
#include <sarafun_tree/ExecuteAction.h>
#include <sarafun_manipulation/PlaceAction.h>

namespace sarafun {
class PlaceAction : ExecuteAction<sarafun_manipulation::PlaceAction,
                                  sarafun_manipulation::PlaceGoal> {
public:
  PlaceAction(std::string node_name, std::string action_name,
              std::string bt_name)
      : ExecuteAction<
            sarafun_manipulation::PlaceAction,
            sarafun_manipulation::PlaceGoal>::ExecuteAction(node_name,
                                                            action_name,
                                                            bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~PlaceAction() {}

  bool fillGoal(sarafun_manipulation::PlaceGoal &goal);
  double getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
#endif
