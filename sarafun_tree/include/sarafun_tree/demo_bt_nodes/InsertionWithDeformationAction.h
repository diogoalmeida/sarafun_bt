#ifndef __INSERTION_WITH_DEFORMATION_ACTION__
#define __INSERTION_WITH_DEFORMATION_ACTION__

#include <ros/ros.h>
#include <sarafun_tree/ExecuteAction.h>
#include <sarafun_assembly/InsertionAction.h>

namespace sarafun {
class InsertionWithDeformationAction
    : ExecuteAction<sarafun_assembly::InsertionAction,
                    sarafun_assembly::InsertionGoal> {
public:
  InsertionWithDeformationAction(std::string node_name, std::string action_name,
                                 std::string bt_name)
      : ExecuteAction<
            sarafun_assembly::InsertionAction,
            sarafun_assembly::InsertionGoal>::ExecuteAction(node_name,
                                                            action_name,
                                                            bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~InsertionWithDeformationAction() {}

  bool fillGoal(sarafun_assembly::InsertionGoal &goal);
  double getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
#endif
