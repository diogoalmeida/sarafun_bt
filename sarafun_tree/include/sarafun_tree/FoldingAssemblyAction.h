#ifndef __FOLDING_ASSEMBLY_ACTION__
#define __FOLDING_ASSEMBLY_ACTION__
#include <ros/ros.h>
#include <sarafun_tree/ExecuteAction.h>
#include <sarafun_assembly/FoldingAction.h>

namespace sarafun {
using sarafun_assembly;
class FoldingAssemblyAction : ExecuteAction<FoldingAction, FoldingGoal> {
public:
  FoldingAssemblyAction(std::string node_name, std::string action_name, bt_name)
      : ExecuteAction<FoldingAction, FoldingGoal>(node_name, action_name,
                                                  bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~FoldingAssemblyAction() {}

  bool fillGoal(FoldingGoal &goal);
  int getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
#endif
