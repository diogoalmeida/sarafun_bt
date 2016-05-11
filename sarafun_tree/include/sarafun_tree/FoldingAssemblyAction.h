#ifndef __FOLDING_ASSEMBLY_ACTION__
#define __FOLDING_ASSEMBLY_ACTION__

#include <ros/ros.h>
#include <sarafun_tree/ExecuteAction.h>
#include <sarafun_assembly/FoldingAction.h>

namespace sarafun {
class FoldingAssemblyAction : ExecuteAction<sarafun_assembly::FoldingAction,
                                            sarafun_assembly::FoldingGoal> {
public:
  FoldingAssemblyAction(std::string node_name, std::string action_name,
                        std::string bt_name)
      : ExecuteAction<sarafun_assembly::FoldingAction,
                      sarafun_assembly::FoldingGoal>::ExecuteAction(node_name,
                                                                    action_name,
                                                                    bt_name),
        node_name_(node_name), bt_name_(bt_name) {
    node_handle_ = ros::NodeHandle(node_name);
  }
  ~FoldingAssemblyAction() {}

  bool fillGoal(sarafun_assembly::FoldingGoal &goal);
  double getTimeoutValue();

private:
  ros::NodeHandle node_handle_;
  std::string node_name_;
  std::string bt_name_;
};
}
#endif
