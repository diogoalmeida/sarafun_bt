#include <sarafun_tree/FoldingAssemblyAction.h>

namespace sarafun {
void FoldingAssemblyAction::preemptionRoutine() {
  ROS_WARN("The node %s was preempted", node_name_.c_str());
}

int FoldingAssemblyAction::executionRoutine() {
  // Get object to grab
  // execute Code
  // return status
  return 1;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "FoldingAssembly");
  sarafun::FoldingAssemblyAction grab_action(ros::this_node::getName(),
                                             "folding_assembly_action");
  ros::spin();
  return 1;
}
