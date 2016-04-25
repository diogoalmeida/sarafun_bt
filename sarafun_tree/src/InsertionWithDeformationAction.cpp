#include <sarafun_tree/InsertionWithDeformationAction.h>

namespace sarafun {
void InsertionWithDeformationAction::preemptionRoutine() {
  ROS_WARN("The node %s was preempted", node_name_.c_str());
}

int InsertionWithDeformationAction::executionRoutine() {
  // Get object to grab
  // execute Code
  // return status
  return 1;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "InsertionWithDeformation");
  sarafun::InsertionWithDeformationAction grab_action(
      ros::this_node::getName(), "insertion_with_deformation_action");
  ros::spin();
  return 1;
}
