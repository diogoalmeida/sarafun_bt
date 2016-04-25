#include <sarafun_tree/ApproachObjectsAction.h>

namespace sarafun {
void ApproachObjectsAction::preemptionRoutine() {
  ROS_WARN("The node %s was preempted", node_name_.c_str());
}

int ApproachObjectsAction::executionRoutine() {
  // Get object to grab
  // execute Code
  // return status
  return 1;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ApproachObjects");
  sarafun::ApproachObjectsAction grab_action(ros::this_node::getName(),
                                             "approach_objects_action");
  ros::spin();
  return 1;
}
