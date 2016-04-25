#include <sarafun_tree/PlaceAction.h>

namespace sarafun {
void PlaceAction::preemptionRoutine() {
  ROS_WARN("The node %s was preempted", node_name_.c_str());
}

int PlaceAction::executionRoutine() {
  // Get object to grab
  // execute Code
  // return status
  return 1;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Place");
  sarafun::PlaceAction grab_action(ros::this_node::getName(), "place_action");
  ros::spin();
  return 1;
}
