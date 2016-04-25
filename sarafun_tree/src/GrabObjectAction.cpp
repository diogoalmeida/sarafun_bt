#include <sarafun_tree/GrabObjectAction.h>

namespace sarafun {
void GrabObjectAction::preemptionRoutine() {
  ROS_WARN("The node %s was preempted", node_name_.c_str());
}

int GrabObjectAction::executionRoutine() {
  // Get object to grab
  // execute Code
  // return status
  return 1;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "GraspObject");
  sarafun::GrabObjectAction grab_action(ros::this_node::getName(),
                                        "grasp_object_action");
  ros::spin();
  return 1;
}
