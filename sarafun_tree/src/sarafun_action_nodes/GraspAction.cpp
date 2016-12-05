#include <sarafun_tree/sarafun_action_nodes/GraspAction.h>

namespace sarafun {
bool GraspAction::fillGoal(sarafun_msgs::GraspKeyframeGoal &goal) {
  return true;
}

double GraspAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt_action_nodes/grasp/timeout", 30.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "GraspAction");
  sarafun::GraspAction grasp_action(
      ros::this_node::getName(), "/sarafun/bt_action_nodes/grasp/name",
      "grasp_action");
  ros::spin();
  return 1;
}
