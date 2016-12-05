#include <sarafun_tree/sarafun_action_nodes/MoveAction.h>

namespace sarafun {
bool MoveAction::fillGoal(sarafun_msgs::MoveKeyframeGoal &goal) {
  return true;
}

double MoveAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt_action_nodes/move/timeout", 30.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "MoveAction");
  sarafun::MoveAction move_action(
      ros::this_node::getName(), "/sarafun/bt_action_nodes/move/name",
      "align_action");
  ros::spin();
  return 1;
}
