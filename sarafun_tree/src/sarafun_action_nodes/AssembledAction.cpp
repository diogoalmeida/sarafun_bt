#include <sarafun_tree/sarafun_action_nodes/AssembledAction.h>

namespace sarafun {
bool AssembledAction::fillGoal(sarafun_msgs::AssembledKeyframeGoal &goal) {
  return true;
}

double AssembledAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt_action_nodes/assembled/timeout", 30.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "AssembledAction");
  sarafun::AssembledAction assembled_action(
      ros::this_node::getName(), "/sarafun/bt_action_nodes/assembled/name",
      "align_action");
  ros::spin();
  return 1;
}
