#include <sarafun_tree/sarafun_action_nodes/RetractAction.h>

namespace sarafun {
bool RetractAction::fillGoal(sarafun_msgs::RetractKeyframeGoal &goal) {
  return true;
}

double RetractAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt_action_nodes/retract/timeout", 30.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "RetractAction");
  sarafun::RetractAction retract_action(
      ros::this_node::getName(), "/sarafun/bt_action_nodes/retract/name",
      "align_action");
  ros::spin();
  return 1;
}
