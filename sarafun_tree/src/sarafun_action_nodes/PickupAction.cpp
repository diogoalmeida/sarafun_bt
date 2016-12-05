#include <sarafun_tree/sarafun_action_nodes/PickupAction.h>

namespace sarafun {
bool PickupAction::fillGoal(sarafun_msgs::PickUpKeyframeGoal &goal) {
  return true;
}

double PickupAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt/bt_action_nodes/pickup", 30.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "PickupAction");
  sarafun::PickupAction pickup_action(
      ros::this_node::getName(), "/sarafun/bt_action_nodes/pickup/name",
      "pickup_action");
  ros::spin();
  return 1;
}
