#include <sarafun_tree/sarafun_action_nodes/InitialAction.h>

namespace sarafun {
bool InitialAction::fillGoal(sarafun_msgs::InitialKeyframeGoal &goal) {
  return true;
}

double InitialAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt_action_nodes/initial/timeout", 30.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "InitialAction");
  sarafun::InitialAction initial_action(
      ros::this_node::getName(), "/sarafun/bt_action_nodes/initial/name",
      "initial_action");
  ros::spin();
  return 1;
}
