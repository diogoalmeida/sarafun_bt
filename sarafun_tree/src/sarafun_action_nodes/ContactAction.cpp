#include <sarafun_tree/sarafun_action_nodes/ContactAction.h>

namespace sarafun {
bool ContactAction::fillGoal(sarafun_msgs::ContactKeyframeGoal &goal) {
  return true;
}

double ContactAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt_action_nodes/contact/timeout", 30.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ContactAction");
  sarafun::ContactAction contact_action(
      ros::this_node::getName(), "/sarafun/bt_action_nodes/contact/name",
      "contact_action");
  ros::spin();
  return 1;
}
