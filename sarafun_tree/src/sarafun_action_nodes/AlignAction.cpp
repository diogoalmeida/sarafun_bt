#include <sarafun_tree/sarafun_action_nodes/AlignAction.h>

namespace sarafun {
bool AlignAction::fillGoal(sarafun_msgs::AlignKeyframeGoal &goal) {
  return true;
}

double AlignAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt_action_nodes/align/timeout", 30.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "AlignAction");
  sarafun::AlignAction align_action(
      ros::this_node::getName(), "/sarafun/bt_action_nodes/align/name",
      "align_action");
  ros::spin();
  return 1;
}
