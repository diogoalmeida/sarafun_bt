#include <sarafun_tree/OnlineMotionAction.h>

namespace sarafun {
double OnlineMotionAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt/online_motion/timeout", 30.0, timeout);

  return timeout;
}

bool OnlineMotionAction::fillGoal(sarafun_hqp_omg::OnlineMotionGoal &goal) {
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_action");
  sarafun::OnlineMotionAction move_action(
      ros::this_node::getName(), "/sarafun/motion/online", "move_action");
  ros::spin();
  return 1;
}
