#include <sarafun_tree/ApproachObjectsAction.h>

namespace sarafun {
bool ApproachObjectsAction::fillGoal(sarafun_manipulation::ApproachGoal &goal) {
  return true;
}

double ApproachObjectsAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt/approach_objects/timeout", 30.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ApproachObjects");
  sarafun::ApproachObjectsAction approach_action(
      ros::this_node::getName(), "/sarafun/manipulation/approach",
      "approach_objects_action");
  ros::spin();
  return 1;
}
