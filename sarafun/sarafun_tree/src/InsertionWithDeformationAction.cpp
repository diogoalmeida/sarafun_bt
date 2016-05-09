#include <sarafun_tree/InsertionWithDeformationAction.h>

namespace sarafun {
double InsertionWithDeformationAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt/insertion_deformation/timeout", 30.0, timeout);

  return timeout;
}

bool InsertionWithDeformationAction::fillGoal(
    sarafun_assembly::InsertionGoal &goal) {
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "InsertionWithDeformation");
  sarafun::InsertionWithDeformationAction insertion_action(
      ros::this_node::getName(), "insertion_with_deformation_al",
      "insertion_with_deformation_action");
  ros::spin();
  return 1;
}
