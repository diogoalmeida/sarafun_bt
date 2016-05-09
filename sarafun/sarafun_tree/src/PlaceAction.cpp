#include <sarafun_tree/PlaceAction.h>

namespace sarafun {
double PlaceAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt/place/timeout", 30.0, timeout);

  return timeout;
}

bool PlaceAction::fillGoal(sarafun_manipulation::PlaceGoal &goal) {
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Place");
  sarafun::PlaceAction place_action(ros::this_node::getName(), "place_al",
                                    "place_action");
  ros::spin();
  return 1;
}
