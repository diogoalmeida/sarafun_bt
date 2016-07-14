#include <sarafun_tree/demo_bt_nodes/GrabObjectAction.h>

namespace sarafun {
double GrabObjectAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt/grab_object/timeout", 30.0, timeout);

  return timeout;
}
bool GrabObjectAction::fillGoal(sarafun_manipulation::GrabGoal &goal) {
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "GrabObject");
  sarafun::GrabObjectAction grab_action(ros::this_node::getName(),
                                        "/sarafun/manipulation/grab",
                                        "grab_object_action");
  ros::spin();
  return 1;
}
