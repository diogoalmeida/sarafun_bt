#include <sarafun_tree/demo_bt_nodes/IsSimple.h>

namespace sarafun {

bool IsSimple::isSystemActive() {
  bool running;
  if (nh_.hasParam("/sarafun/bt/running")) {
    nh_.getParam("/sarafun/bt/running", running);

    if (running) {
      return true;
    }
  } else {
    ROS_WARN("The parameter /sarafun/bt/running must be set"
             "for a BT action to run!");
  }
  return false;
}

bool IsSimple::conditionEvaluation() {

  if (isSystemActive()) {
      counter_++;
  }

  ROS_INFO("%s: counter is %d", bt_name_.c_str(), counter_);

  if (counter_ < condition_limit_)
  {
  	return true;
  }

  return false;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "IsSimple");
  sarafun::IsSimple is_simple(ros::this_node::getName(),
							  "IsSimple");
  ros::spin();
  return 1;
}
