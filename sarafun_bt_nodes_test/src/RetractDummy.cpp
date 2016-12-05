#include <sarafun_bt_nodes_test/RetractDummy.h>

namespace sarafun {
bool TestRetract::parseGoal(const sarafun_msgs::RetractKeyframeGoalConstPtr &goal) {
  ROS_INFO("%s got goal idx: %d", action_name_.c_str(), goal->idx);
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "retract_action_test");
  sarafun::TestRetract action(ros::this_node::getName(),
                              "/sarafun/bt_nodes_test/retract");
  ros::spin();
  return -1;
}
