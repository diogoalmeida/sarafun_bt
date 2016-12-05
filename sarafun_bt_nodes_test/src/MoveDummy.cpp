#include <sarafun_bt_nodes_test/MoveDummy.h>

namespace sarafun {
bool TestMove::parseGoal(const sarafun_msgs::MoveKeyframeGoalConstPtr &goal) {
  ROS_INFO("%s got goal idx: %d", action_name_.c_str(), goal->idx);
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "move_action_test");
  sarafun::TestMove action(ros::this_node::getName(),
                              "/sarafun/bt_nodes_test/move");
  ros::spin();
  return -1;
}
