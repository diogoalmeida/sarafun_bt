#include <sarafun_bt_nodes_test/AlignDummy.h>

namespace sarafun {
bool TestAlign::parseGoal(const sarafun_msgs::AlignKeyframeGoalConstPtr &goal) {
  ROS_INFO("%s got goal idx: %d", action_name_.c_str(), goal->idx);
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "align_action_test");
  sarafun::TestAlign action(ros::this_node::getName(),
                              "/sarafun/bt_nodes_test/align");
  ros::spin();
  return -1;
}
