#include <sarafun_bt_nodes_test/AssembledDummy.h>

namespace sarafun {
bool TestAssembled::parseGoal(const sarafun_msgs::AssembledKeyframeGoalConstPtr &goal) {
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "assembled_action_test");
  sarafun::TestAssembled action(ros::this_node::getName(),
                              "/sarafun/bt_nodes_test/assembled");
  ros::spin();
  return -1;
}
