#include <sarafun_bt_nodes_test/InitialDummy.h>

namespace sarafun {
bool TestInitial::parseGoal(const sarafun_msgs::InitialKeyframeGoalConstPtr &goal) {
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "initial_action_test");
  sarafun::TestInitial action(ros::this_node::getName(),
                              "/sarafun/bt_nodes_test/initial");
  ros::spin();
  return -1;
}
