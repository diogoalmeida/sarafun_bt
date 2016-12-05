#include <sarafun_bt_nodes_test/GraspDummy.h>

namespace sarafun {
bool TestGrasp::parseGoal(const sarafun_msgs::GraspKeyframeGoalConstPtr &goal) {
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "grasp_action_test");
  sarafun::TestGrasp action(ros::this_node::getName(),
                              "/sarafun/bt_nodes_test/grasp");
  ros::spin();
  return -1;
}
