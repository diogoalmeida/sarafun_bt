#include <sarafun_assembly/FoldingDummy.h>

namespace sarafun {
bool TestFolding::parseGoal(const sarafun_assembly::FoldingGoalConstPtr &goal) {
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "folding_action_test");
  sarafun::TestFolding action(ros::this_node::getName(),
                              "/sarafun/assembly/folding");
  ros::spin();
  return -1;
}
