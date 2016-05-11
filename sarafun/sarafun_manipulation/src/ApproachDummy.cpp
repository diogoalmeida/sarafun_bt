#include <sarafun_manipulation/ApproachDummy.h>

namespace sarafun {
bool TestApproach::parseGoal(
    const sarafun_manipulation::ApproachGoalConstPtr &goal) {
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "approach_action_test");
  sarafun::TestApproach action(ros::this_node::getName(),
                               "/sarafun/manipulation/approach");
  ros::spin();
  return -1;
}
