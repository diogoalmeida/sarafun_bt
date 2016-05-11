#include <sarafun_assembly/InsertionDummy.h>

namespace sarafun {
bool TestInsertion::parseGoal(
    const sarafun_assembly::InsertionGoalConstPtr &goal) {
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "insertion_action_test");
  sarafun::TestInsertion action(ros::this_node::getName(),
                                "/sarafun/assembly/insertion");
  ros::spin();

  return -1;
}
