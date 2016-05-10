#include <sarafun_manipulation/GrabDummy.h>

namespace sarafun {
bool TestGrab::parseGoal(const sarafun_manipulation::GrabGoalConstPtr &goal) {
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "grab_action_test");
  sarafun::TestGrab action(ros::this_node::getName(),
                           "/sarafun/manipulation/grab");
  return -1;
}
