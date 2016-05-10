#include <sarafun_manipulation/PlaceDummy.h>

namespace sarafun {
bool TestPlace::parseGoal(const sarafun_manipulation::PlaceGoalConstPtr &goal) {
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "place_action_test");
  sarafun::TestPlace action(ros::this_node::getName(),
                            "/sarafun/manipulation/place");
  return -1;
}
