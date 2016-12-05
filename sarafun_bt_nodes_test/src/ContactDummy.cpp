#include <sarafun_bt_nodes_test/ContactDummy.h>

namespace sarafun {
bool TestContact::parseGoal(const sarafun_msgs::ContactKeyframeGoalConstPtr &goal) {
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "contact_action_test");
  sarafun::TestContact action(ros::this_node::getName(),
                              "/sarafun/bt_nodes_test/contact");
  ros::spin();
  return -1;
}
