#include <sarafun_bt_nodes_test/PickupDummy.h>

namespace sarafun {
bool TestPickUp::parseGoal(const sarafun_msgs::PickUpKeyframeGoalConstPtr &goal) {
  ROS_INFO("%s got goal idx: %d", action_name_.c_str(), goal->idx);
  return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pickup_action_test");
  sarafun::TestPickUp action(ros::this_node::getName(),
                              "/sarafun/bt_nodes_test/pickup");
  ros::spin();
  return -1;
}
