#include <sarafun_tree/OnlineMotionAction.h>

namespace sarafun {
double OnlineMotionAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt/online_motion/timeout", 30.0, timeout);

  return timeout;
}

bool OnlineMotionAction::fillGoal(sarafun_hqp_omg::OnlineMotionGoal &goal) {
	double x = 0, y = 0, z = 0;
	
	ros::param::get(ros::this_node::getName() + "/position/x", x);
  	ros::param::get(ros::this_node::getName() + "/position/y", y);
	ros::param::get(ros::this_node::getName() + "/position/z", z);

	goal.ref.position.x = x;
	goal.ref.position.y = y;
	goal.ref.position.z = z;

	goal.ref.orientation.x = 0;
	goal.ref.orientation.y = 0;
	goal.ref.orientation.z = 0;
	goal.ref.orientation.w = 1;

	return true;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "online_motion_action");

  sarafun::OnlineMotionAction online_motion_action(
      ros::this_node::getName(), "/sarafun/motion/online", ros::this_node::getName());

  ROS_INFO("Started %s!", ros::this_node::getName().c_str());
  ros::spin();
  return 1;
}
