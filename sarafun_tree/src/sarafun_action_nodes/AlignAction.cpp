#include <sarafun_tree/sarafun_action_nodes/AlignAction.h>

namespace sarafun {
bool AlignAction::fillGoal(sarafun_msgs::AlignKeyframeGoal &goal) {
  return true;
}

double AlignAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt_action_nodes/align/timeout", 30.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "AlignAction");
  std::string bt_client_name, action_server_name;

  if (!ros::param::get("/sarafun/bt_action_nodes/align/action_server_name", action_server_name))
  {
    ROS_ERROR("%s missing action server name", ros::this_node::getName().c_str());
    return -1;
  }

  if (!ros::param::get("/sarafun/bt_action_nodes/align/bt_client_name", bt_client_name))
  {
    ROS_ERROR("%s missing bt client name", ros::this_node::getName().c_str());
    return -1;
  }

  sarafun::AlignAction align_action(
      ros::this_node::getName(), action_server_name, bt_client_name);
  ros::spin();
  return 1;
}
