#include <sarafun_tree/sarafun_action_nodes/MoveAction.h>

namespace sarafun {
bool MoveAction::fillGoal(sarafun_msgs::MoveKeyframeGoal &goal) {
  return true;
}

double MoveAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt_action_nodes/move/timeout", 30.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "MoveAction");
  std::string bt_client_name, action_server_name;

  if (!ros::param::get("/sarafun/bt_action_nodes/move/action_server_name", action_server_name))
  {
    ROS_ERROR("%s missing action server name", ros::this_node::getName().c_str());
    return -1;
  }

  if (!ros::param::get("/sarafun/bt_action_nodes/move/bt_client_name", bt_client_name))
  {
    ROS_ERROR("%s missing bt client name", ros::this_node::getName().c_str());
    return -1;
  }

  sarafun::MoveAction move_action(
      ros::this_node::getName(),  action_server_name, bt_client_name);
  ros::spin();
  return 1;
}
