#include <sarafun_tree/sarafun_action_nodes/InsertAction.h>

namespace sarafun {
bool InsertAction::fillGoal(sarafun_msgs::InsertionKeyframeGoal &goal) {
  int idx;

  fillParameter("/sarafun/insert/idx", 0, idx);
  goal.idx = idx;

  return true;
}

double InsertAction::getTimeoutValue() {
  double timeout = 0;
  fillParameter("/sarafun/bt_action_nodes/insert/timeout", 30.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "InsertAction");
  std::string bt_client_name, action_server_name;

  if (!ros::param::get("/sarafun/bt_action_nodes/insert/action_server_name", action_server_name))
  {
    ROS_ERROR("%s missing action server name", ros::this_node::getName().c_str());
    return -1;
  }

  if (!ros::param::get("/sarafun/bt_action_nodes/insert/bt_client_name", bt_client_name))
  {
    ROS_ERROR("%s missing bt client name", ros::this_node::getName().c_str());
    return -1;
  }

  sarafun::InsertAction insert_action(
      ros::this_node::getName(),  action_server_name, bt_client_name);
  ros::spin();
  return 1;
}
