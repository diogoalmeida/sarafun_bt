#include <sarafun_tree/sarafun_action_nodes/GraspAction.h>

namespace sarafun {
bool GraspAction::fillGoal(sarafun_msgs::GraspKeyframeGoal &goal) {
  return true;
}

double GraspAction::getTimeoutValue() {
  double timeout = 0;
  std::string ns = ros::this_node::getNamespace();
  ns=ns+"/";
  fillParameter("/sarafun/bt_action_nodes/"+ns+"grasp/timeout", 30.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "GraspAction");
  std::string bt_client_name, action_server_name;
  std::string ns = ros::this_node::getNamespace();
  ns=ns+"/";
  if (!ros::param::get("/sarafun/bt_action_nodes/"+ns+"grasp/action_server_name", action_server_name))
  {
    ROS_ERROR("%s missing action server name", ros::this_node::getName().c_str());
    return -1;
  }

  if (!ros::param::get("/sarafun/bt_action_nodes/"+ns+"grasp/bt_client_name", bt_client_name))
  {
    ROS_ERROR("%s missing bt client name", ros::this_node::getName().c_str());
    return -1;
  }

  sarafun::GraspAction grasp_action(
      ros::this_node::getName(), action_server_name, bt_client_name);
  ros::spin();
  return 1;
}
