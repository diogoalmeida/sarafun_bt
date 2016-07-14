#include <ros/ros.h>
#include <behavior_tree_core/BehaviorTree.h>
#include <behavior_tree_core/ControlNode.h>
#include <sarafun_tree/demo_bt_nodes/ApproachObjectsAction.h>
#include <sarafun_tree/demo_bt_nodes/GrabObjectAction.h>
#include <sarafun_tree/demo_bt_nodes/FoldingAssemblyAction.h>
#include <sarafun_tree/demo_bt_nodes/InsertionWithDeformationAction.h>
#include <sarafun_tree/demo_bt_nodes/PlaceAction.h>
#include <sarafun_tree/parse_tree.h>

using namespace sarafun;
using namespace BT;

int main(int argc, char **argv) {
  ros::init(argc, argv, "sarafun_bt_demo");
  std::string filename;
  int TickPeriod_milliseconds = 0;

  if (ros::param::has("/sarafun/bt/file")) {
    ros::param::get("/sarafun/bt/file", filename);
  } else {
    filename = std::string("example.json");
  }

  if (ros::param::has("/sarafun/bt/tick_period")){
	ros::param::get("/sarafun/bt/tick_period", TickPeriod_milliseconds);
  } else {
    TickPeriod_milliseconds = 1000;
  }

  std::string path =
      ros::package::getPath("sarafun_tree") + "/data/" + filename;
  bt_parser::Parser parser(path);
  try {
    ControlNode *root = dynamic_cast<ControlNode *>(parser.parseTree());

    if (root != nullptr) {
      Execute(root, TickPeriod_milliseconds);
    }
  } catch (BehaviorTreeException &Exception) {
    ROS_ERROR("%s", Exception.what());
  }

  return 0;
}
