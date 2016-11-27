#include <ros/ros.h>
#include <sarafun_tree/demo_bt_nodes/ApproachObjectsAction.h>
#include <sarafun_tree/demo_bt_nodes/GrabObjectAction.h>
#include <sarafun_tree/demo_bt_nodes/FoldingAssemblyAction.h>
#include <sarafun_tree/demo_bt_nodes/InsertionWithDeformationAction.h>
#include <sarafun_tree/demo_bt_nodes/PlaceAction.h>
#include <sarafun_tree/TreeRunner.h>
#include <std_srvs/Empty.h>

using namespace sarafun;
using namespace BT;

TreeRunner *runner;
std::string filename;

/*
  Starts behavior tree execution
*/
bool startTreeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &ans)
{
  runner->startTree(filename);
}

/*
  Stops behavior tree execution
*/
bool stopTreeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &ans)
{
  runner->stopTree();
}

/*
  Restarts behavior tree execution
*/
bool restartTreeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &ans)
{
  runner->stopTree();
  runner->startTree(filename);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sarafun_bt_demo");
  ros::NodeHandle n;
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

  ros::ServiceServer bt_start_service = n.advertiseService("/sarafun/start_tree", startTreeCallback);
  ros::ServiceServer bt_stop_service = n.advertiseService("/sarafun/stop_tree", stopTreeCallback);
  ros::ServiceServer bt_restart_service = n.advertiseService("/sarafun/restart_tree", restartTreeCallback);

  runner = new TreeRunner(TickPeriod_milliseconds);
  ros::spin();

  return 0;
}
