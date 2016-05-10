#ifndef __GENERIC_TEST_SERVER__
#define __GENERIC_TEST_SERVER__
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

namespace sarafun {
/**
  This class implements an interface for generic test servers to be built.
  They will mimic the actual servers implemented by the remaining project
  workpackages.
*/
template <class ActionClass, class ActionGoalConstPtr, class ActionFeedback,
          class ActionResult>
class TestServer {
public:
  TestServer(std::string node_name, std::string actionlib_name);
  virtual ~TestServer() {}
  void executeCB(const ActionGoalConstPtr &goal);

protected:
  /*
      Should print goal contents on the screen and return true or false
     depending on
      if they are properly filled in or not.
  */
  virtual bool parseGoal(const ActionGoalConstPtr &goal) = 0;

private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<ActionClass> action_server_;
  std::string action_name_;
  ActionFeedback feedback_;
  ActionResult result_;
};

template <class ActionClass, class ActionGoalConstPtr, class ActionFeedback,
          class ActionResult>
TestServer<ActionClass, ActionGoalConstPtr, ActionFeedback,
           ActionResult>::TestServer(std::string node_name,
                                     std::string actionlib_name)
    : action_server_(nh_, actionlib_name,
                     boost::bind(&TestServer::executeCB, this, _1), false),
      action_name_(actionlib_name) {
  action_server_.start();
}

template <class ActionClass, class ActionGoalConstPtr, class ActionFeedback,
          class ActionResult>
void TestServer<ActionClass, ActionGoalConstPtr, ActionFeedback,
                ActionResult>::executeCB(const ActionGoalConstPtr &goal) {
  ROS_INFO("BT Test server %s has been called!", action_name_.c_str());
  if (!parseGoal(goal)) {
    action_server_.setPreempted();
    return;
  }

  ROS_INFO("Press 'p' for preempting this action, 'a' to abort, anything else "
           "to succeed");
  std::string input;
  char key;
  getline(std::cin, input);

  if (input.length() == 1) {
    key = input[0];
  } else {
    key = 's';
  }

  switch (key) {
  case 'p':
    ROS_WARN("Preempting!");
    action_server_.setPreempted();
    break;
  case 'a':
    ROS_ERROR("Aborting!");
    action_server_.setAborted();
    break;
  default:
    ROS_INFO("Succeeding!");
    action_server_.setSucceeded();
    break;
  }
}
}
#endif
