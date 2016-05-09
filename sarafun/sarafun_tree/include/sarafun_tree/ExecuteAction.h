#ifndef __EXECUTE_ACTION__
#define __EXECUTE_ACTION__

#include <ros/ros.h>
#include <behavior_tree_leaves/ActionTemplate.h>
#include <actionlib/client/simple_action_client.h>

namespace sarafun {
/**
  This class implements an interface for BT actions destined to call a generic
  rosaction server.
*/
template <class ActionClass, class ActionGoal>
class ExecuteAction : ActionTemplate {
public:
  /*
    Contructor receives:

    node_name: The ROS node name
    actionlib_name: The actionlib server name that this action will call
    bt_name: The name defined in the "name" tag from the BT json input file
  */
  ExecuteAction(std::string node_name, std::string actionlib_name,
                std::string bt_name);
  virtual ~ExecuteAction();

  int executionRoutine();
  void preemptionRoutine();

protected:
  /*
    Fills in the goal for a particular action.
  */
  virtual bool fillGoal(ActionGoal &goal) = 0;

  /*
    Provides the client with a timeout value (in seconds) for actionlib
    connections
  */
  virtual double getTimeoutValue() = 0;

  /*
      Fills in a generic parameter from the parameter server
  */
  bool fillParameter(std::string param_name, std::string &param_val);
  void fillParameter(std::string param_name, std::string def,
                     std::string &param_val);
  void fillParameter(std::string param_name, double def, double &param_val);

  /*
      Checks if BT action execution is allowed
  */
  bool isSystemActive();

private:
  actionlib::SimpleActionClient<ActionClass> *action_client_;
  ActionGoal goal_;
  std::string action_name_;
};

// Template classes "must" be implemented in the header files... -->
// http://stackoverflow.com/questions/495021/why-can-templates-only-be-implemented-in-the-header-file

template <class ActionClass, class ActionGoal>
ExecuteAction<ActionClass, ActionGoal>::ExecuteAction(
    std::string node_name, std::string actionlib_name, std::string bt_name)
    : ActionTemplate::ActionTemplate(bt_name) {
  nh_ = ros::NodeHandle("~");
  action_name_ = actionlib_name;
}

template <class ActionClass, class ActionGoal>
ExecuteAction<ActionClass, ActionGoal>::~ExecuteAction() {
  delete action_client_;
}

template <class ActionClass, class ActionGoal>
bool ExecuteAction<ActionClass, ActionGoal>::isSystemActive() {
  bool running;
  if (nh_.hasParam("/sarafun/bt/running")) {
    nh_.getParam("/sarafun/bt/running", running);

    if (running) {
      return true;
    }
  } else {
    ROS_ERROR("The parameter /sarafun/bt/running must be set"
              "for a BT action to run!");
  }
  return false;
}

template <class ActionClass, class ActionGoal>
void ExecuteAction<ActionClass, ActionGoal>::preemptionRoutine() {
  ROS_WARN("The node %s was preempted", action_name_.c_str());
}

template <class ActionClass, class ActionGoal>
int ExecuteAction<ActionClass, ActionGoal>::executionRoutine() {
  action_client_ =
      new actionlib::SimpleActionClient<ActionClass>(action_name_, true);

  if (!isSystemActive()) {
    return 0; // Keep running
  }

  bool active_server = action_client_->waitForServer(ros::Duration(2.0));

  if (!active_server) {
    ROS_ERROR("Actionlib server failed to start for action %s!",
              action_name_.c_str());
    return -1; // Failure
  }

  bool has_parameters = fillGoal(goal_);

  if (!has_parameters) {
    ROS_ERROR("Failed to get parameters for action %s!", action_name_.c_str());
    return -1; // Failure
  }

  ROS_INFO("Sending goal from action: %s. Timeout value: %.2f",
           action_name_.c_str(), getTimeoutValue());
  action_client_->sendGoal(goal_);
  bool finished =
      action_client_->waitForResult(ros::Duration(getTimeoutValue()));

  if (finished) {
    action_client_->cancelAllGoals(); // To be safe
    actionlib::SimpleClientGoalState state = action_client_->getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());

    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      action_client_->getResult();
      return 1;
    } else {
      return -1;
    }
  } else {
    ROS_ERROR("Action %s timeout! Cancelling goals...", action_name_.c_str());

    return -1;
  }
}

template <class ActionClass, class ActionGoal>
bool ExecuteAction<ActionClass, ActionGoal>::fillParameter(
    std::string param_name, std::string &param_val) {
  if (nh_.hasParam(param_name.c_str())) {
    nh_.getParam(param_name.c_str(), param_val);
  } else {
    ROS_ERROR("%s not set!", param_name.c_str());
    return false;
  }
  return true;
}

template <class ActionClass, class ActionGoal>
void ExecuteAction<ActionClass, ActionGoal>::fillParameter(
    std::string param_name, std::string def, std::string &param_val) {
  if (nh_.hasParam(param_name.c_str())) {
    nh_.getParam(param_name.c_str(), param_val);
  } else {
    ROS_WARN("Param '%s' not set. Using default: %s", param_name.c_str(),
             def.c_str());
  }
}

template <class ActionClass, class ActionGoal>
void ExecuteAction<ActionClass, ActionGoal>::fillParameter(
    std::string param_name, double def, double &param_val) {
  if (nh_.hasParam(param_name.c_str())) {
    nh_.getParam(param_name.c_str(), param_val);
  } else {
    ROS_WARN("Param '%s' not set. Using default: %.2f", param_name.c_str(),
             def);
  }
}
}
#endif
