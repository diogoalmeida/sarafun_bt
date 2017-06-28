#ifndef __EXECUTE_ACTION__
#define __EXECUTE_ACTION__

#include <ros/ros.h>
#include <behavior_tree_leaves/ActionTemplate.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>

namespace sarafun {
/**
  This class implements an interface for BT actions destined to call a generic
  rosaction server.

  It establishes a bridge between the behavior tree action class and
  an externally implemented action: BT <--> ExecuteAction <--> external implementation
**/
template <class ActionClass, class ActionGoal>
class ExecuteAction : ActionTemplate {
public:
  /**
    Contructor.

    @param node_name The ROS node name.
    @param actionlib_name The actionlib server name that this action will call.
    @param bt_name The name defined in the "name" tag from the BT JSON input file.
  **/
  ExecuteAction(std::string node_name, std::string actionlib_name,
                std::string bt_name);
  virtual ~ExecuteAction();

  int executionRoutine();
  void preemptionRoutine();

protected:
  /**
    Fills in the goal for a particular action.

    @param goal The actionlib goal message of the externally implemented action.
    @return False in case of error, true otherwise.
  **/
  virtual bool fillGoal(ActionGoal &goal) = 0;

  /**
    Provides the client with a timeout value for actionlib
    connections.

    @return The timeout value (in seconds).
  **/
  virtual double getTimeoutValue() = 0;

  /**
      Gets generic parameter from the parameter server.

      @param The parameter name.
      @param_val The parameter value.
      @return False in case the parameter does not exist, true otherise.
  **/
  bool fillParameter(std::string param_name, std::string &param_val);

  /**
      Gets a generic parameter from the parameter server.
      Sets it to a default in case the parameter does not exist.

      @param The parameter name
      @param def The default parameter value
      @param_val The parameter value
  **/
  void fillParameter(std::string param_name, std::string def,
                     std::string &param_val);
  void fillParameter(std::string param_name, double def, double &param_val);
  void fillParameter(std::string param_name, int def, int &param_val);

  /**
      Checks if BT action execution is allowed.

      @return True if the tree is active, false otherwise.
  **/
  bool isSystemActive();

private:
  actionlib::SimpleActionClient<ActionClass> *action_client_;
  ActionGoal goal_;
  std::string action_name_;
  ros::Time start_time_; // Current execution start time
  bool first_call_;
  boost::mutex action_mutex_;
};

// Template classes "must" be implemented in the header files... -->
// http://stackoverflow.com/questions/495021/why-can-templates-only-be-implemented-in-the-header-file

template <class ActionClass, class ActionGoal>
ExecuteAction<ActionClass, ActionGoal>::ExecuteAction(
    std::string node_name, std::string actionlib_name, std::string bt_name)
    : ActionTemplate::ActionTemplate(bt_name) {
  nh_ = ros::NodeHandle("~");
  action_name_ = actionlib_name;
  action_client_ = new actionlib::SimpleActionClient<ActionClass>(action_name_, true);
  ROS_WARN("Action %s is waiting for the corresponding actionlib server!", action_name_.c_str());
  bool active_server = action_client_->waitForServer(ros::Duration(10.0));

  if (!active_server)
  {
    ROS_ERROR("%s could not connect to %s", bt_name.c_str(), actionlib_name.c_str());
    nh_.shutdown();
  } 
  first_call_ = false;
}

template <class ActionClass, class ActionGoal>
ExecuteAction<ActionClass, ActionGoal>::~ExecuteAction() {
  // action_client_->cancelGoal(); // To be safe
  actionlib::SimpleClientGoalState goal_state = action_client_->getState();
  if (goal_state != actionlib::SimpleClientGoalState::ACTIVE)
  {
    action_client_->cancelGoal(); // To be safe
    first_call_ = true;
  }
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
    ROS_WARN("The parameter /sarafun/bt/running must be set"
             "for a BT action to run!");
  }
  return false;
}

template <class ActionClass, class ActionGoal>
void ExecuteAction<ActionClass, ActionGoal>::preemptionRoutine() {
  boost::lock_guard<boost::mutex> guard(action_mutex_);
  if (!first_call_)
  {
  ROS_WARN("Preempting node %s", action_name_.c_str());
  actionlib::SimpleClientGoalState goal_state = action_client_->getState();
  if (goal_state != actionlib::SimpleClientGoalState::ACTIVE)
  {
    action_client_->cancelGoal(); // To be safe
    first_call_ = true;  
  }
  ROS_WARN("The node %s was preempted", action_name_.c_str());
  }
  else
  {
    ROS_ERROR("Got preemption, but already destroyed action client in %s", action_name_.c_str());
  }
}

template <class ActionClass, class ActionGoal>
int ExecuteAction<ActionClass, ActionGoal>::executionRoutine() {
  boost::lock_guard<boost::mutex> guard(action_mutex_);
  if (first_call_ && isSystemActive()) // fresh call!
  {
    ROS_INFO("Action %s is waiting for the corresponding actionlib server!", action_name_.c_str());
    bool active_server = action_client_->waitForServer(ros::Duration(2.0));
    if (!active_server) {
      ROS_ERROR("Actionlib server failed to start for action %s!",
      action_name_.c_str());
      actionlib::SimpleClientGoalState goal_state = action_client_->getState();
      if (goal_state != actionlib::SimpleClientGoalState::ACTIVE)
      {
        action_client_->cancelGoal(); // To be safe
        first_call_ = true;  
      }
      
      return -1; // Failure
    }
    start_time_ = ros::Time::now();

    bool has_parameters = fillGoal(goal_);

    if (!has_parameters) {
      ROS_ERROR("Failed to get parameters for action %s!", action_name_.c_str());
      return -1; // Failure
    }

    ROS_INFO("Sending goal from action: %s. Timeout value: %.2f", action_name_.c_str(), getTimeoutValue());
    action_client_->sendGoal(goal_);
    first_call_ = false;
  }

  if (!isSystemActive()) {
    return 0; // Keep running
  }

  bool finished = false, timeout = false;
  if ((ros::Time::now() - start_time_).toSec() < getTimeoutValue())
  {
    actionlib::SimpleClientGoalState goal_state = action_client_->getState();
    if (goal_state != actionlib::SimpleClientGoalState::ACTIVE && goal_state != actionlib::SimpleClientGoalState::PENDING)
    {
      finished = true;
    }
  }
  else
  {
    timeout = true;
  }

  if (finished) {
    actionlib::SimpleClientGoalState goal_state = action_client_->getState();
    if (goal_state != actionlib::SimpleClientGoalState::ACTIVE)
    {
      action_client_->cancelGoal(); // To be safe
      first_call_ = true;  
    }
    actionlib::SimpleClientGoalState state = action_client_->getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());

    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      return 1;
    } else {
      return -1;
    }
  } else {
    if (timeout)
    {
      ROS_ERROR("Action %s timeout! Cancelling goals...", action_name_.c_str());
      actionlib::SimpleClientGoalState goal_state = action_client_->getState();
      if (goal_state != actionlib::SimpleClientGoalState::ACTIVE)
      {
        action_client_->cancelGoal(); // To be safe
        first_call_ = true;  
      }
      return -1;
    }
  }
  
  // ROS_INFO("Action %s RETURNING", action_name_.c_str());
  return 0;
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

template <class ActionClass, class ActionGoal>
void ExecuteAction<ActionClass, ActionGoal>::fillParameter(
    std::string param_name, int def, int &param_val) {
  if (nh_.hasParam(param_name.c_str())) {
    nh_.getParam(param_name.c_str(), param_val);
  } else {
    ROS_WARN("Param '%s' not set. Using default: %d", param_name.c_str(),
             def);
  }
}
}
#endif
