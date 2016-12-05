#ifndef __INITIAL_DUMMY__
#define __INITIAL_DUMMY__

#include <ros/ros.h>
#include <sarafun_msgs/InitialKeyframeAction.h>
#include <sarafun_generic_al_server/sarafun_test_server.h>

namespace sarafun {
class TestInitial : TestServer<sarafun_msgs::InitialKeyframeAction,
                               sarafun_msgs::InitialKeyframeGoalConstPtr,
                               sarafun_msgs::InitialKeyframeFeedback,
                               sarafun_msgs::InitialKeyframeResult> {
public:
  TestInitial(std::string node_name, std::string actionlib_name)
      : TestServer<
            sarafun_msgs::InitialKeyframeAction,
            sarafun_msgs::InitialKeyframeGoalConstPtr,
            sarafun_msgs::InitialKeyframeFeedback,
            sarafun_msgs::InitialKeyframeResult>::TestServer(node_name,
                                                         actionlib_name) {}

  ~TestInitial() {}

protected:
  bool parseGoal(const sarafun_msgs::InitialKeyframeGoalConstPtr &goal);
};
}
#endif
