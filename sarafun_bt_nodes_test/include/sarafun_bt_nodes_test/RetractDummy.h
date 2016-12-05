#ifndef __RETRACT_DUMMY__
#define __RETRACT_DUMMY__

#include <ros/ros.h>
#include <sarafun_msgs/RetractKeyframeAction.h>
#include <sarafun_generic_al_server/sarafun_test_server.h>

namespace sarafun {
class TestRetract : TestServer<sarafun_msgs::RetractKeyframeAction,
                               sarafun_msgs::RetractKeyframeGoalConstPtr,
                               sarafun_msgs::RetractKeyframeFeedback,
                               sarafun_msgs::RetractKeyframeResult> {
public:
  TestRetract(std::string node_name, std::string actionlib_name)
      : TestServer<
            sarafun_msgs::RetractKeyframeAction,
            sarafun_msgs::RetractKeyframeGoalConstPtr,
            sarafun_msgs::RetractKeyframeFeedback,
            sarafun_msgs::RetractKeyframeResult>::TestServer(node_name,
                                                         actionlib_name) {}

  ~TestRetract() {}

protected:
  bool parseGoal(const sarafun_msgs::RetractKeyframeGoalConstPtr &goal);
};
}
#endif
