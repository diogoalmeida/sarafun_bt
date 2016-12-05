#ifndef __ASSEMBLED_DUMMY__
#define __ASSEMBLED_DUMMY__

#include <ros/ros.h>
#include <sarafun_msgs/AssembledKeyframeAction.h>
#include <sarafun_generic_al_server/sarafun_test_server.h>

namespace sarafun {
class TestAssembled : TestServer<sarafun_msgs::AssembledKeyframeAction,
                               sarafun_msgs::AssembledKeyframeGoalConstPtr,
                               sarafun_msgs::AssembledKeyframeFeedback,
                               sarafun_msgs::AssembledKeyframeResult> {
public:
  TestAssembled(std::string node_name, std::string actionlib_name)
      : TestServer<
            sarafun_msgs::AssembledKeyframeAction,
            sarafun_msgs::AssembledKeyframeGoalConstPtr,
            sarafun_msgs::AssembledKeyframeFeedback,
            sarafun_msgs::AssembledKeyframeResult>::TestServer(node_name,
                                                         actionlib_name) {}

  ~TestAssembled() {}

protected:
  bool parseGoal(const sarafun_msgs::AssembledKeyframeGoalConstPtr &goal);
};
}
#endif
