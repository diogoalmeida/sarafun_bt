#ifndef __ALIGN_DUMMY__
#define __ALIGN_DUMMY__

#include <ros/ros.h>
#include <sarafun_msgs/AlignKeyframeAction.h>
#include <sarafun_generic_al_server/sarafun_test_server.h>

namespace sarafun {
class TestAlign : TestServer<sarafun_msgs::AlignKeyframeAction,
                               sarafun_msgs::AlignKeyframeGoalConstPtr,
                               sarafun_msgs::AlignKeyframeFeedback,
                               sarafun_msgs::AlignKeyframeResult> {
public:
  TestAlign(std::string node_name, std::string actionlib_name)
      : TestServer<
            sarafun_msgs::AlignKeyframeAction,
            sarafun_msgs::AlignKeyframeGoalConstPtr,
            sarafun_msgs::AlignKeyframeFeedback,
            sarafun_msgs::AlignKeyframeResult>::TestServer(node_name,
                                                         actionlib_name) {}

  ~TestAlign() {}

protected:
  bool parseGoal(const sarafun_msgs::AlignKeyframeGoalConstPtr &goal);
};
}
#endif
